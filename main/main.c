#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "audio_pipeline.h"
#include "audio_element.h"
#include "audio_event_iface.h"
#include "i2s_stream.h"
#include "filter_resample.h"
#include "ringbuf.h"
#include "bluetooth_service.h"

#include "app_config.h"

static const char *TAG = "MAIN";

/* Bluedroid's own host-stack task is pinned to core 0
 * (CONFIG_BT_BLUEDROID_PINNED_TO_CORE=0 in sdkconfig), and bt_writer itself
 * has no task of its own (task_stack=-1 - see bluetooth_service_create_
 * stream()'s underlying a2dp_stream_init()): its data gets pulled straight
 * out of resample_filter's ring buffer from inside Bluedroid's own callback
 * context. So "BT streaming" already lives entirely on core 0 without any
 * action here - what needs explicit pinning is everything upstream of it
 * (I2S capture, resampling, and this file's own control task), pinned to
 * core 1 so neither core does both jobs at once. */
#define APP_AUDIO_CPU_CORE 1

/* bt_writer can only ever be created once per process (bluetooth_service_
 * create_stream() errors "Bluetooth stream have been created" on a second
 * call - same restriction as the single-chip project) - created once, then
 * reused across every pipeline rebuild via register/unregister, never
 * deinit'd. i2s_reader has no such restriction and is rebuilt fresh every
 * time. */
static audio_element_handle_t bt_writer;
static SemaphoreHandle_t bt_connected_sem;

/* Forwarded from ADF's own bt_a2d_source_cb before it touches its internal
 * state machine (bluetooth_service.c) - gives us the raw connection-state
 * event without needing esp_periph_set at all. Runs on the Bluedroid task,
 * not bt_task - the connection-state branch stays minimal (no logging/
 * allocation) as before. ESP_A2D_AUDIO_CFG_EVT is the one deliberate
 * exception: it fires once per connection (not per packet), and this exact
 * pattern - ESP_LOGI() directly inside this same Bluedroid-task callback
 * context - is what ESP-ADF's own bt_a2d_sink_cb in a2dp_stream.c already
 * does for the equivalent sink-side event, so it's proven safe here. */
static void bt_a2d_event_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    if (event == ESP_A2D_CONNECTION_STATE_EVT &&
        param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED &&
        bt_connected_sem) {
        xSemaphoreGive(bt_connected_sem);
    } else if (event == ESP_A2D_AUDIO_CFG_EVT && param->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
        /* The real negotiated SBC config with the speaker - not something
         * this project's own code chooses (see RADIO_STREAM_SAMPLE_RATE_HZ's
         * comment in app_config.h: ESP-IDF's Bluedroid SOURCE role only ever
         * advertises 44.1kHz). cie.sbc_info's named bitfields (esp_a2dp_api.h)
         * are the current, non-deprecated way to read this - the raw
         * cie.sbc[] byte array they replaced is __attribute__((deprecated)). */
        const esp_a2d_cie_sbc_t *sbc = &param->audio_cfg.mcc.cie.sbc_info;
        int freq_hz = (sbc->samp_freq & 0x8) ? 16000 : (sbc->samp_freq & 0x4) ? 32000 :
                      (sbc->samp_freq & 0x2) ? 44100 : (sbc->samp_freq & 0x1) ? 48000 : 0;
        const char *ch_mode = (sbc->ch_mode & 0x8) ? "mono" : (sbc->ch_mode & 0x4) ? "dual" :
                              (sbc->ch_mode & 0x2) ? "stereo" : (sbc->ch_mode & 0x1) ? "joint-stereo" : "?";
        ESP_LOGI(TAG, "A2DP negotiated with speaker: SBC %d Hz, %s, bitpool %u-%u",
                 freq_hz, ch_mode, sbc->min_bitpool, sbc->max_bitpool);
    }
}

static esp_err_t start_bluetooth_service(void)
{
    bt_connected_sem = xSemaphoreCreateBinary();
    if (!bt_connected_sem) return ESP_ERR_NO_MEM;

    bluetooth_service_cfg_t cfg = {
        .device_name = RADIO_LOCAL_BT_NAME,
        .remote_name = RADIO_SPEAKER_NAME,
        .mode = BLUETOOTH_A2DP_SOURCE,
        .user_callback = {
            .user_a2d_cb = bt_a2d_event_cb,
        },
    };
    ESP_LOGI(TAG, "Starting A2DP source; scanning for speaker name '%s'", RADIO_SPEAKER_NAME);
    esp_err_t err = bluetooth_service_start(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bluetooth_service_start failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

/* --- I2S audio-level probe ------------------------------------------
 * Answers directly, from onboard evidence: is real audio actually arriving
 * over I2S, or is this chip just relaying silence through to a speaker
 * that's otherwise perfectly connected? Isolates "esp32_wifi_streamer / the
 * physical link isn't sending real PCM" from "something downstream on THIS
 * chip is swallowing it silently" - using only this project's own code.
 *
 * NOT a separate pipeline element (that was tried first and reverted -
 * hardware-tested: even a trivial pass-through element adds its own task
 * plus an extra ring-buffer hop/copy of the full 48kHz stream, and this
 * board's Bluedroid+resample+I2S load is already tight enough on both
 * cores that the extra task starved that core's IDLE task past the 5s
 * task-watchdog timeout - happened on core 1 pinned alongside i2s_reader/
 * resample_filter, then again on core 0 alongside Bluedroid after moving
 * it there. Wherever the extra task went, it broke that core.
 *
 * Instead this taps i2s_reader's OWN existing task via ESP-ADF's write-
 * callback hook (audio_element_set_write_cb) - runs inline, right where
 * i2s_stream would otherwise call rb_write() directly, so there's no new
 * task and no extra buffer hop: same rb_write() call as before, just with
 * a cheap peak-scan added in front of it. Logs once a second: the loudest
 * sample seen in that window, and whether the window counts as silence
 * (peak below a small noise floor) or real signal. */
#define I2S_PROBE_LOG_INTERVAL_US   (1 * 1000 * 1000)
#define I2S_PROBE_SILENCE_THRESHOLD 32 /* ~-66dBFS for 16-bit PCM - comfortably above DC bias/electrical noise, comfortably below any real audio */

static int i2s_probe_write_cb(audio_element_handle_t self, char *buffer, int len,
                              TickType_t ticks_to_wait, void *ctx)
{
    static int16_t peak_since_log = 0;
    static int64_t last_log_us = 0;
    static bool ever_seen_signal = false;
    /* DIAGNOSTIC (slow-playback investigation, see RADIO_STREAM_SAMPLE_RATE_HZ's
     * comment): this chip has no way to MEASURE the master's real bit-clock
     * rate - as I2S slave its own "rate" config is nominal only, actual
     * timing is whatever BCLK/WS the master drives. But every stereo 16-bit
     * frame written here through rb_write() IS one real I2S word-select
     * cycle, so counting frames over a wall-clock window (esp_timer_get_time,
     * not the nominal config) gives the true incoming sample rate directly -
     * no oscilloscope needed. If this reads notably above 48000 Hz, the
     * fixed RADIO_STREAM_SAMPLE_RATE_HZ resample assumption below is wrong
     * and that mismatch alone would explain slower-than-source playback
     * (resampler under-converts real audio duration into the same nominal
     * 44.1kHz output length). */
    static uint32_t frames_since_log = 0;

    if (len > 0) {
        int16_t *samples = (int16_t *)buffer;
        int sample_count = len / (int)sizeof(int16_t);
        for (int i = 0; i < sample_count; i++) {
            int16_t mag = samples[i] < 0 ? (int16_t)(-samples[i]) : samples[i];
            if (mag > peak_since_log) {
                peak_since_log = mag;
            }
        }
        /* Stereo 16-bit: one frame = 2 samples (L+R) = one WS cycle. */
        frames_since_log += sample_count / 2;

        int64_t now_us = esp_timer_get_time();
        if (now_us - last_log_us >= I2S_PROBE_LOG_INTERVAL_US) {
            double elapsed_s = (now_us - last_log_us) / 1e6;
            double measured_rate_hz = elapsed_s > 0 ? (double)frames_since_log / elapsed_s : 0;
            if (peak_since_log > I2S_PROBE_SILENCE_THRESHOLD) {
                ever_seen_signal = true;
                ESP_LOGI(TAG, "I2S PROBE: audio present, peak=%d (%.1f%% of full scale), "
                         "measured rate=%.1f Hz (nominal src=%d Hz, %+.2f%%)",
                         peak_since_log, 100.0f * peak_since_log / 32767.0f, measured_rate_hz,
                         RADIO_STREAM_SAMPLE_RATE_HZ,
                         100.0 * (measured_rate_hz - RADIO_STREAM_SAMPLE_RATE_HZ) / RADIO_STREAM_SAMPLE_RATE_HZ);
            } else {
                ESP_LOGW(TAG, "I2S PROBE: FLAT LINE - no signal above noise floor (peak=%d), measured rate=%.1f Hz%s",
                         peak_since_log, measured_rate_hz, ever_seen_signal ? "" : " - never seen real audio since boot");
            }
            peak_since_log = 0;
            frames_since_log = 0;
            last_log_us = now_us;
        }
    }

    /* Reproduce exactly what audio_element_output() would have done by
     * default (IO_TYPE_RB -> rb_write) - this callback fully replaces that,
     * it doesn't wrap it. */
    return rb_write((ringbuf_handle_t)ctx, buffer, len, ticks_to_wait);
}

static esp_err_t ensure_bt_connected(void)
{
    ESP_LOGI(TAG, "Waiting for a real Bluetooth connection (timeout %d s)...",
             RADIO_BT_CONNECT_TIMEOUT_MS / 1000);
    if (xSemaphoreTake(bt_connected_sem, pdMS_TO_TICKS(RADIO_BT_CONNECT_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Timed out waiting for '%s' to connect", RADIO_SPEAKER_NAME);
        return ESP_ERR_TIMEOUT;
    }
    ESP_LOGI(TAG, "Bluetooth connected");

    if (!bt_writer) {
        bt_writer = bluetooth_service_create_stream();
        if (!bt_writer) {
            ESP_LOGE(TAG, "Could not create Bluetooth stream");
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

/* One I2S-in -> Bluetooth-out pipeline session: builds it, runs it, blocks
 * until an element reports a terminal status (e.g. the I2S master chip
 * reset and the link glitched), then tears down everything except bt_writer
 * (see its declaration comment) so the caller can rebuild and try again
 * without needing a fresh Bluetooth connection each time. */
static esp_err_t run_bridge_session(void)
{
    /* DIAGNOSTIC (slow-playback investigation): unlike wifi_streamer's
     * in-place restart, this pipeline is fully deinit'd and rebuilt fresh
     * every single bridge session (see this function's own header comment),
     * alongside classic-BT's own SBC-frame malloc/free churn at connect
     * time (see esp32-radio-known-issues memory - heap fragmentation from
     * exactly this pattern has caused real allocation failures on this
     * project's single-chip predecessor). Log heap/largest-block right at
     * entry so a slow fragmentation trend, or a rebuild landing on a smaller
     * block than the previous one, is visible across sessions rather than
     * inferred after the fact. */
    ESP_LOGI(TAG, "run_bridge_session start: free heap=%" PRIu32 ", largest block=%" PRIu32,
             esp_get_free_heap_size(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    audio_pipeline_cfg_t pipe_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    audio_pipeline_handle_t pipeline = audio_pipeline_init(&pipe_cfg);
    if (!pipeline) return ESP_ERR_NO_MEM;

    /* I2S SLAVE (this chip only listens - BCLK/WS are inputs, no clock
     * generation) - the companion esp32_wifi_streamer chip drives the bus
     * as master. Pins from app_config.h, must match that project's pins and
     * the physical wiring exactly. */
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT_WITH_PARA(
        I2S_NUM_0, RADIO_I2S_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, AUDIO_STREAM_READER);
    i2s_cfg.chan_cfg.role = I2S_ROLE_SLAVE;
    i2s_cfg.std_cfg.gpio_cfg.bclk = RADIO_I2S_BCLK_GPIO;
    i2s_cfg.std_cfg.gpio_cfg.ws = RADIO_I2S_WS_GPIO;
    i2s_cfg.std_cfg.gpio_cfg.din = RADIO_I2S_DATA_GPIO;
    i2s_cfg.std_cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
    i2s_cfg.std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
    /* Bump past ESP-ADF's generic default - see RADIO_I2S_RINGBUFFER_BYTES's
     * comment in app_config.h for why this chip can afford a much healthier
     * margin here. */
    i2s_cfg.out_rb_size = RADIO_I2S_RINGBUFFER_BYTES;
    /* Keep off Bluedroid's core - see APP_AUDIO_CPU_CORE's comment. */
    i2s_cfg.task_core = APP_AUDIO_CPU_CORE;
    audio_element_handle_t i2s_reader = i2s_stream_init(&i2s_cfg);
    if (!i2s_reader) {
        ESP_LOGE(TAG, "Could not allocate I2S reader element");
        audio_pipeline_deinit(pipeline);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "i2s_reader created (rb_size=%d): free heap=%" PRIu32 ", largest block=%" PRIu32,
             (int)i2s_cfg.out_rb_size, esp_get_free_heap_size(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    /* Resample the real 48kHz PCM (RADIO_STREAM_SAMPLE_RATE_HZ - see its
     * comment in app_config.h) down to 44.1kHz: ESP-IDF's vendored Bluedroid
     * A2DP SOURCE role can only ever negotiate 44.1kHz SBC with the speaker
     * (bta_av_co_sbc_caps in esp-idf's bta_av_co.c only sets
     * A2D_SBC_IE_SAMP_FREQ_44, no 48kHz bit - confirmed against that source
     * directly, and independently against github.com/espressif/esp-idf#5997:
     * a user there patched that struct to unlock 48kHz and it played for a
     * few seconds then froze/hung - not a usable path). Feeding 48kHz PCM to
     * an encoder that treats it as 44.1kHz plays back ~8.8% fast/pitched-up.
     * RESAMPLE_ENCODE_MODE (not the decode-mode default) is used
     * deliberately: this chip has no decoder of its own to report real music
     * info the way decode-mode's auto src-rate detection relies on - i2s_
     * reader is a slave with no rate-sensing hardware, so any info it might
     * report would just reflect its own nominal RADIO_I2S_SAMPLE_RATE config
     * (44100), which is wrong. Encode mode takes src_rate as a fixed,
     * explicitly-correct value instead of trying to infer it. */
    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.mode        = RESAMPLE_ENCODE_MODE;
    /* RADIO_STREAM_ACTUAL_RATE_HZ, not the nominal RADIO_STREAM_SAMPLE_RATE_HZ
     * - see that constant's comment in app_config.h (JITTER FIX): the master's
     * real I2S clock runs ~0.15% off nominal, and encode-mode resampling needs
     * the true rate here to avoid a systematic input/output drift that drains
     * bt_writer's ring buffer and causes audible jitter. */
    rsp_cfg.src_rate    = RADIO_STREAM_ACTUAL_RATE_HZ;
    rsp_cfg.src_ch      = 2;
    rsp_cfg.src_bits    = 16;
    rsp_cfg.dest_rate   = RADIO_I2S_SAMPLE_RATE;
    rsp_cfg.dest_ch     = 2;
    rsp_cfg.dest_bits   = 16;
    rsp_cfg.out_rb_size = RADIO_RESAMPLE_RINGBUFFER_BYTES;
    rsp_cfg.task_core   = APP_AUDIO_CPU_CORE;
    /* No PSRAM on this board (WROOM-32U) - audio_thread_create() falls back
     * to internal memory safely either way, but say so explicitly rather
     * than relying on that silent fallback. */
    rsp_cfg.stack_in_ext = false;
    audio_element_handle_t resample_filter = rsp_filter_init(&rsp_cfg);
    if (!resample_filter) {
        ESP_LOGE(TAG, "Could not allocate resample filter element");
        audio_element_deinit(i2s_reader);
        audio_pipeline_deinit(pipeline);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "resample_filter created (src=%d dest=%d, rb_size=%d): free heap=%" PRIu32 ", largest block=%" PRIu32,
             (int)rsp_cfg.src_rate, (int)rsp_cfg.dest_rate, (int)rsp_cfg.out_rb_size,
             esp_get_free_heap_size(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    esp_err_t err = ESP_OK;
    err |= audio_pipeline_register(pipeline, i2s_reader, "i2s");
    err |= audio_pipeline_register(pipeline, resample_filter, "rsp");
    err |= audio_pipeline_register(pipeline, bt_writer, "bt");
    const char *links[] = {"i2s", "rsp", "bt"};
    err |= audio_pipeline_link(pipeline, links, 3);
    /* Tap i2s_reader's own task for the I2S PROBE logging - see
     * i2s_probe_write_cb()'s header comment for why this replaces a separate
     * pipeline element. Must happen AFTER audio_pipeline_link() above: that
     * call is what creates and wires the real ring buffer between i2s_reader
     * and resample_filter in the first place (from i2s_cfg.out_rb_size) -
     * this fetches that same buffer and hands it to the callback as its
     * write target, so nothing about the actual data path changes. */
    if (err == ESP_OK) {
        ringbuf_handle_t i2s_to_rsp_rb = audio_element_get_output_ringbuf(i2s_reader);
        err = audio_element_set_write_cb(i2s_reader, i2s_probe_write_cb, i2s_to_rsp_rb);
    }

    audio_event_iface_cfg_t event_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t event_iface = audio_event_iface_init(&event_cfg);
    if (!event_iface) {
        ESP_LOGE(TAG, "Could not allocate ADF event interface");
        err = ESP_ERR_NO_MEM;
    } else {
        err |= audio_pipeline_set_listener(pipeline, event_iface);
    }

    if (err == ESP_OK) {
        err = audio_pipeline_run(pipeline);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start I2S->resample->Bluetooth pipeline: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "I2S -> resample (%d->%d Hz) -> Bluetooth A2DP pipeline running",
                 RADIO_STREAM_SAMPLE_RATE_HZ, RADIO_I2S_SAMPLE_RATE);

        while (true) {
            audio_event_iface_msg_t msg;
            esp_err_t listen_err = audio_event_iface_listen(event_iface, &msg, portMAX_DELAY);
            if (listen_err != ESP_OK) {
                continue;
            }
            if (msg.source_type != AUDIO_ELEMENT_TYPE_ELEMENT || msg.cmd != AEL_MSG_CMD_REPORT_STATUS) {
                continue;
            }
            int status = (int)(intptr_t)msg.data;
            if (status == AEL_STATUS_ERROR_OPEN || status == AEL_STATUS_ERROR_INPUT ||
                status == AEL_STATUS_ERROR_PROCESS || status == AEL_STATUS_ERROR_OUTPUT ||
                status == AEL_STATUS_ERROR_CLOSE || status == AEL_STATUS_ERROR_TIMEOUT ||
                status == AEL_STATUS_ERROR_UNKNOWN || status == AEL_STATUS_STATE_STOPPED ||
                status == AEL_STATUS_STATE_FINISHED) {
                ESP_LOGW(TAG, "Pipeline element reported status=%d; session needs replacing", status);
                err = ESP_FAIL;
                break;
            }
        }
    }

    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);
    audio_pipeline_unregister(pipeline, i2s_reader);
    audio_pipeline_unregister(pipeline, resample_filter);
    audio_pipeline_unregister(pipeline, bt_writer);
    if (event_iface) {
        audio_pipeline_remove_listener(pipeline);
        audio_event_iface_destroy(event_iface);
    }
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(i2s_reader);
    /* resample_filter has no "once per process" restriction either - same
     * as i2s_reader, safe to fully deinit and recreate fresh every session. */
    audio_element_deinit(resample_filter);
    /* bt_writer is intentionally NOT deinit'd - see its declaration comment.
     * Needs its state force-reset to AEL_STATE_INIT for reuse - see the
     * single-chip project's radio_pipeline.c for why this specific call is
     * required for a task_stack<=0 element like this one. */
    audio_element_reset_state(bt_writer);

    return err;
}

static void bt_task(void *pvParameters)
{
    ESP_ERROR_CHECK(start_bluetooth_service());

    while (true) {
        esp_err_t err = ensure_bt_connected();
        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        err = run_bridge_session();
        ESP_LOGW(TAG, "Bridge session ended: %s; rebuilding", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MAIN", ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "ESP32 Bluetooth bridge chip booting (I2S in <- esp32_wifi_streamer)");
    ESP_LOGI(TAG, "Target speaker='%s'", RADIO_SPEAKER_NAME);
    ESP_LOGI(TAG, "Free heap at boot=%" PRIu32, esp_get_free_heap_size());

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* bt_task itself only builds the pipeline and blocks on the ADF event
     * queue - cheap - but pin it alongside i2s_reader/resample_filter on
     * APP_AUDIO_CPU_CORE anyway, purely so nothing belonging to this file's
     * control flow ever lands on Bluedroid's core 0 by scheduler luck. */
    BaseType_t result = xTaskCreatePinnedToCore(
        bt_task,
        "bt_task",
        8192,
        NULL,
        5,
        NULL,
        APP_AUDIO_CPU_CORE
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create bt_task");
        abort();
    }
}
