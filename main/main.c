#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/uart.h"
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
/* Not for configuring anything - only to READ BACK the pins ADF is really
 * going to use, so check_i2s_pins() below can catch a silent mismatch.
 * Available without a CMakeLists change: audio_stream lists audio_board in
 * its own public COMPONENT_REQUIRES. */
#include "board_pins_config.h"

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

/* Last absolute volume (0-127) reported to us via
 * ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT - see avrc_tg_event_cb() below.
 * This bridge chip has no volume control of its own to actually apply, but
 * ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT's INTERIM response must echo back
 * *some* current volume value for the AVRCP notification handshake to be
 * protocol-correct, so the last value the speaker itself told us is the
 * only honest one to hand back. Starts at max (127) - an arbitrary but
 * harmless default since this chip never actually attenuates audio. */
static uint8_t s_avrc_tg_volume = 127;

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

/* --- AVRCP -> UART command bridge -------------------------------------
 * This chip is an A2DP SOURCE, so the natural AVRCP counterpart role is
 * TARGET (TG): the connected speaker acts as AVRCP CONTROLLER and can send
 * this chip transport commands (play/pause/stop/next/previous/rewind/
 * fast-forward) and absolute-volume-set commands, e.g. from its own
 * physical/remote controls.
 *
 * esp_avrc_tg_init() itself is NOT called here: it lives in the shared,
 * vendored bluetooth_service.c one level up (grep it for esp_avrc_tg_init),
 * called from inside bluetooth_service_start(). That placement is load-
 * bearing, not incidental - esp_avrc_api.h's own doc comment for
 * esp_avrc_tg_init() requires AVRC to be initialized BEFORE A2DP, and
 * bluetooth_service_start() is what calls esp_a2d_source_init() and kicks
 * off discovery. A TG init living in this file, called after
 * start_bluetooth_service() returns, would always run too late, and the
 * observed symptom of that (on the sibling esp32_bt_speaker_48khz project,
 * 2026-08-24) is AVRCP connecting fine while passthrough commands never
 * reach the app callback at all. Only the callback registration and the
 * supported-command filter live here - those are order-sensitive relative
 * to esp_avrc_tg_init(), not to A2DP.
 *
 * See RADIO_AVRC_UART_PORT's comment in app_config.h for the wire format
 * and for why every command received here is forwarded over a dedicated
 * UART link to esp32_wifi_streamer: that's the chip actually driving
 * playback, so it's the one that can act on these, not this bridge chip. */

/* esp_avrc_pt_cmd_t (see esp_avrc_api.h) covers far more than audio
 * transport controls - TV/menu-navigation codes, numeric keypad codes, etc.
 * Only the subset a Bluetooth speaker's remote/physical controls could
 * plausibly send is named here; anything else is logged/forwarded as its
 * raw hex code (see avrc_tg_event_cb() below) rather than silently dropped,
 * since "AVRCP has more real world variety than one lookup table" is exactly
 * the kind of thing worth surfacing instead of hiding. */
static const char *avrc_pt_cmd_name(uint8_t key_code)
{
    switch (key_code) {
    case ESP_AVRC_PT_CMD_PLAY:         return "PLAY";
    case ESP_AVRC_PT_CMD_PAUSE:        return "PAUSE";
    case ESP_AVRC_PT_CMD_STOP:         return "STOP";
    case ESP_AVRC_PT_CMD_FORWARD:      return "NEXT";
    case ESP_AVRC_PT_CMD_BACKWARD:     return "PREVIOUS";
    case ESP_AVRC_PT_CMD_REWIND:       return "REWIND";
    case ESP_AVRC_PT_CMD_FAST_FORWARD: return "FAST_FORWARD";
    case ESP_AVRC_PT_CMD_VOL_UP:       return "VOL_UP";
    case ESP_AVRC_PT_CMD_VOL_DOWN:     return "VOL_DOWN";
    case ESP_AVRC_PT_CMD_MUTE:         return "MUTE";
    default:                           return NULL;
    }
}

/* Formats and sends one line of the wire format documented next to
 * RADIO_AVRC_UART_PORT in app_config.h. tx_buffer_size > 0 (see
 * init_avrc_uart()) means this returns as soon as the line is copied into
 * the driver's TX ring buffer - it does not block waiting for the bytes to
 * actually go out over the wire, so it's safe to call directly from
 * avrc_tg_event_cb() below even though that runs on Bluedroid's own task. */
static void avrc_uart_send_line(const char *fmt, ...)
{
    char line[80];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return;
    }
    size_t len = (n < (int)sizeof(line)) ? (size_t)n : sizeof(line) - 1;
    uart_write_bytes(RADIO_AVRC_UART_PORT, line, len);
}

/* AVRCP TARGET event callback - see this section's header comment for why
 * this chip is TG rather than CT. Runs on Bluedroid's own host-stack task,
 * same context/safety reasoning as bt_a2d_event_cb() above: ESP_LOGI()
 * directly inside a Bluedroid callback is the same pattern ESP-ADF's own
 * a2dp_stream.c uses for the equivalent event, and avrc_uart_send_line()
 * above is non-blocking, so nothing here risks stalling the BT stack. */
static void avrc_tg_event_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_TG_CONNECTION_STATE_EVT: {
        bool connected = param->conn_stat.connected;
        ESP_LOGI(TAG, "AVRCP %s", connected ? "connected" : "disconnected");
        avrc_uart_send_line("AVRCP:CONN:%s\n", connected ? "CONNECTED" : "DISCONNECTED");
        break;
    }
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT: {
        uint8_t key_code = param->psth_cmd.key_code;
        const char *state = (param->psth_cmd.key_state == ESP_AVRC_PT_CMD_STATE_PRESSED) ? "PRESSED" : "RELEASED";
        const char *name = avrc_pt_cmd_name(key_code);
        char hex_name[8];
        if (!name) {
            snprintf(hex_name, sizeof(hex_name), "0x%02X", key_code);
            name = hex_name;
        }
        ESP_LOGI(TAG, "AVRCP command received: %s (%s)", name, state);
        avrc_uart_send_line("AVRCP:CMD:%s:%s\n", name, state);
        break;
    }
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT: {
        uint8_t volume = param->set_abs_vol.volume; /* 0-127, see esp_avrc_api.h */
        s_avrc_tg_volume = volume;
        ESP_LOGI(TAG, "AVRCP command received: SET_VOLUME %u/127 (%.0f%%)",
                 volume, 100.0f * volume / 127.0f);
        avrc_uart_send_line("AVRCP:VOL:%u\n", (unsigned)volume);
        break;
    }
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT: {
        /* Only VOLUME_CHANGE is advertised as supported (see
         * esp_avrc_tg_set_rn_evt_cap() in start_avrc_bridge()) - must send
         * an INTERIM response with the current volume or the speaker may
         * treat this AVRCP target as not actually supporting the
         * notification and stop syncing volume over AVRCP at all. Not a
         * "command received" in its own right (no PRESSED/RELEASED, nothing
         * for esp32_wifi_streamer to act on), so this isn't forwarded over
         * UART - only logged. */
        if (param->reg_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
            ESP_LOGD(TAG, "AVRCP registered for volume-change notifications");
            esp_avrc_rn_param_t rn_param = { .volume = s_avrc_tg_volume };
            esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_INTERIM, &rn_param);
        }
        break;
    }
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT:
        ESP_LOGD(TAG, "AVRC remote features %" PRIx32 ", CT features 0x%x",
                 param->rmt_feats.feat_mask, param->rmt_feats.ct_feat_flag);
        break;
    case ESP_AVRC_TG_PROF_STATE_EVT:
        ESP_LOGD(TAG, "AVRCP TG profile state: %d", param->avrc_tg_init_stat.state);
        break;
    default:
        ESP_LOGD(TAG, "Unhandled AVRC TG event: %d", event);
        break;
    }
}

/* One-way UART for forwarding AVRCP commands to esp32_wifi_streamer - see
 * RADIO_AVRC_UART_PORT's comment in app_config.h. A small nonzero
 * tx_buffer_size (rather than 0) is deliberate - see avrc_uart_send_line()'s
 * comment for why. rx_buffer_size is likewise nonzero purely because
 * uart_driver_install() requires one even when, as here, no RX pin is ever
 * configured (UART_PIN_NO_CHANGE below) and nothing is ever read back. */
static void init_avrc_uart(void)
{
    const uart_config_t cfg = {
        .baud_rate = RADIO_AVRC_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(RADIO_AVRC_UART_PORT, 256, 256, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RADIO_AVRC_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(RADIO_AVRC_UART_PORT, RADIO_AVRC_UART_TX_GPIO,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "AVRCP->UART bridge: UART%d TX=GPIO%d @ %d baud",
             RADIO_AVRC_UART_PORT, RADIO_AVRC_UART_TX_GPIO, RADIO_AVRC_UART_BAUD);
}

/* Called once from bt_task, not per bridge session: unlike i2s_reader/
 * resample_filter, AVRCP TG has no per-session state to rebuild - it's tied
 * to the Bluetooth connection itself, which bluetooth_service already
 * manages independently of run_bridge_session()'s pipeline churn. Must be
 * called AFTER start_bluetooth_service(), which is what runs
 * esp_avrc_tg_init() (see this section's header comment). */
static esp_err_t start_avrc_bridge(void)
{
    init_avrc_uart();

    ESP_ERROR_CHECK(esp_avrc_tg_register_callback(avrc_tg_event_cb));

    /* Bluedroid's default AVRCP TG "supported passthrough commands" set is
     * all-zero (cs_psth_dft_supported_cmd[8] = {0} in ESP-IDF's btc_avrc.c)
     * until this is called - every passthrough command a remote sends is
     * checked against that set in bta_av_act.c's bta_av_op_supported() (via
     * bta_avrc_co.c's rc_cmd() -> btc_avrc_tg_get_supported_command())
     * BEFORE it ever reaches our app callback: a command not in the set gets
     * an automatic "NOT IMPLEMENTED" response at the stack level and
     * BTA_AV_REMOTE_CMD_EVT is never even raised, so avrc_tg_event_cb()
     * never fires. Without this call the whole bridge is silently deaf to
     * every button press - hardware-confirmed on the sibling
     * esp32_bt_speaker_48khz project (2026-08-23): "AVRCP connected" and the
     * remote-features log both appeared, passthrough commands never did.
     * Explicitly opt in to the transport/volume commands this bridge
     * actually forwards (the same set avrc_pt_cmd_name() names, so
     * "supported" and "named on the wire" stay in sync). */
    esp_avrc_psth_bit_mask_t psth_set = {0};
    esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth_set, ESP_AVRC_PT_CMD_PLAY);
    esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth_set, ESP_AVRC_PT_CMD_PAUSE);
    esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth_set, ESP_AVRC_PT_CMD_STOP);
    esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth_set, ESP_AVRC_PT_CMD_FORWARD);
    esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth_set, ESP_AVRC_PT_CMD_BACKWARD);
    esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth_set, ESP_AVRC_PT_CMD_REWIND);
    esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth_set, ESP_AVRC_PT_CMD_FAST_FORWARD);
    esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth_set, ESP_AVRC_PT_CMD_VOL_UP);
    esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth_set, ESP_AVRC_PT_CMD_VOL_DOWN);
    esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth_set, ESP_AVRC_PT_CMD_MUTE);
    ESP_ERROR_CHECK(esp_avrc_tg_set_psth_cmd_filter(ESP_AVRC_PSTH_FILTER_SUPPORTED_CMD, &psth_set));

    /* Advertise this target as able to report volume changes - lets the
     * speaker keep its own volume in sync via AVRCP absolute volume instead
     * of only its local physical knob. Matches the pattern ESP-IDF's own
     * classic_bt examples use (esp_avrc_rn_evt_cap_mask_t + bitmask
     * helper), not something specific to this project. */
    esp_avrc_rn_evt_cap_mask_t evt_set = {0};
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
    ESP_ERROR_CHECK(esp_avrc_tg_set_rn_evt_cap(&evt_set));

    ESP_LOGI(TAG, "AVRCP target ready - commands from '%s' will be logged and forwarded over UART", RADIO_SPEAKER_NAME);
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

/*
 * 2026-09-04: this callback now does a second job alongside the logging
 * above - see RADIO_I2S_RATE_CANDIDATE_LOW_HZ's comment in app_config.h.
 * esp32_wifi_streamer no longer fixes its I2S output at one rate (some
 * stations are 44.1kHz, some 48kHz), and this chip has no back-channel to
 * be TOLD which - so it measures the real incoming rate itself, the exact
 * same frame-counting this callback already did for the log line, and
 * retargets resample_filter's source rate live via
 * rsp_filter_change_src_info() whenever the classified rate changes.
 * Bundled into ONE small static struct (not heap-allocated - this board
 * has ~220KB usable RAM total, no PSRAM, every byte counted) passed as the
 * write-callback's ctx, replacing the plain ringbuf_handle_t ctx used
 * before.
 */
typedef struct {
    ringbuf_handle_t        out_rb;          /* what this callback still forwards every buffer to, unchanged */
    audio_element_handle_t  resample_filter; /* NULL until run_bridge_session() creates it */
    int                     applied_src_rate; /* what resample_filter is currently configured for, so this only calls rsp_filter_change_src_info() on an actual change */
} i2s_probe_ctx_t;

/* File-scope, not heap-allocated (this board has ~220KB usable RAM total,
 * no PSRAM) - repopulated at the top of every run_bridge_session() call,
 * read for the lifetime of that session's i2s_reader task. */
static i2s_probe_ctx_t s_i2s_probe_ctx;

static int i2s_probe_write_cb(audio_element_handle_t self, char *buffer, int len,
                              TickType_t ticks_to_wait, void *ctx_ptr)
{
    i2s_probe_ctx_t *ctx = (i2s_probe_ctx_t *)ctx_ptr;
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

            /* Classify the measured rate to the nearer of the two real
             * candidates esp32_wifi_streamer can actually send (see
             * RADIO_I2S_RATE_CANDIDATE_LOW_HZ's comment in app_config.h) and
             * retarget the resampler if that differs from what it is
             * currently set to. Guarded on a real signal-carrying window -
             * see this struct's own comment - a near-zero frame count (I2S
             * link not up yet, or a brief master-side reset) would otherwise
             * misclassify as whichever candidate is numerically closer to
             * zero, corrupting a perfectly good existing configuration for
             * no reason. Deliberately independent of ever_seen_signal/the
             * silence check above - clock activity (frames arriving at all)
             * is what this needs, not signal amplitude; WS toggles every
             * frame regardless of whether the PCM value is silence. */
            if (ctx->resample_filter && frames_since_log > (uint32_t)(RADIO_I2S_RATE_CANDIDATE_LOW_HZ / 10)) {
                int classified_rate = (measured_rate_hz < RADIO_I2S_RATE_CLASSIFY_MIDPOINT_HZ)
                                       ? RADIO_I2S_RATE_CANDIDATE_LOW_HZ : RADIO_I2S_RATE_CANDIDATE_HIGH_HZ;
                if (classified_rate != ctx->applied_src_rate) {
                    esp_err_t rsp_err = rsp_filter_change_src_info(ctx->resample_filter, classified_rate, 2, 16);
                    if (rsp_err == ESP_OK) {
                        ESP_LOGI(TAG, "I2S PROBE: incoming rate classified as %d Hz (measured %.1f Hz) - "
                                 "resampler retargeted (was %d Hz)%s",
                                 classified_rate, measured_rate_hz, ctx->applied_src_rate,
                                 classified_rate == RADIO_BT_SAMPLE_RATE_HZ
                                     ? " - matches the BT output rate, effectively a passthrough" : "");
                        ctx->applied_src_rate = classified_rate;
                    } else {
                        ESP_LOGW(TAG, "I2S PROBE: rsp_filter_change_src_info(%d Hz) failed: %s",
                                 classified_rate, esp_err_to_name(rsp_err));
                    }
                }
            }

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
    return rb_write(ctx->out_rb, buffer, len, ticks_to_wait);
}

/* --- I2S pin verification ---------------------------------------------
 * The three RADIO_I2S_*_GPIO defines in app_config.h, and the
 * i2s_cfg.std_cfg.gpio_cfg assignments in run_bridge_session() below, DO NOT
 * reach the hardware on their own. On IDF >= 5.0 the audio_stream component
 * compiles i2s_stream_idf5.c, whose i2s_driver_startup() calls the selected
 * audio_board's get_i2s_pins() and memcpy()s the result straight over
 * rx_std_cfg.gpio_cfg - unconditionally, immediately before
 * i2s_channel_init_std_mode(). ADF exposes no API to override that or to
 * reach the i2s_chan_handle_t afterwards. The board wins, always.
 *
 * That makes the pins a TWO-PLACE setting (app_config.h, which humans read,
 * and esp-adf/components/audio_board/lyrat_v4_2/board_pins_config.c, which
 * the hardware obeys) whose two halves can drift apart with no compile
 * error, no runtime error, and no symptom other than a dead bus. It has
 * already happened twice in this project family:
 *
 *   - This chip, until 2026-08-31: no CONFIG_*_BOARD was set at all, so
 *     ADF's Kconfig default (LyraT v4.3) applied and this chip was really
 *     listening on bck 5 / ws 25 / din 35. Only ws matched, by coincidence.
 *   - ../esp32_wifi_streamer, same day: was driving the stock M5Stack
 *     AtomS3R pins bck 8 / ws 6 / dout 5 - note 5 and 6 both present but in
 *     SWAPPED roles, so even a scope probe looks almost-right.
 *
 * Both were found only by disassembling get_i2s_pins in the built .elf. The
 * esp-adf tree those edits live in is vendored and shared by every project
 * here, so one re-clone or upstream update reverts them just as silently -
 * and that is not hypothetical either: ../esp32_wifi_streamer_520kbram and
 * ...-multistation select LyraT v4.3, whose board file is still stock, so
 * they drive bck 5 / ws 25 / dout 26 (plus an MCLK on GPIO0, a strapping
 * pin) rather than the 26/25/27 their own app_config.h documents.
 *
 * So: read the pins back from the same function the driver will call, and
 * refuse to build a pipeline that would quietly clock the wrong GPIOs.
 * Costs one call at session start and turns a scope-required failure into
 * one line of boot log. */
static esp_err_t check_i2s_pins(void)
{
    board_i2s_pin_t pins = { 0 };
    esp_err_t err = get_i2s_pins(I2S_NUM_0, &pins);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "get_i2s_pins(I2S_NUM_0) failed: %s - cannot verify the I2S bus",
                 esp_err_to_name(err));
        return err;
    }

    /* data_out is deliberately unchecked: this is an RX-only slave, and IDF
     * gates strictly on handle->dir, so whatever the board reports for the
     * unused direction is ignored. mclk must be -1 though - a board that
     * hands back a real MCLK pin (LyraT v4.3 returns GPIO0) would claim a
     * GPIO this link neither needs nor wires. */
    bool ok = pins.bck_io_num  == RADIO_I2S_BCLK_GPIO &&
              pins.ws_io_num   == RADIO_I2S_WS_GPIO   &&
              pins.data_in_num == RADIO_I2S_DATA_GPIO &&
              pins.mck_io_num  == -1;

    if (!ok) {
        ESP_LOGE(TAG, "I2S PIN MISMATCH - the selected audio_board's get_i2s_pins() "
                      "does not match app_config.h, and the board is what the hardware obeys.");
        ESP_LOGE(TAG, "  board says : bck=%d ws=%d din=%d mclk=%d",
                 pins.bck_io_num, pins.ws_io_num, pins.data_in_num, pins.mck_io_num);
        ESP_LOGE(TAG, "  expected   : bck=%d ws=%d din=%d mclk=-1",
                 RADIO_I2S_BCLK_GPIO, RADIO_I2S_WS_GPIO, RADIO_I2S_DATA_GPIO);
        ESP_LOGE(TAG, "  fix        : esp-adf/components/audio_board/lyrat_v4_2/board_pins_config.c "
                      "(and check CONFIG_ESP_LYRAT_V4_2_BOARD is still set in sdkconfig)");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "I2S pins verified against the audio_board the driver actually uses: "
                  "bck=%d ws=%d din=%d, no mclk (slave/RX)",
             pins.bck_io_num, pins.ws_io_num, pins.data_in_num);
    return ESP_OK;
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

    /* Before anything else: confirm the pins the driver will really use are
     * the ones this project thinks it wired. See check_i2s_pins(). */
    esp_err_t pin_err = check_i2s_pins();
    if (pin_err != ESP_OK) {
        return pin_err;
    }

    audio_pipeline_cfg_t pipe_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    audio_pipeline_handle_t pipeline = audio_pipeline_init(&pipe_cfg);
    if (!pipeline) return ESP_ERR_NO_MEM;

    /* I2S SLAVE (this chip only listens - BCLK/WS are inputs, no clock
     * generation) - the companion esp32_wifi_streamer chip drives the bus
     * as master. Pins from app_config.h, must match that project's pins and
     * the physical wiring exactly. */
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT_WITH_PARA(
        I2S_NUM_0, RADIO_STREAM_SAMPLE_RATE_HZ, I2S_DATA_BIT_WIDTH_16BIT, AUDIO_STREAM_READER);
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

    /* Resample to 44.1kHz for SBC/A2DP - see RADIO_BT_SAMPLE_RATE_HZ's
     * comment in app_config.h for why that side is fixed (ESP-IDF's
     * vendored Bluedroid A2DP SOURCE role can only ever negotiate 44.1kHz
     * SBC, full stop). The SOURCE side is no longer assumed fixed, though:
     * esp32_wifi_streamer sends either 44.1kHz or 48kHz depending on the
     * station, with no way for this chip to be told which up front - see
     * RADIO_I2S_RATE_CANDIDATE_LOW_HZ's comment. src_rate below is only
     * this element's STARTING guess (same 48000 default as before, the
     * more common case in practice); i2s_probe_write_cb() corrects it live,
     * within the first ~1s of real I2S clock activity, via
     * rsp_filter_change_src_info() - which is what RESAMPLE_ENCODE_MODE (not
     * the decode-mode default) exists to take a live update to: this chip
     * has no decoder of its own to report real music info the way decode
     * mode's auto src-rate detection relies on - i2s_reader is a slave with
     * no rate-sensing hardware of its own, so decode mode would only ever
     * see i2s_reader's own nominal RADIO_STREAM_SAMPLE_RATE_HZ config, not
     * the real incoming rate. Encode mode takes src_rate as an explicit
     * value this file controls directly instead. */
    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.mode        = RESAMPLE_ENCODE_MODE;
    /* Flat 48000 (RADIO_STREAM_SAMPLE_RATE_HZ), NOT the jitter-measured
     * RADIO_STREAM_ACTUAL_RATE_HZ (47928) - confirmed on-device that 47928
     * isn't just imprecise, it's rejected outright: "FIR_RESAMPLE: sample
     * rate is out of range [8000 96000], set input samplerate = 47928 :
     * output sample rate = 44100" / "Failed to create the resample handler",
     * which aborts the pipeline immediately (AEL_STATUS_ERROR_OPEN) - no
     * audio plays at all. The resampler's [8000, 96000] check is misleading
     * (47928 is inside that numeric range); in practice it wants a rate off
     * its own supported table (matching RADIO_I2S_RATE_CANDIDATE_LOW_HZ/
     * _HIGH_HZ's own reasoning: classify to a known-good candidate, never
     * pass a raw measurement straight through). See RADIO_STREAM_ACTUAL_
     * RATE_HZ's comment in app_config.h for the measured clock-drift data -
     * kept for reference, not wired in here anymore. */
    rsp_cfg.src_rate    = RADIO_STREAM_SAMPLE_RATE_HZ;
    rsp_cfg.src_ch      = 2;
    rsp_cfg.src_bits    = 16;
    rsp_cfg.dest_rate   = RADIO_BT_SAMPLE_RATE_HZ;
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
        /* Reset fresh every session (this whole pipeline, resample_filter
         * included, is rebuilt from scratch each time - see this function's
         * own header comment) - applied_src_rate starts at whatever
         * rsp_cfg.src_rate was just created with above, so the classifier
         * only logs/retargets on an ACTUAL change from that starting point,
         * not a redundant one on the very first window. */
        s_i2s_probe_ctx.out_rb = audio_element_get_output_ringbuf(i2s_reader);
        s_i2s_probe_ctx.resample_filter = resample_filter;
        s_i2s_probe_ctx.applied_src_rate = (int)rsp_cfg.src_rate;
        err = audio_element_set_write_cb(i2s_reader, i2s_probe_write_cb, &s_i2s_probe_ctx);
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
        ESP_LOGI(TAG, "I2S -> resample (starting guess %d Hz, live-corrected -> %d Hz) -> Bluetooth A2DP "
                 "pipeline running", (int)rsp_cfg.src_rate, RADIO_BT_SAMPLE_RATE_HZ);

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
    ESP_ERROR_CHECK(start_avrc_bridge());

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
