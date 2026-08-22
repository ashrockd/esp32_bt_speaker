#pragma once

/*
 * User configuration for the Bluetooth chip - the other half of the
 * two-ESP32 split of esp32_tunein_bt_radio. This chip has no Wi-Fi/mbedTLS
 * at all: it only listens for PCM over I2S (as slave - see
 * ../../esp32_wifi_streamer, the I2S master) and streams it out to the
 * Bluetooth speaker over classic A2DP. All the RAM this board has is
 * available to Bluedroid alone.
 */
#define RADIO_SPEAKER_NAME          "EDIFIER R1280DB"
#define RADIO_LOCAL_BT_NAME         "ESP32-BT-Bridge"

/* How long to wait for the target speaker to connect before giving up on
 * one attempt and retrying discovery (bluetooth_service keeps scanning in
 * the background regardless - see the single-chip project's history of why
 * this is deliberately generous: discovery has taken 20-60+ seconds in
 * testing, the speaker isn't always found on the first inquiry cycle). */
#define RADIO_BT_CONNECT_TIMEOUT_MS (3 * 60 * 1000)

/* I2S input from the Wi-Fi/streamer chip - this chip is the I2S SLAVE
 * (BCLK/WS are inputs, DIN only - no clock generation). Must match
 * ../../esp32_wifi_streamer's app_config.h pin numbers exactly, and the
 * physical wiring between the two boards. No MCLK wire needed for a direct
 * ESP32-to-ESP32 link. */
#define RADIO_I2S_BCLK_GPIO         26
#define RADIO_I2S_WS_GPIO           25
#define RADIO_I2S_DATA_GPIO         27
#define RADIO_I2S_SAMPLE_RATE       44100

/* Real physical rate esp32_wifi_streamer's master actually drives on this
 * bus. That project's radio_pipeline.c retunes its own I2S clock (via
 * i2s_stream_set_clk()) to match whatever its AAC decoder reports, and for
 * this fixed TuneIn station the CMAF init segment always parses as 48kHz -
 * see that project's app_config.h/radio_pipeline.c history. RADIO_I2S_
 * SAMPLE_RATE above is only THIS chip's own I2S peripheral's nominal config
 * value - harmless/cosmetic since a slave's actual sample rate is set
 * entirely by the master's BCLK/WS edges, not by this field - so it's left
 * at 44100 rather than touched. This is the real number the resample stage
 * below needs to know about.
 *
 * Why resample here and not fix it upstream: esp32_wifi_streamer is
 * already memory-constrained at runtime (documented OOM history in its own
 * app_config.h) and must not be given another buffer-hungry pipeline stage.
 * This chip, by contrast, is Bluedroid-only with ~200KB+ free heap at boot
 * and nothing else competing for it - the right place to absorb the
 * conversion. */
#define RADIO_STREAM_SAMPLE_RATE_HZ (48000)

/* JITTER FIX (2026-08-22): the value above is the CONTENT's nominal rate
 * (correct - AAC really is 48kHz), but it is NOT what the I2S PROBE actually
 * measures arriving on the wire. On-device logs consistently show
 * measured rate=47927.8 Hz (-0.15% vs 48000) as the steady-state reading,
 * with an occasional 48236.9 Hz (+0.49%) reading that recurs once every
 * several 1s log windows - too frequent to be a real clock change (the
 * master only reconfigures its I2S clock on a pipeline restart, which per
 * esp32_wifi_streamer's own logs happens every ~2-3 minutes, not every other
 * second) and exactly consistent with a one-DMA-chunk phase artifact of the
 * 1-second logging window itself. So the true, physically stable rate
 * esp32_wifi_streamer's I2S peripheral actually drives this bus at is
 * ~47928 Hz, not exactly 48000 Hz - an ordinary ESP32 I2S clock-divider
 * quantization error for a 48kHz target, not a bug on that chip.
 *
 * RESAMPLE_ENCODE_MODE takes src_rate as a fixed, trusted value (see its own
 * comment below) - feeding it the nominal 48000 here means resample_filter
 * produces slightly less audio than Bluedroid consumes (real input arrives
 * ~0.15% slower than assumed), so the ring buffer between resample_filter
 * and bt_writer drains a little further every second. bt_writer's read of
 * that buffer blocks with no timeout (portMAX_DELAY, confirmed in ESP-ADF's
 * audio_element.c) from directly inside Bluedroid's own real-time data-
 * request callback (bt_a2d_source_data_cb in a2dp_stream.c) - so once the
 * buffer runs dry, that stall is what leaks out as the reported "audible
 * jitter", recurring roughly every time the buffer finishes draining rather
 * than constantly. Using the true measured rate here removes that systematic
 * drift instead of just making the buffer bigger (which only delays the same
 * problem). If a future measurement shows a different steady-state rate,
 * update this constant to match rather than RADIO_STREAM_SAMPLE_RATE_HZ
 * above, which should stay the real content rate. */
#define RADIO_STREAM_ACTUAL_RATE_HZ (47928)

/* Ring buffer between the I2S reader element and the resample filter
 * element. ESP-ADF's generic default (I2S_STREAM_RINGBUFFER_SIZE, 8KB) is
 * sized for boards where Wi-Fi/HTTP/decoders/a display etc. are all
 * competing for the same heap. This chip has none of that - it's
 * Bluedroid-only, ~200KB+ free heap at boot (see boot log) - so give it much
 * more headroom against I2S DMA / resample task scheduling jitter. 32KB at
 * the real 48kHz input rate =~170ms. (Not the same risk as the single-chip
 * project's RADIO_DECODER_BUFFER_BYTES OOM history - that was BT+Wi-Fi+TLS+
 * decoder all competing at once; this chip has no Wi-Fi/mbedTLS at all.) */
#define RADIO_I2S_RINGBUFFER_BYTES  (32 * 1024)

/* Ring buffer between the resample filter and the A2DP writer element - the
 * real "last mile" buffer now (bt_writer has none of its own, it pulls
 * straight out of this one via audio_element_input()), so this is what
 * actually absorbs A2DP retransmission stalls / BT scheduling jitter.
 * 32KB at 44.1kHz/stereo/16-bit =~185ms. Same free-heap headroom reasoning
 * as RADIO_I2S_RINGBUFFER_BYTES above - both together still leave >140KB of
 * the ~200KB+ free heap untouched. */
#define RADIO_RESAMPLE_RINGBUFFER_BYTES (32 * 1024)
