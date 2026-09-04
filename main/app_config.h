#pragma once

#include "driver/uart.h" /* for UART_NUM_2 (RADIO_AVRC_UART_PORT below) */

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
 * ESP32-to-ESP32 link.
 *
 * The I2S peripheral itself is configured at RADIO_STREAM_SAMPLE_RATE_HZ
 * (48000, below) - the real rate esp32_wifi_streamer's master actually
 * drives on this bus - NOT at RADIO_BT_SAMPLE_RATE_HZ (44100, the separate
 * Bluetooth output rate). Even though a slave's actual bit timing is set
 * entirely by the master's BCLK/WS edges rather than this field, i2s_stream
 * still uses its configured rate to size DMA buffers and the input ring
 * buffer, so it should reflect the real incoming rate rather than the
 * unrelated downstream BT rate - the two used to be the same #define, which
 * was a coincidence, not a design. */
#define RADIO_I2S_BCLK_GPIO         26
#define RADIO_I2S_WS_GPIO           25
#define RADIO_I2S_DATA_GPIO         27

/* Real physical rate esp32_wifi_streamer's master actually drives on this
 * bus, and what i2s_reader's own I2S peripheral config (main.c) is set to.
 * That project's radio_pipeline.c retunes its own I2S clock (via
 * i2s_stream_set_clk()) to match whatever its AAC decoder reports, and for
 * this fixed TuneIn station the CMAF init segment always parses as 48kHz -
 * see that project's app_config.h/radio_pipeline.c history. This is also the
 * real number the resample stage below needs to know about (see
 * RADIO_STREAM_ACTUAL_RATE_HZ for the finer-grained measured value actually
 * fed to the resampler).
 *
 * Why resample here and not fix it upstream: esp32_wifi_streamer is
 * already memory-constrained at runtime (documented OOM history in its own
 * app_config.h) and must not be given another buffer-hungry pipeline stage.
 * This chip, by contrast, is Bluedroid-only with ~200KB+ free heap at boot
 * and nothing else competing for it - the right place to absorb the
 * conversion. */
#define RADIO_STREAM_SAMPLE_RATE_HZ (48000)

/* JITTER FIX (2026-08-22), STATUS: NOT CURRENTLY WIRED IN (2026-08-23) - see
 * main.c's rsp_cfg.src_rate comment. On-device, FIR_RESAMPLE refused to
 * create the resample handler at all with src_rate=47928 ("sample rate is
 * out of range [8000 96000]", despite 47928 being numerically inside that
 * range - it wants a supported/table rate, not an arbitrary one), which
 * aborted the pipeline immediately - a hard failure, worse than the jitter
 * this was meant to fix. rsp_cfg.src_rate now uses flat
 * RADIO_STREAM_SAMPLE_RATE_HZ (48000) instead. The measurement below is kept
 * for reference in case a future resampler/approach can use it. Original
 * comment follows: the value above is the CONTENT's nominal rate
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

/* Final output rate fed to bt_writer / negotiated over A2DP - NOT the I2S
 * input rate (see RADIO_STREAM_SAMPLE_RATE_HZ/RADIO_STREAM_ACTUAL_RATE_HZ
 * above for that). ESP-IDF's vendored Bluedroid A2DP SOURCE role can only
 * ever negotiate 44.1kHz SBC with the speaker (bta_av_co_sbc_caps in
 * esp-idf's bta_av_co.c only sets A2D_SBC_IE_SAMP_FREQ_44, no 48kHz bit -
 * confirmed against that source directly, and independently against
 * github.com/espressif/esp-idf#5997: a user there patched that struct to
 * unlock 48kHz and it played for a few seconds then froze/hung - not a
 * usable path), so resample_filter's dest_rate is fixed at this value
 * regardless of the real input rate. */
#define RADIO_BT_SAMPLE_RATE_HZ (44100)

/* 2026-09-04: esp32_wifi_streamer no longer fixes its I2S output at one
 * rate - some stations are 44.1kHz, some (CMAF/HLS) are 48kHz, decided per-
 * station on that chip, with no back-channel telling this one which. So
 * this chip now measures the real incoming rate itself (main.c's I2S PROBE
 * write-callback, already existed for logging - now also drives this) and
 * retargets resample_filter's source rate live via rsp_filter_change_src_info(),
 * every ~1s, instead of assuming RADIO_STREAM_SAMPLE_RATE_HZ is always right.
 * A raw jitter-corrected measurement (e.g. 47928, see
 * RADIO_STREAM_ACTUAL_RATE_HZ below) is NOT fed to the resampler directly -
 * confirmed on hardware that it refuses non-table rates outright ("sample
 * rate is out of range", despite being numerically in range) - so the
 * measurement is only ever used to pick the NEARER of these two candidates,
 * never passed through as-is. When the measured rate rounds to
 * RADIO_BT_SAMPLE_RATE_HZ (44.1kHz) itself, resample_filter's src_rate is
 * set equal to its dest_rate - not a true pipeline bypass (the element stays
 * in the graph; removing it live would mean tearing down and relinking the
 * whole pipeline, which this project's own hard-won history flags as
 * genuinely risky on this board - see run_bridge_session()'s "I2S audio-
 * level probe" comment on an earlier extra-element attempt starving a
 * core's watchdog) - but src==dest is a lossless identity pass as far as
 * the resample math is concerned, which is what "don't resample" actually
 * means for audio quality/pitch purposes; the CPU cost of running data
 * through the filter at 1:1 is negligible next to a real 48->44.1 convert. */
#define RADIO_I2S_RATE_CANDIDATE_LOW_HZ   44100
#define RADIO_I2S_RATE_CANDIDATE_HIGH_HZ  48000
#define RADIO_I2S_RATE_CLASSIFY_MIDPOINT_HZ \
    ((RADIO_I2S_RATE_CANDIDATE_LOW_HZ + RADIO_I2S_RATE_CANDIDATE_HIGH_HZ) / 2)

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

/* --- AVRCP -> UART command bridge -------------------------------------
 * This chip is the AVRCP TARGET (see avrc_tg_event_cb() in main.c): the
 * connected Bluetooth speaker is the CONTROLLER side and can send transport
 * commands (play/pause/stop/next/previous/volume) to this chip, e.g. from
 * its own remote/physical controls. Those commands are only meaningful to
 * whatever is actually driving playback - esp32_wifi_streamer, on the other
 * side of the I2S link - so every command received here is forwarded over a
 * dedicated, one-way UART link to that chip, which acts on next/previous by
 * changing station (see ../../esp32_wifi_streamer/main/avrcp_uart.h and
 * station_list.h there).
 *
 * A separate hardware UART (not UART0/the USB-serial console on GPIO1/3) on
 * a pin clear of this chip's I2S bus (GPIO25/26/27 - see RADIO_I2S_BCLK_GPIO
 * above), clear of the strapping pins (0/2/5/12/15), the flash pins (6-11)
 * and the input-only pins (34-39). TX only - this bridge only ever sends,
 * esp32_wifi_streamer has nothing to report back over this link, so no RX
 * pin is configured (one less physical wire to route between the two
 * boards). Wire this GPIO to RADIO_AVRCP_UART_RX_GPIO on the
 * esp32_wifi_streamer board, plus a COMMON GROUND between the two boards -
 * without a shared GND the receiving side sees garbage, not silence. */
#define RADIO_AVRC_UART_PORT        UART_NUM_2
#define RADIO_AVRC_UART_TX_GPIO     16
#define RADIO_AVRC_UART_BAUD        115200

/* Wire format sent over the link above: one ASCII line per event, fields
 * separated by ':', terminated by '\n' -
 *
 *   AVRCP:CONN:<CONNECTED|DISCONNECTED>
 *   AVRCP:CMD:<name>:<PRESSED|RELEASED>
 *   AVRCP:VOL:<0-127>
 *
 * <name> is one of PLAY/PAUSE/STOP/NEXT/PREVIOUS/REWIND/FAST_FORWARD/
 * VOL_UP/VOL_DOWN/MUTE, or 0xNN for anything else AVRCP defines (see
 * avrc_pt_cmd_name() in main.c) - deliberately plain ASCII/line-oriented
 * rather than a binary struct so it's directly readable on a plain serial
 * monitor for debugging, and trivial for another ESP32 to parse with a
 * line-buffered uart_read_bytes() + strcmp(), no shared struct definition
 * or binary framing between the two projects required.
 *
 * The AVRCP key code is deliberately translated to a stable NAME here
 * rather than forwarded as a raw AVRCP opcode: the receiving chip then
 * needs no AVRCP/Bluedroid headers at all (it has no Bluetooth stack
 * compiled in), and an unrecognized code still crosses the link intact as
 * "0xNN" instead of being silently dropped at this end. */
