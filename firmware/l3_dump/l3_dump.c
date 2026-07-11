/* Stage-1c: L3 raw-ADC rolling buffer + dump-on-command  (SKELETON).
 *
 * The ring-buffer/windowing logic and the wire format below are REAL and final;
 * the SDK-specific glue is marked `TODO(SDK)` and must be filled in against the
 * mmWave SDK source, then built on x86 Linux (see ../README.md). Start from the
 * closest SDK ADC-capture example, NOT the full mmw demo — you only need: chirp
 * config, per-frame raw-ADC archival, a CLI command, and UART out.
 *
 * Model (the OPS243 rolling buffer, ported to the IWR): every frame, copy the
 * raw ADC into a ring slot in L3. On the Pi's "l3dump" command (sent at the
 * sound-trigger edge) DON'T dump immediately — the ball is still on the tee.
 * Instead keep capturing POST_FRAMES more, then stream the window
 * [trigger - PRE_FRAMES, trigger + POST_FRAMES] in the l3_dump_header_t format.
 */
#include <stdint.h>
#include <string.h>

#include "dump_format.h"

/* --- capture geometry: MUST match the .cfg sent to the sensor ------------- */
#define N_TX              2
#define N_RX              4
#define N_SAMPLES         128
#define LOOPS             1          /* TODO: match frameCfg numLoops */
#define CHIRPS_PER_FRAME  (N_TX * LOOPS)
#define FRAME_COMPLEX     (CHIRPS_PER_FRAME * N_RX * N_SAMPLES)

/* --- rolling buffer: ~L3 budget / raw frame size -------------------------- */
#define RING_FRAMES  12              /* ~1.5 MB L3 / ~128 KB per raw frame */
#define PRE_FRAMES   2               /* kept before the trigger (latency cushion) */
#define POST_FRAMES  8               /* captured after (ball flight, ~40 ms @ 200 fps) */

/* int16 I/Q -> 2 int16 per complex sample. Place g_ring in L3 via the linker. */
static int16_t g_ring[RING_FRAMES][FRAME_COMPLEX * 2];   /* TODO(SDK): .l3ram section */
static volatile uint32_t g_head;             /* next slot to write */
static volatile int32_t  g_post_left = -1;   /* >=0 while capturing post-trigger */
static volatile uint32_t g_trigger_slot;

/* Frame-done hook: register with the datapath so this runs once per frame. */
void l3_on_frame_done(void)
{
    uint32_t slot = g_head % RING_FRAMES;

    /* TODO(SDK): EDMA-copy this frame's raw ADC (all chirps x RX x samples,
     * from the ADCBUF ping/pong) into g_ring[slot]. This replaces the range-FFT
     * the OOB demo would run here — we archive raw and process on the Pi. */
    /* edma_copy_adcbuf_to(g_ring[slot]); */

    g_head++;

    if (g_post_left > 0) {
        g_post_left--;
    } else if (g_post_left == 0) {
        g_post_left = -1;
        l3_stream_dump();
    }
}

/* CLI handler for "l3dump" (register with the SDK CLI task). The Pi sends this
 * on the sound-trigger edge. Mark the trigger frame; keep capturing post-frames. */
int32_t l3_cli_dump(int32_t argc, char *argv[])
{
    (void)argc; (void)argv;
    g_trigger_slot = g_head;          /* current frame ~= impact */
    g_post_left    = POST_FRAMES;
    return 0;
}

/* Stream one dump window over the DATA UART in the l3_dump_header_t format. */
void l3_stream_dump(void)
{
    l3_dump_header_t h;
    memcpy(h.magic, L3_DUMP_MAGIC, 4);
    h.version          = L3_DUMP_VERSION;
    h.n_frames         = PRE_FRAMES + POST_FRAMES;
    h.chirps_per_frame = CHIRPS_PER_FRAME;
    h.n_tx             = N_TX;
    h.n_rx             = N_RX;
    h.n_samples        = N_SAMPLES;
    h.sample_fmt       = L3_SAMPLE_INT16_IQ;
    h._pad             = 0;
    h.trigger_frame    = PRE_FRAMES;
    h._pad2            = 0;

    /* TODO(SDK): UART_write(gDataUartHandle, &h, sizeof(h)); */

    /* window starts PRE_FRAMES before the trigger slot (mod the ring) */
    uint32_t start = g_trigger_slot + RING_FRAMES - PRE_FRAMES;
    for (uint16_t i = 0; i < h.n_frames; i++) {
        uint32_t slot = (start + i) % RING_FRAMES;
        /* TODO(SDK): UART_write(gDataUartHandle, g_ring[slot],
         *                       sizeof(g_ring[slot])); */
        (void)slot;
    }
}
