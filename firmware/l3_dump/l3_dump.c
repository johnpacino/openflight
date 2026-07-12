/* Stage-1c: L3 raw-ADC rolling buffer + dump-on-command  (IWR6843 MSS/R4F).
 *
 * MILESTONE 2a (this build): board bring-up + the Pi<->firmware UART contract.
 *   boot -> dual UART (CLI 115200 in, DATA 921600 out) -> CLI "l3dump" command
 *   -> stream a header + stub (zero) payload. This proves the whole toolchain
 *   AND the contract end-to-end: build/flash/boot, CLI receives the command,
 *   the data UART streams a burst iwr6843_runtime.read_dump() can frame.
 *   NOT yet wired here: RF chirp config (2b) and real ADCBUF->L3 capture (3).
 *
 * The ring-buffer + post-trigger windowing logic and the wire format below are
 * REAL and final. Per-frame capture (l3_on_frame_done) still has the ADCBUF
 * EDMA copy stubbed `TODO(SDK)`. g_ring is declared and RETAINed so that THIS
 * build already proves the raw-ADC ring fits in L3 (the layout is de-risked
 * before we write the capture path).
 *
 * Model (the OPS243 rolling buffer, ported to the IWR): every frame, copy the
 * raw ADC into a ring slot in L3. On the Pi's "l3dump" command (sent at the
 * sound-trigger edge) DON'T dump immediately -- the ball is still on the tee.
 * Instead keep capturing POST_FRAMES more, then stream the window
 * [trigger - PRE_FRAMES, trigger + POST_FRAMES] in the l3_dump_header_t format.
 */
#include <stdint.h>
#include <string.h>

/* BIOS/XDC */
#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* mmWave SDK drivers */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/utils/cli/cli.h>

#include "dump_format.h"

/* --- capture geometry: MUST match the .cfg the firmware programs (2b) ------ */
/* Mirrors config/iwr6843_levm_ball.cfg: channelCfg 15 5 -> 4 RX + 2 TX,
 * profileCfg ... 128 4000 -> 128 samples, frameCfg 0 1 32 -> 32 loops.        */
#define N_TX              2
#define N_RX              4
#define N_SAMPLES         128
#define LOOPS             32         /* frameCfg numLoops */
#define CHIRPS_PER_FRAME  (N_TX * LOOPS)                      /* 64 */
#define FRAME_COMPLEX     (CHIRPS_PER_FRAME * N_RX * N_SAMPLES)  /* 32768 */
#define FRAME_BYTES       (FRAME_COMPLEX * 2 * (uint32_t)sizeof(int16_t)) /* 128 KB */

/* --- rolling buffer -------------------------------------------------------- */
/* 128 KB per full-loop frame. The default SDK L3 is 6 banks x 128 KB = 768 KB
 * (mmwave_sdk_xwr68xx.mak), so RING_FRAMES is capped near 5-6. More frames
 * would need fewer loops per frame or more L3 banks (MMWAVE_L3RAM_NUM_BANK);
 * see docs/plans/stage1c_l3_burst_dump.md. RING_FRAMES=5 leaves L3 headroom. */
#define RING_FRAMES  5
#define PRE_FRAMES   2               /* kept before the trigger (latency cushion) */
#define POST_FRAMES  3               /* captured after (ball flight) */
/* PRE_FRAMES + POST_FRAMES must be <= RING_FRAMES. */

/* int16 I/Q -> 2 int16 per complex sample. Placed in L3 via the linker
 * (.l3ring -> L3_RAM in mss_linker.cmd). RETAIN keeps it in the 2a image so the
 * link already validates that the ring fits L3. */
#pragma DATA_SECTION(g_ring, ".l3ring")
#pragma RETAIN(g_ring)
static int16_t g_ring[RING_FRAMES][FRAME_COMPLEX * 2];
static volatile uint32_t g_head;             /* next slot to write */
static volatile int32_t  g_post_left = -1;   /* >=0 while capturing post-trigger */
static volatile uint32_t g_trigger_slot;

/* --- SDK handles ----------------------------------------------------------- */
static SOC_Handle  gSocHandle;
static UART_Handle gCliUart;      /* MSS UARTA, instance 0, 115200, has RX */
static UART_Handle gDataUart;     /* MSS UARTB, instance 1, 921600, TX only */
static uint32_t    gCpuClock = 200U * 1000000U;

/* Fill the 20-byte dump header (dump_format.h / iwr6843_l3dump.HEADER). */
static void l3_fill_header(l3_dump_header_t *h, uint16_t n_frames,
                           uint16_t trigger_frame)
{
    memcpy(h->magic, L3_DUMP_MAGIC, 4);
    h->version          = L3_DUMP_VERSION;
    h->n_frames         = n_frames;
    h->chirps_per_frame = CHIRPS_PER_FRAME;
    h->n_tx             = N_TX;
    h->n_rx             = N_RX;
    h->n_samples        = N_SAMPLES;
    h->sample_fmt       = L3_SAMPLE_INT16_IQ;
    h->_pad             = 0;
    h->trigger_frame    = trigger_frame;
    h->_pad2            = 0;
}

/* 2a stub: stream header + n_frames of zeros so the Pi can validate framing
 * without a live sensor. Kept small (fast over UART, well under read_dump's
 * timeout). Replaced by l3_stream_dump() once capture is real (step 3). */
static void l3_stream_stub(uint16_t n_frames)
{
    static const uint8_t zeros[4096] = {0};
    l3_dump_header_t h;
    uint32_t remaining = (uint32_t)n_frames * FRAME_BYTES;

    l3_fill_header(&h, n_frames, 0);
    UART_writePolling(gDataUart, (uint8_t *)&h, sizeof(h));
    while (remaining) {
        uint32_t n = (remaining < sizeof(zeros)) ? remaining : sizeof(zeros);
        UART_writePolling(gDataUart, (uint8_t *)zeros, n);
        remaining -= n;
    }
}

/* Frame-done hook (step 3: register with the datapath so this runs per frame). */
void l3_on_frame_done(void)
{
    uint32_t slot = g_head % RING_FRAMES;
    (void)slot;

    /* TODO(SDK): EDMA-copy this frame's raw ADC (all chirps x RX x samples,
     * from the ADCBUF ping/pong) into g_ring[slot]. This replaces the range-FFT
     * the OOB demo would run here -- we archive raw and process on the Pi. */
    /* edma_copy_adcbuf_to(g_ring[slot]); */

    g_head++;

    if (g_post_left > 0) {
        g_post_left--;
    } else if (g_post_left == 0) {
        g_post_left = -1;
        l3_stream_dump();
    }
}

/* CLI handler for "l3dump" (registered below; the Pi sends it on the sound-
 * trigger edge). 2a: stream a stub burst immediately. Step 3: instead mark the
 * trigger frame and keep capturing POST_FRAMES, then dump (see l3_on_frame_done). */
int32_t l3_cli_dump(int32_t argc, char *argv[])
{
    (void)argc; (void)argv;
    l3_stream_stub(2);
    return 0;
}

/* Stream one real dump window over the DATA UART (step 3 path; compiled now so
 * the ring/window logic is validated against the real UART API). */
void l3_stream_dump(void)
{
    l3_dump_header_t h;
    uint32_t start;
    uint16_t i;

    l3_fill_header(&h, PRE_FRAMES + POST_FRAMES, PRE_FRAMES);
    UART_writePolling(gDataUart, (uint8_t *)&h, sizeof(h));

    /* window starts PRE_FRAMES before the trigger slot (mod the ring) */
    start = g_trigger_slot + RING_FRAMES - PRE_FRAMES;
    for (i = 0; i < h.n_frames; i++) {
        uint32_t slot = (start + i) % RING_FRAMES;
        UART_writePolling(gDataUart, (uint8_t *)g_ring[slot], sizeof(g_ring[slot]));
    }
}

/* System init task: pinmux + open both UARTs + start the CLI. */
static void l3_initTask(UArg arg0, UArg arg1)
{
    UART_Params  uartParams;
    static CLI_Cfg cliCfg;        /* static: zero-init, keeps it off the stack */

    (void)arg0; (void)arg1;

    UART_init();

    /* MSS UARTA (CLI, instance 0): TX on PINN5, RX on PINN4. */
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN5_PADBE, SOC_XWR68XX_PINN5_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN5_PADBE,
                            PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN4_PADBD, SOC_XWR68XX_PINN4_PADBD_MSS_UARTA_RX);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN4_PADBD,
                            PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);

    /* MSS UARTB (DATA, instance 1): TX only on PINF14. */
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINF14_PADAJ,
                            PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINF14_PADAJ, SOC_XWR68XX_PINF14_PADAJ_MSS_UARTB_TX);

    /* CLI UART: 115200, 8N1, RX-capable. */
    UART_Params_init(&uartParams);
    uartParams.clockFrequency = gCpuClock;
    uartParams.baudRate       = 115200;
    uartParams.isPinMuxDone    = 1;
    gCliUart = UART_open(0, &uartParams);

    /* DATA UART: 921600, TX only (the burst egress). */
    UART_Params_init(&uartParams);
    uartParams.clockFrequency = gCpuClock;
    uartParams.baudRate       = 921600;
    uartParams.isPinMuxDone    = 1;
    gDataUart = UART_open(1, &uartParams);

    /* CLI with our single command. mmWave extension OFF (no sensor config in 2a). */
    cliCfg.cliPrompt             = "l3dump:/>";
    cliCfg.cliBanner             = "OpenFlight L3-dump firmware (2a UART/CLI bringup)\n";
    cliCfg.cliUartHandle         = gCliUart;
    cliCfg.socHandle             = gSocHandle;
    cliCfg.enableMMWaveExtension = 0;
    cliCfg.mmWaveHandle          = NULL;
    cliCfg.taskPriority          = 3;
    cliCfg.usePolledMode         = true;
    cliCfg.tableEntry[0].cmd           = "l3dump";
    cliCfg.tableEntry[0].helpString    = "Stream one raw-ADC dump";
    cliCfg.tableEntry[0].cmdHandlerFxn = l3_cli_dump;
    CLI_open(&cliCfg);
}

int32_t main(void)
{
    Task_Params taskParams;
    SOC_Cfg     socCfg;
    int32_t     errCode;

    /* ESM: don't clear errors (TI RTOS does it). */
    ESM_init(0U);

    memset((void *)&socCfg, 0, sizeof(SOC_Cfg));
    socCfg.clockCfg = SOC_SysClock_INIT;
    gSocHandle = SOC_init(&socCfg, &errCode);
    if (gSocHandle == NULL) {
        return -1;
    }

    Task_Params_init(&taskParams);
    taskParams.stackSize = 8 * 1024;
    Task_create(l3_initTask, &taskParams, NULL);

    BIOS_start();
    return 0;
}
