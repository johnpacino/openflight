/* Stage-1c: L3 raw-ADC rolling buffer + dump-on-command  (IWR6843 MSS/R4F).
 *
 * MILESTONE 2b (this build): the radar actually runs. On top of 2a's dual-UART
 * + CLI, this brings up the mmWave control module (FULL config mode) and enables
 * TI's CLI mmWave extension, so the host configures the chirp profile over the
 * CLI (profileCfg/chirpCfg/frameCfg/...) and `sensorStart` opens+configs+starts
 * the front-end. The board then free-runs, chirping continuously.
 *   Config source: host-sent (config/iwr6843_l3dump.cfg) -> flexible, no reflash
 *   to retune. `l3dump` still streams the ZERO stub -- real ADCBUF->L3 capture
 *   is step 3 (l3_on_frame_done EDMA copy, still TODO(SDK)).
 *
 * The ring-buffer + post-trigger windowing logic and the wire format below are
 * REAL and final. g_ring is RETAINed so the build proves it fits L3.
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

/* mmWave SDK drivers + control */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/utils/cli/cli.h>

#include "dump_format.h"

/* --- task priorities: mmWave control MUST outrank the CLI so config/start
 * requests (issued from the CLI task) get their BSS responses serviced. ------ */
#define L3_INIT_TASK_PRIORITY  5
#define L3_CTRL_TASK_PRIORITY  6
#define L3_CLI_TASK_PRIORITY   3

/* --- capture geometry: MUST match the .cfg the host sends ------------------ */
/* Mirrors config/iwr6843_l3dump.cfg: channelCfg 15 5 -> 4 RX + 2 TX,
 * profileCfg ... 128 4000 -> 128 samples, frameCfg 0 1 32 -> 32 loops.        */
#define N_TX              2
#define N_RX              4
#define N_SAMPLES         128
#define LOOPS             32         /* frameCfg numLoops */
#define CHIRPS_PER_FRAME  (N_TX * LOOPS)                        /* 64 */
#define FRAME_COMPLEX     (CHIRPS_PER_FRAME * N_RX * N_SAMPLES)   /* 32768 */
#define FRAME_BYTES       (FRAME_COMPLEX * 2 * (uint32_t)sizeof(int16_t)) /* 128 KB */

/* --- rolling buffer -------------------------------------------------------- */
/* 128 KB per full-loop frame. Default SDK L3 = 6 banks x 128 KB = 768 KB, so
 * RING_FRAMES is capped near 5-6 (see docs/plans/stage1c_l3_burst_dump.md). */
#define RING_FRAMES  5
#define PRE_FRAMES   2               /* kept before the trigger (latency cushion) */
#define POST_FRAMES  3               /* captured after (ball flight) */
/* PRE_FRAMES + POST_FRAMES must be <= RING_FRAMES. */

#pragma DATA_SECTION(g_ring, ".l3ring")
#pragma RETAIN(g_ring)
static int16_t g_ring[RING_FRAMES][FRAME_COMPLEX * 2];
static volatile uint32_t g_head;             /* next slot to write */
static volatile int32_t  g_post_left = -1;   /* >=0 while capturing post-trigger */
static volatile uint32_t g_trigger_slot;

/* --- SDK handles ----------------------------------------------------------- */
static SOC_Handle    gSocHandle;
static UART_Handle   gCliUart;    /* MSS UARTA, instance 0, 115200, has RX */
static UART_Handle   gDataUart;   /* MSS UARTB, instance 1, 921600, TX only */
static MMWave_Handle gMMWaveHandle;
static uint8_t       gSensorOpened;           /* MMWave_open done once */
static uint32_t      gCpuClock = 200U * 1000000U;

/* Config pulled from the CLI mmWave extension (kept off the stack -- large). */
static MMWave_OpenCfg gOpenCfg;
static MMWave_CtrlCfg gCtrlCfg;

/* Forward declarations. */
void    l3_on_frame_done(void);
int32_t l3_cli_dump(int32_t argc, char *argv[]);
void    l3_stream_dump(void);
static int32_t l3_cli_sensorStart(int32_t argc, char *argv[]);
static int32_t l3_cli_sensorStop(int32_t argc, char *argv[]);

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

/* Stub: stream header + n_frames of zeros so the host can validate framing
 * (2a/2b). Replaced by l3_stream_dump() once capture is real (step 3). */
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
     * from the ADCBUF ping/pong) into g_ring[slot]. */
    /* edma_copy_adcbuf_to(g_ring[slot]); */

    g_head++;

    if (g_post_left > 0) {
        g_post_left--;
    } else if (g_post_left == 0) {
        g_post_left = -1;
        l3_stream_dump();
    }
}

/* CLI handler for "l3dump". 2b: still streams the stub (capture is step 3). */
int32_t l3_cli_dump(int32_t argc, char *argv[])
{
    (void)argc; (void)argv;
    l3_stream_stub(2);
    return 0;
}

/* Stream one real dump window over the DATA UART (step 3 path; compiled now). */
void l3_stream_dump(void)
{
    l3_dump_header_t h;
    uint32_t start;
    uint16_t i;

    l3_fill_header(&h, PRE_FRAMES + POST_FRAMES, PRE_FRAMES);
    UART_writePolling(gDataUart, (uint8_t *)&h, sizeof(h));

    start = g_trigger_slot + RING_FRAMES - PRE_FRAMES;
    for (i = 0; i < h.n_frames; i++) {
        uint32_t slot = (start + i) % RING_FRAMES;
        UART_writePolling(gDataUart, (uint8_t *)g_ring[slot], sizeof(g_ring[slot]));
    }
}

/* mmWave async-event callback (BSS faults, calibration status, frame events).
 * Minimal for bring-up: MMWave_start blocks on the calibration handshake
 * internally, so we don't need to act on these to run -- just accept them.
 * Step 3 can expand this (fault handling / frame accounting). */
static int32_t l3_mmwaveEvent(uint16_t msgId, uint16_t sbId, uint16_t sbLen,
                              uint8_t *payload)
{
    (void)msgId; (void)sbId; (void)sbLen; (void)payload;
    return 0;
}

/* mmWave control execution context: pumps MMWave_execute in a loop. Must run at
 * higher priority than the CLI task (which issues config/start). */
static void l3_mmwaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;
    (void)arg0; (void)arg1;

    while (1) {
        /* Blocks internally; services BSS mailbox traffic for open/config/start. */
        MMWave_execute(gMMWaveHandle, &errCode);
    }
}

/* CLI "sensorStart": pull the profile the extension collected, then
 * open (once) -> config -> start the front-end. Errors are decoded to the CLI
 * so a bad profile is visible to the host (send_config sees no "Done"). */
static int32_t l3_cli_sensorStart(int32_t argc, char *argv[])
{
    MMWave_CalibrationCfg calibrationCfg;
    MMWave_ErrorLevel     errorLevel;
    int16_t               mmwErr, subErr;
    int32_t               errCode;
    (void)argc; (void)argv;

    if (!gSensorOpened) {
        CLI_getMMWaveExtensionOpenConfig(&gOpenCfg);
        if (MMWave_open(gMMWaveHandle, &gOpenCfg, NULL, &errCode) < 0) {
            MMWave_decodeError(errCode, &errorLevel, &mmwErr, &subErr);
            CLI_write("Error: MMWave_open failed [mmwave %d subsys %d]\n", mmwErr, subErr);
            return -1;
        }
        gSensorOpened = 1U;
    }

    CLI_getMMWaveExtensionConfig(&gCtrlCfg);
    if (MMWave_config(gMMWaveHandle, &gCtrlCfg, &errCode) < 0) {
        MMWave_decodeError(errCode, &errorLevel, &mmwErr, &subErr);
        CLI_write("Error: MMWave_config failed [mmwave %d subsys %d]\n", mmwErr, subErr);
        return -1;
    }

    memset((void *)&calibrationCfg, 0, sizeof(calibrationCfg));
    calibrationCfg.dfeDataOutputMode                          = gCtrlCfg.dfeDataOutputMode;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = true;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = true;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

    if (MMWave_start(gMMWaveHandle, &calibrationCfg, &errCode) < 0) {
        MMWave_decodeError(errCode, &errorLevel, &mmwErr, &subErr);
        CLI_write("Error: MMWave_start failed [mmwave %d subsys %d]\n", mmwErr, subErr);
        return -1;
    }
    return 0;
}

/* CLI "sensorStop": stop the front-end (honest -- actually stops chirping). */
static int32_t l3_cli_sensorStop(int32_t argc, char *argv[])
{
    int32_t errCode;
    (void)argc; (void)argv;
    MMWave_stop(gMMWaveHandle, &errCode);   /* may return informational error; ignore */
    return 0;
}

/* System init task: mmWave control + dual UART + CLI (with mmWave extension). */
static void l3_initTask(UArg arg0, UArg arg1)
{
    MMWave_InitCfg  initCfg;
    UART_Params     uartParams;
    Task_Params     taskParams;
    static CLI_Cfg  cliCfg;        /* static: zero-init, keeps it off the stack */
    int32_t         errCode;

    (void)arg0; (void)arg1;

    /* Mailbox is the MSS<->BSS transport used by the mmWave control module. */
    Mailbox_init(MAILBOX_TYPE_MSS);

    /* Bring up the mmWave control module (FULL config, MSS runs the link solo). */
    memset((void *)&initCfg, 0, sizeof(MMWave_InitCfg));
    initCfg.domain                  = MMWave_Domain_MSS;
    initCfg.socHandle               = gSocHandle;
    initCfg.eventFxn                = l3_mmwaveEvent;
    initCfg.linkCRCCfg.useCRCDriver = 1U;
    initCfg.linkCRCCfg.crcChannel   = CRC_Channel_CH1;
    initCfg.cfgMode                 = MMWave_ConfigurationMode_FULL;
    initCfg.executionMode           = MMWave_ExecutionMode_ISOLATION;
    gMMWaveHandle = MMWave_init(&initCfg, &errCode);
    if (gMMWaveHandle == NULL) {
        return;
    }

    /* Synchronize (prerequisite before any control API). */
    while (1) {
        int32_t syncStatus = MMWave_sync(gMMWaveHandle, &errCode);
        if (syncStatus < 0) {
            return;
        }
        if (syncStatus == 1) {
            break;
        }
        Task_sleep(1);
    }

    /* Control execution context (higher priority than the CLI). */
    Task_Params_init(&taskParams);
    taskParams.priority  = L3_CTRL_TASK_PRIORITY;
    taskParams.stackSize = 3 * 1024;
    Task_create(l3_mmwaveCtrlTask, &taskParams, NULL);

    /* --- UARTs (same wiring as 2a) --- */
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

    UART_Params_init(&uartParams);
    uartParams.clockFrequency = gCpuClock;
    uartParams.baudRate       = 115200;
    uartParams.isPinMuxDone    = 1;
    gCliUart = UART_open(0, &uartParams);

    UART_Params_init(&uartParams);
    uartParams.clockFrequency = gCpuClock;
    uartParams.baudRate       = 921600;
    uartParams.isPinMuxDone    = 1;
    gDataUart = UART_open(1, &uartParams);

    /* --- CLI with the mmWave extension (registers profileCfg/chirpCfg/... ) --- */
    cliCfg.cliPrompt             = "l3dump:/>";
    cliCfg.cliBanner             = "OpenFlight L3-dump firmware (2b: RF config + free-run)\n";
    cliCfg.cliUartHandle         = gCliUart;
    cliCfg.socHandle             = gSocHandle;
    cliCfg.mmWaveHandle          = gMMWaveHandle;
    cliCfg.enableMMWaveExtension = 1U;
    cliCfg.taskPriority          = L3_CLI_TASK_PRIORITY;
    cliCfg.usePolledMode         = true;
    cliCfg.tableEntry[0].cmd           = "sensorStart";
    cliCfg.tableEntry[0].helpString    = "Configure + start the front-end (no args)";
    cliCfg.tableEntry[0].cmdHandlerFxn = l3_cli_sensorStart;
    cliCfg.tableEntry[1].cmd           = "sensorStop";
    cliCfg.tableEntry[1].helpString    = "Stop the front-end";
    cliCfg.tableEntry[1].cmdHandlerFxn = l3_cli_sensorStop;
    cliCfg.tableEntry[2].cmd           = "l3dump";
    cliCfg.tableEntry[2].helpString    = "Stream one raw-ADC dump";
    cliCfg.tableEntry[2].cmdHandlerFxn = l3_cli_dump;
    CLI_open(&cliCfg);
}

int32_t main(void)
{
    Task_Params taskParams;
    SOC_Cfg     socCfg;
    int32_t     errCode;

    ESM_init(0U);

    memset((void *)&socCfg, 0, sizeof(SOC_Cfg));
    socCfg.clockCfg = SOC_SysClock_INIT;
    gSocHandle = SOC_init(&socCfg, &errCode);
    if (gSocHandle == NULL) {
        return -1;
    }

    Task_Params_init(&taskParams);
    taskParams.priority  = L3_INIT_TASK_PRIORITY;
    taskParams.stackSize = 8 * 1024;
    Task_create(l3_initTask, &taskParams, NULL);

    BIOS_start();
    return 0;
}
