/* Stage-1c: L3 raw-ADC rolling buffer + dump-on-command  (IWR6843 MSS/R4F).
 *
 * MILESTONE 3 (this build): REAL raw-ADC capture. On top of 2b (RF configures
 * and free-runs, host-config over the CLI), this captures every chirp's raw
 * ADC into an L3 rolling buffer via EDMA, and `l3dump` freezes the ring and
 * streams the most recent RING_FRAMES frames of real per-antenna I/Q.
 *
 * Capture path (mirrors the SDK mem_capture/capture.c technique):
 *   DFE -> ADCBUF (one chirp, non-interleaved: [rx0 128 cplx][rx1]..[rx3])
 *   chirp-available ISR -> EDMA one chirp (CHIRP_BYTES) ADCBUF -> g_ring[slot]
 *   EDMA-done -> advance the destination by one chirp
 *   frame-start ISR -> that frame is complete: advance the ring slot (rolling)
 *   l3dump -> freeze at the next frame boundary, stream the ring, unfreeze
 *
 * Chirp order filled == chirp order fired == TDM (chirp c -> tx=c%N_TX,
 * loop=c/N_TX), which is exactly what iwr6843_l3dump expects.
 */
#include <stdint.h>
#include <stdio.h>
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
#include <ti/drivers/adcbuf/ADCBuf.h>
#include <ti/drivers/edma/edma.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/utils/cli/cli.h>

#include "dump_format.h"

/* --- task priorities (mirror the mmw demo): ctrl > CLI. -------------------- */
#define L3_INIT_TASK_PRIORITY  2
#define L3_CLI_TASK_PRIORITY   3
#define L3_CTRL_TASK_PRIORITY  5

/* --- capture geometry: MUST match the .cfg the host sends ------------------ */
#define N_TX              2
#define N_RX              4
#define N_SAMPLES         128
#define LOOPS             32
#define CHIRPS_PER_FRAME  (N_TX * LOOPS)                        /* 64 */
#define FRAME_COMPLEX     (CHIRPS_PER_FRAME * N_RX * N_SAMPLES)   /* 32768 */
#define FRAME_BYTES       (FRAME_COMPLEX * 2 * (uint32_t)sizeof(int16_t)) /* 128 KB */
/* One chirp in ADCBUF (non-interleaved): N_RX x N_SAMPLES x 4 bytes (int16 I/Q). */
#define CHIRP_BYTES       (N_RX * N_SAMPLES * 2 * (uint32_t)sizeof(int16_t))  /* 2048 */

/* --- rolling buffer (default L3 = 6 banks x 128 KB = 768 KB) ---------------- */
#define RING_FRAMES  5

#pragma DATA_SECTION(g_ring, ".l3ring")
#pragma DATA_ALIGN(g_ring, 8)
static int16_t g_ring[RING_FRAMES][FRAME_COMPLEX * 2];

/* --- EDMA channel (free channel + shadow link, per SDK mem_capture) --------- */
#define L3_EDMA_CHANNEL       EDMA_TPCC0_REQ_FREE_13
#define L3_EDMA_LINK_CHANNEL  EDMA_NUM_DMA_CHANNELS

/* --- SDK handles ----------------------------------------------------------- */
static SOC_Handle    gSocHandle;
static UART_Handle   gCliUart;    /* MSS UARTA, instance 0, 115200, has RX */
static UART_Handle   gDataUart;   /* MSS UARTB, instance 1, 921600, TX only */
static MMWave_Handle gMMWaveHandle;
static EDMA_Handle   gEdmaHandle;
static ADCBuf_Handle gAdcbufHandle;
static uint8_t       gSensorOpened;
static uint32_t      gCpuClock = 200U * 1000000U;

/* --- capture state (touched by ISRs) --------------------------------------- */
static volatile uint8_t  gCaptureActive;    /* ring is being filled */
static volatile uint8_t  gFreezeReq;        /* dump requested; freeze at next frame */
static volatile uint8_t  gFrozen;           /* ring frozen (ISRs skip EDMA/advance) */
static volatile uint8_t  gEdmaBusy;         /* an EDMA chirp transfer is in flight */
static volatile uint32_t gCurrDstAddr;      /* EDMA dest for the next chirp (EDMA view) */
static volatile uint32_t gHead;             /* frames completed (== next write slot base) */
static volatile uint32_t gFrozenHead;       /* gHead latched at freeze */
/* diagnostics */
static volatile uint32_t gNumChirp;
static volatile uint32_t gNumEdma;
static volatile uint32_t gNumFrame;

/* Config pulled from the CLI mmWave extension (kept off the stack -- large). */
static MMWave_OpenCfg gOpenCfg;
static MMWave_CtrlCfg gCtrlCfg;

/* Forward declarations. */
int32_t l3_cli_dump(int32_t argc, char *argv[]);
static int32_t l3_cli_sensorStart(int32_t argc, char *argv[]);
static int32_t l3_cli_sensorStop(int32_t argc, char *argv[]);
static int32_t l3_cli_stats(int32_t argc, char *argv[]);

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

/* Chirp-available ISR: EDMA this chirp's ADC out of the ADCBUF into the ring.
 * Short + hardware-timed (fires per chirp ~ every 45 us). */
static void l3_chirpISR(uintptr_t arg)
{
    (void)arg;
    gNumChirp++;
    if (!gCaptureActive || gFrozen || gEdmaBusy) {
        return;
    }
    /* Copy one chirp (CHIRP_BYTES) from the ADCBUF base to the current slot. */
    if (EDMA_setDestinationAddress(gEdmaHandle, (uint16_t)L3_EDMA_CHANNEL,
                                   gCurrDstAddr) != EDMA_NO_ERROR) {
        return;
    }
    if (EDMA_startDmaTransfer(gEdmaHandle, L3_EDMA_CHANNEL) != EDMA_NO_ERROR) {
        return;
    }
    gEdmaBusy = 1U;
}

/* EDMA transfer-complete callback: advance the destination by one chirp. */
static void l3_edmaCB(uintptr_t arg, uint8_t tcCode)
{
    (void)arg; (void)tcCode;
    gNumEdma++;
    gEdmaBusy = 0U;
    gCurrDstAddr += CHIRP_BYTES;
}

/* Frame-start ISR: the previous frame's chirps are all captured -> advance the
 * ring slot (rolling buffer). If a dump was requested, freeze here (at a clean
 * frame boundary) so the CLI task can stream a consistent window. */
static void l3_frameStartISR(uintptr_t arg)
{
    uint32_t slot;
    (void)arg;
    gNumFrame++;
    if (!gCaptureActive || gFrozen) {
        return;
    }
    gHead++;                                  /* a frame just completed */
    if (gFreezeReq) {
        gFrozenHead = gHead;
        gFrozen     = 1U;                     /* stop filling until the dump is done */
        return;
    }
    slot          = gHead % RING_FRAMES;      /* next frame goes here */
    gCurrDstAddr  = SOC_translateAddress((uint32_t)&g_ring[slot][0],
                                         SOC_TranslateAddr_Dir_TO_EDMA, NULL);
    gEdmaBusy     = 0U;
}

/* CLI "l3dump": freeze the ring at the next frame boundary, then stream the
 * most recent RING_FRAMES frames of real ADC, then resume capture. The Pi
 * frames the burst purely by the header's byte count. */
int32_t l3_cli_dump(int32_t argc, char *argv[])
{
    l3_dump_header_t h;
    uint32_t         base, i, waited;
    (void)argc; (void)argv;

    if (!gCaptureActive) {
        return -1;
    }

    /* Request a freeze and wait for the frame-start ISR to latch it (<= ~1 frame,
     * 8 ms). Bounded so a stalled sensor can't hang the CLI. */
    gFreezeReq = 1U;
    for (waited = 0; waited < 200 && !gFrozen; waited++) {
        Task_sleep(1);
    }
    if (!gFrozen) {                            /* sensor not producing frames */
        gFreezeReq = 0U;
        return -1;
    }

    /* Stream the RING_FRAMES completed frames, oldest -> newest. At freeze,
     * gFrozenHead is the next-to-write slot, i.e. the oldest of the ring. */
    l3_fill_header(&h, RING_FRAMES, 0);
    UART_writePolling(gDataUart, (uint8_t *)&h, sizeof(h));
    base = gFrozenHead;
    for (i = 0; i < RING_FRAMES; i++) {
        uint32_t slot = (base + i) % RING_FRAMES;
        UART_writePolling(gDataUart, (uint8_t *)g_ring[slot], sizeof(g_ring[slot]));
    }

    /* Resume the rolling buffer at the current slot. */
    gCurrDstAddr = SOC_translateAddress((uint32_t)&g_ring[gHead % RING_FRAMES][0],
                                        SOC_TranslateAddr_Dir_TO_EDMA, NULL);
    gEdmaBusy  = 0U;
    gFrozen    = 0U;
    gFreezeReq = 0U;
    return 0;
}

/* CLI "stats": report capture counters (diagnostic). */
static int32_t l3_cli_stats(int32_t argc, char *argv[])
{
    (void)argc; (void)argv;
    CLI_write("chirps=%u edma=%u frames=%u head=%u active=%d frozen=%d\n",
              (unsigned)gNumChirp, (unsigned)gNumEdma, (unsigned)gNumFrame,
              (unsigned)gHead, (int)gCaptureActive, (int)gFrozen);
    return 0;
}

/* Configure the ADCBUF for our chirp format (complex, non-interleaved, 4 RX). */
static int32_t l3_configAdcBuf(void)
{
    ADCBuf_dataFormat dataFormat;
    ADCBuf_RxChanConf rxChanCfg;
    uint32_t          rxChanMask = 0xFU;
    uint32_t          chirpThreshold = 1U;
    uint8_t           ch;
    int32_t           rc;

    if (ADCBuf_control(gAdcbufHandle, ADCBufMMWave_CMD_CHANNEL_DISABLE,
                       (void *)&rxChanMask) != ADCBuf_STATUS_SUCCESS) {
        return -1;
    }
    memset((void *)&dataFormat, 0, sizeof(dataFormat));
    dataFormat.adcOutFormat      = 0;   /* complex */
    dataFormat.sampleInterleave  = 1;   /* per config/iwr6843_l3dump.cfg adcCfg/adcbuf */
    dataFormat.channelInterleave = 1;   /* non-interleaved: RX in contiguous blocks */
    if (ADCBuf_control(gAdcbufHandle, ADCBufMMWave_CMD_CONF_DATA_FORMAT,
                       (void *)&dataFormat) != ADCBuf_STATUS_SUCCESS) {
        return -1;
    }
    memset((void *)&rxChanCfg, 0, sizeof(rxChanCfg));
    for (ch = 0; ch < N_RX; ch++) {
        rxChanCfg.channel = ch;
        rc = ADCBuf_control(gAdcbufHandle, ADCBufMMWave_CMD_CHANNEL_ENABLE,
                            (void *)&rxChanCfg);
        if (rc != ADCBuf_STATUS_SUCCESS) {
            return -1;
        }
        rxChanCfg.offset += N_SAMPLES * 2 * (uint32_t)sizeof(int16_t); /* 512 B/chan */
    }
    if (ADCBuf_control(gAdcbufHandle, ADCBufMMWave_CMD_SET_PING_CHIRP_THRESHHOLD,
                       (void *)&chirpThreshold) != ADCBuf_STATUS_SUCCESS) {
        return -1;
    }
    if (ADCBuf_control(gAdcbufHandle, ADCBufMMWave_CMD_SET_PONG_CHIRP_THRESHHOLD,
                       (void *)&chirpThreshold) != ADCBuf_STATUS_SUCCESS) {
        return -1;
    }
    return 0;
}

/* Configure the EDMA channel: ADCBUF -> ring, CHIRP_BYTES per software trigger. */
static int32_t l3_configEdma(void)
{
    EDMA_channelConfig_t   ch;
    EDMA_paramSetConfig_t *ps;
    EDMA_paramConfig_t     linkCfg;
    uint32_t               srcAddr, dstAddr;
    int32_t                rc;

    srcAddr = SOC_translateAddress(SOC_XWR68XX_MSS_ADCBUF_BASE_ADDRESS,
                                   SOC_TranslateAddr_Dir_TO_EDMA, NULL);
    dstAddr = SOC_translateAddress((uint32_t)&g_ring[0][0],
                                   SOC_TranslateAddr_Dir_TO_EDMA, NULL);

    (void)EDMA_disableChannel(gEdmaHandle, L3_EDMA_CHANNEL, EDMA3_CHANNEL_TYPE_DMA);

    memset((void *)&ch, 0, sizeof(ch));
    ch.channelId   = L3_EDMA_CHANNEL;
    ch.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    ch.paramId     = L3_EDMA_CHANNEL;
    ch.eventQueueId = 1U;
    ch.transferCompletionCallbackFxn    = l3_edmaCB;
    ch.transferCompletionCallbackFxnArg = (uintptr_t)0U;

    ps = &ch.paramSetConfig;
    ps->sourceAddress      = srcAddr;
    ps->destinationAddress = dstAddr;
    ps->aCount             = (uint16_t)CHIRP_BYTES;
    ps->bCount             = 1U;
    ps->cCount             = 1U;
    ps->bCountReload       = ps->bCount;
    ps->sourceBindex       = (int16_t)CHIRP_BYTES;
    ps->destinationBindex  = (int16_t)CHIRP_BYTES;
    ps->sourceCindex       = 0;
    ps->destinationCindex  = 0;
    ps->linkAddress        = EDMA_NULL_LINK_ADDRESS;
    ps->transferType       = (uint8_t)EDMA3_SYNC_AB;
    ps->transferCompletionCode = (uint8_t)L3_EDMA_CHANNEL;
    ps->sourceAddressingMode      = (uint8_t)EDMA3_ADDRESSING_MODE_LINEAR;
    ps->destinationAddressingMode = (uint8_t)EDMA3_ADDRESSING_MODE_LINEAR;
    ps->fifoWidth          = (uint8_t)EDMA3_FIFO_WIDTH_8BIT;
    ps->isStaticSet        = false;
    ps->isEarlyCompletion  = false;
    ps->isFinalTransferInterruptEnabled        = true;
    ps->isIntermediateTransferInterruptEnabled = false;
    ps->isFinalChainingEnabled        = false;
    ps->isIntermediateChainingEnabled = false;

    rc = EDMA_configChannel(gEdmaHandle, &ch, false);
    if (rc != EDMA_NO_ERROR) {
        return -1;
    }
    memcpy((void *)&linkCfg.paramSetConfig, (void *)ps, sizeof(EDMA_paramSetConfig_t));
    linkCfg.transferCompletionCallbackFxn    = l3_edmaCB;
    linkCfg.transferCompletionCallbackFxnArg = (uintptr_t)0U;
    if (EDMA_configParamSet(gEdmaHandle, L3_EDMA_LINK_CHANNEL, &linkCfg) != EDMA_NO_ERROR) {
        return -1;
    }
    if (EDMA_linkParamSets(gEdmaHandle, L3_EDMA_CHANNEL, L3_EDMA_LINK_CHANNEL) != EDMA_NO_ERROR) {
        return -1;
    }
    /* Self-link the shadow param set so it reloads forever -- without this the
     * channel runs exactly twice (own set + one reload) then stalls. */
    if (EDMA_linkParamSets(gEdmaHandle, L3_EDMA_LINK_CHANNEL, L3_EDMA_LINK_CHANNEL) != EDMA_NO_ERROR) {
        return -1;
    }
    return 0;
}

/* mmWave async-event callback (minimal for bring-up). */
static int32_t l3_mmwaveEvent(uint16_t msgId, uint16_t sbId, uint16_t sbLen,
                              uint8_t *payload)
{
    (void)msgId; (void)sbId; (void)sbLen; (void)payload;
    return 0;
}

/* mmWave control execution context (must outrank the CLI task). */
static void l3_mmwaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;
    (void)arg0; (void)arg1;
    while (1) {
        MMWave_execute(gMMWaveHandle, &errCode);
    }
}

/* CLI "sensorStart": open -> config -> (ADCBUF + EDMA + start capture) -> start. */
static int32_t l3_cli_sensorStart(int32_t argc, char *argv[])
{
    MMWave_CalibrationCfg calibrationCfg;
    MMWave_ErrorLevel     errorLevel;
    int16_t               mmwErr, subErr;
    int32_t               errCode;
    (void)argc; (void)argv;

    if (!gSensorOpened) {
        CLI_getMMWaveExtensionOpenConfig(&gOpenCfg);
        gOpenCfg.freqLimitLow                = 600U;
        gOpenCfg.freqLimitHigh               = 640U;
        gOpenCfg.calibMonTimeUnit            = 1;
        gOpenCfg.useCustomCalibration        = false;
        gOpenCfg.customCalibrationEnableMask = 0x0U;
        gOpenCfg.disableFrameStartAsyncEvent = false;
        gOpenCfg.disableFrameStopAsyncEvent  = false;
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

    /* Data path: ADCBUF format + EDMA channel, then arm the rolling buffer. */
    if (l3_configAdcBuf() < 0) {
        CLI_write("Error: ADCBUF config failed\n");
        return -1;
    }
    if (l3_configEdma() < 0) {
        CLI_write("Error: EDMA config failed\n");
        return -1;
    }
    gHead        = 0U;
    gNumChirp    = 0U;
    gNumEdma     = 0U;
    gNumFrame    = 0U;
    gEdmaBusy    = 0U;
    gFrozen      = 0U;
    gFreezeReq   = 0U;
    gCurrDstAddr = SOC_translateAddress((uint32_t)&g_ring[0][0],
                                        SOC_TranslateAddr_Dir_TO_EDMA, NULL);
    gCaptureActive = 1U;

    memset((void *)&calibrationCfg, 0, sizeof(calibrationCfg));
    calibrationCfg.dfeDataOutputMode                          = gCtrlCfg.dfeDataOutputMode;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = true;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = true;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

    if (MMWave_start(gMMWaveHandle, &calibrationCfg, &errCode) < 0) {
        gCaptureActive = 0U;
        MMWave_decodeError(errCode, &errorLevel, &mmwErr, &subErr);
        CLI_write("Error: MMWave_start failed [mmwave %d subsys %d]\n", mmwErr, subErr);
        return -1;
    }
    return 0;
}

static int32_t l3_cli_sensorStop(int32_t argc, char *argv[])
{
    int32_t errCode;
    (void)argc; (void)argv;
    gCaptureActive = 0U;
    MMWave_stop(gMMWaveHandle, &errCode);
    return 0;
}

/* System init task: UART, mmWave control, EDMA + ADCBUF + capture ISRs, CLI. */
static void l3_initTask(UArg arg0, UArg arg1)
{
    MMWave_InitCfg       initCfg;
    UART_Params          uartParams;
    Task_Params          taskParams;
    ADCBuf_Params        adcbufParams;
    SOC_SysIntListenerCfg socIntCfg;
    EDMA_instanceInfo_t  edmaInstanceInfo;
    static CLI_Cfg       cliCfg;
    int32_t              errCode;

    (void)arg0; (void)arg1;

    /* --- UARTs --- */
    UART_init();
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN5_PADBE, SOC_XWR68XX_PINN5_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN5_PADBE,
                            PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN4_PADBD, SOC_XWR68XX_PINN4_PADBD_MSS_UARTA_RX);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN4_PADBD,
                            PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
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

    /* --- EDMA + ADCBUF drivers --- */
    EDMA_init(0);
    gEdmaHandle = EDMA_open(0, &errCode, &edmaInstanceInfo);
    if (gEdmaHandle == NULL) {
        return;
    }
    ADCBuf_init();
    ADCBuf_Params_init(&adcbufParams);
    adcbufParams.chirpThresholdPing = 1U;
    adcbufParams.chirpThresholdPong = 1U;
    adcbufParams.continousMode      = 0U;
    adcbufParams.socHandle          = gSocHandle;
    gAdcbufHandle = ADCBuf_open(0, &adcbufParams);
    if (gAdcbufHandle == NULL) {
        return;
    }

    /* --- capture interrupts: chirp-available + frame-start --- */
    memset((void *)&socIntCfg, 0, sizeof(socIntCfg));
    socIntCfg.systemInterrupt = SOC_XWR68XX_MSS_CHIRP_AVAIL_IRQ;
    socIntCfg.listenerFxn     = l3_chirpISR;
    socIntCfg.arg             = (uintptr_t)0U;
    if (SOC_registerSysIntListener(gSocHandle, &socIntCfg, &errCode) == NULL) {
        return;
    }
    memset((void *)&socIntCfg, 0, sizeof(socIntCfg));
    socIntCfg.systemInterrupt = SOC_XWR68XX_MSS_FRAME_START_INT;
    socIntCfg.listenerFxn     = l3_frameStartISR;
    socIntCfg.arg             = (uintptr_t)0U;
    if (SOC_registerSysIntListener(gSocHandle, &socIntCfg, &errCode) == NULL) {
        return;
    }

    /* --- mmWave control (FULL, ISOLATION) --- */
    Mailbox_init(MAILBOX_TYPE_MSS);
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
    Task_Params_init(&taskParams);
    taskParams.priority  = L3_CTRL_TASK_PRIORITY;
    taskParams.stackSize = 3 * 1024;
    Task_create(l3_mmwaveCtrlTask, &taskParams, NULL);

    /* --- CLI with the mmWave extension --- */
    cliCfg.cliPrompt             = "l3dump:/>";
    cliCfg.cliBanner             = "OpenFlight L3-dump firmware (3: raw ADC capture)\n";
    cliCfg.cliUartHandle         = gCliUart;
    cliCfg.socHandle             = gSocHandle;
    cliCfg.mmWaveHandle          = gMMWaveHandle;
    cliCfg.enableMMWaveExtension = 1U;
    cliCfg.taskPriority          = L3_CLI_TASK_PRIORITY;
    cliCfg.usePolledMode         = true;
    cliCfg.tableEntry[0].cmd           = "sensorStart";
    cliCfg.tableEntry[0].helpString    = "Configure + start capture (no args)";
    cliCfg.tableEntry[0].cmdHandlerFxn = l3_cli_sensorStart;
    cliCfg.tableEntry[1].cmd           = "sensorStop";
    cliCfg.tableEntry[1].helpString    = "Stop the front-end";
    cliCfg.tableEntry[1].cmdHandlerFxn = l3_cli_sensorStop;
    cliCfg.tableEntry[2].cmd           = "l3dump";
    cliCfg.tableEntry[2].helpString    = "Stream one raw-ADC dump";
    cliCfg.tableEntry[2].cmdHandlerFxn = l3_cli_dump;
    cliCfg.tableEntry[3].cmd           = "stats";
    cliCfg.tableEntry[3].helpString    = "Report capture counters";
    cliCfg.tableEntry[3].cmdHandlerFxn = l3_cli_stats;
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
