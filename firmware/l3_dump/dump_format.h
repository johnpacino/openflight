/* Stage-1c L3 burst-dump wire format.
 *
 * MUST stay byte-for-byte in sync with iwr6843_l3dump.py (HEADER =
 * struct.Struct("<4sHHHBBHBBHH")). Little-endian. 20-byte header, then payload:
 *   raw fmt:
 *     for each frame, each chirp, each rx: n_samples * (int16 I, int16 Q).
 *   range-bin fmt 1:
 *     for each frame, each chirp, each rx: n_samples selected range bins *
 *     (int16 I, int16 Q), where _pad carries the first range-bin index.
 *   range-bin fmt 2 (version 4+):
 *     n_frames uint8 start-bin values immediately follow this header, then
 *     the same frame/chirp/rx/IQ payload. Each frame can therefore retain a
 *     different contiguous range window without making the decoder guess.
 * Chirp order is TDM-interleaved: chirp c -> tx = c % n_tx, loop = c / n_tx.
 */
#ifndef L3_DUMP_FORMAT_H
#define L3_DUMP_FORMAT_H

#include <stdint.h>

#define L3_DUMP_MAGIC       "ILD1"
#define L3_DUMP_VERSION     3   /* v2: trigger_frame = oldest ring slot;
                                 * v3: frame_period_us populated */
#define L3_DUMP_VERSION_WINDOWED 4
#define L3_SAMPLE_INT16_IQ  0
#define L3_SAMPLE_RANGE_FFT_IQ16 1
#define L3_SAMPLE_RANGE_FFT_IQ16_WINDOWED 2

typedef struct __attribute__((packed)) {
    char     magic[4];          /* "ILD1" */
    uint16_t version;           /* L3_DUMP_VERSION */
    uint16_t n_frames;          /* frames in this dump window */
    uint16_t chirps_per_frame;  /* n_tx * loops */
    uint8_t  n_tx;
    uint8_t  n_rx;
    uint16_t n_samples;         /* ADC samples per chirp */
    uint8_t  sample_fmt;        /* L3_SAMPLE_INT16_IQ */
    uint8_t  _pad;              /* fmt 1: first range-bin index */
    uint16_t trigger_frame;     /* v2+: OLDEST ring slot = time-order start
                                 * (slots stream in memory order; rotate by
                                 * this to get chronological frames). 0 in v1. */
    uint16_t frame_period_us;   /* v3+: frame periodicity in microseconds
                                 * (self-describing timing for the range-walk
                                 * speed fit). 0 in v1/v2 dumps (was padding). */
} l3_dump_header_t;             /* sizeof == 20 */

#endif /* L3_DUMP_FORMAT_H */
