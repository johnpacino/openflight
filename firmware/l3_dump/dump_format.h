/* Stage-1c L3 burst-dump wire format.
 *
 * MUST stay byte-for-byte in sync with iwr6843_l3dump.py (HEADER =
 * struct.Struct("<4sHHHBBHBBHH")). Little-endian. 20-byte header, then payload:
 *   for each frame, each chirp, each rx: n_samples * (int16 I, int16 Q).
 * Chirp order is TDM-interleaved: chirp c -> tx = c % n_tx, loop = c / n_tx.
 */
#ifndef L3_DUMP_FORMAT_H
#define L3_DUMP_FORMAT_H

#include <stdint.h>

#define L3_DUMP_MAGIC       "ILD1"
#define L3_DUMP_VERSION     1
#define L3_SAMPLE_INT16_IQ  0

typedef struct __attribute__((packed)) {
    char     magic[4];          /* "ILD1" */
    uint16_t version;           /* L3_DUMP_VERSION */
    uint16_t n_frames;          /* frames in this dump window */
    uint16_t chirps_per_frame;  /* n_tx * loops */
    uint8_t  n_tx;
    uint8_t  n_rx;
    uint16_t n_samples;         /* ADC samples per chirp */
    uint8_t  sample_fmt;        /* L3_SAMPLE_INT16_IQ */
    uint8_t  _pad;
    uint16_t trigger_frame;     /* window index that straddles the trigger */
    uint16_t _pad2;
} l3_dump_header_t;             /* sizeof == 20 */

#endif /* L3_DUMP_FORMAT_H */
