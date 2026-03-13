#ifndef SDP_CONFIG_H
#define SDP_CONFIG_H

#include <stdint.h>

/* Sliding window size. Default 2. Must be 1-7.                          */
#ifndef SDP_WINDOW
#  define SDP_WINDOW 2
#endif

#if SDP_WINDOW < 1 || SDP_WINDOW > 7
#  error "SDP_WINDOW must be in range 1-7"
#endif

/* Sequence space: largest multiple of SDP_WINDOW that fits in 0x0-0xE. */
/* Guaranteed >= 2 * SDP_WINDOW for all valid window sizes.              */
#define SDP_SEQ_SIZE ((15 / SDP_WINDOW) * SDP_WINDOW)
#define SDP_SEQ_MAX  (SDP_SEQ_SIZE - 1)

/* Maximum payload bytes per frame. Default 255. Must be 3-255.         */
/* Minimum 3 ensures the mandatory ANNOUNCE bytes always fit.            */
#ifndef SDP_MAX_PAYLOAD
#  define SDP_MAX_PAYLOAD 255
#endif

#if SDP_MAX_PAYLOAD < 3 || SDP_MAX_PAYLOAD > 255
#  error "SDP_MAX_PAYLOAD must be in range 3-255"
#endif

/* Buffer slot: header(3) + payload + crc(2), FLAG bytes stripped.      */
#define SDP_SLOT_SIZE (SDP_MAX_PAYLOAD + 5)

/* rx_pos counter type: uint8_t is sufficient when slot fits in a byte. */
#if SDP_MAX_PAYLOAD <= 250
   typedef uint8_t  sdp_rx_pos_t;
#else
   typedef uint16_t sdp_rx_pos_t;
#endif

#ifndef SDP_SILENCE_TIMEOUT_MS
#  define SDP_SILENCE_TIMEOUT_MS 30000U
#endif

#endif /* SDP_CONFIG_H */
