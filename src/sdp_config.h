#ifndef SDP_CONFIG_H
#define SDP_CONFIG_H

#ifndef SDP_WINDOW
#  define SDP_WINDOW 2
#endif

#if SDP_WINDOW < 1 || SDP_WINDOW > 7
#  error "SDP_WINDOW must be in range 1-7"
#endif

#ifndef SDP_SILENCE_TIMEOUT_MS
#  define SDP_SILENCE_TIMEOUT_MS 30000U
#endif

#endif /* SDP_CONFIG_H */
