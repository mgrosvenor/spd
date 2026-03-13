#ifndef SDP_H
#define SDP_H

#include <stdint.h>
#include "sdp_config.h"

/* ── Compile-time constants ─────────────────────────────────────────────── */
#define SDP_FLAG_BYTE       0x7E
#define SDP_ESC_BYTE        0x7D
#define SDP_SYNC_BYTE       0xAA
#define SDP_MAX_PAYLOAD     128
#define SDP_SLOT_SIZE       133  /* header(3) + payload(128) + crc(2) */
#define SDP_SEQ_NONE        0xF  /* startup sentinel */
#define SDP_SEQ_MAX         0xE  /* highest usable SEQ */
#define SDP_FLAG_RTX        0x02

/* ── Transport message types ────────────────────────────────────────────── */
#define SDP_T_ANNOUNCE      0x00
#define SDP_T_ACK           0x30
#define SDP_T_NAK           0x31
#define SDP_T_RESET         0x32
#define SDP_T_UNSUPPORTED   0x3C

/* ── Handshake constants ────────────────────────────────────────────────── */
#define SDP_SYNC_COUNT      8    /* consecutive 0xAA bytes required */
#define SDP_SYNC_SEND       8    /* host sends 8 */
#define SDP_HOLDOFF_MS      40U
#define SDP_ACK_TIMEOUT_MS  50U
#define SDP_CONNECT_RETRY_MS 20U

/* ── Status codes ───────────────────────────────────────────────────────── */
typedef enum {
    SDP_OK                  =  0,
    SDP_ERR_WINDOW_FULL     = -1,
    SDP_ERR_PAYLOAD_TOO_LARGE = -2,
    SDP_ERR_NOT_LINKED      = -3,
    SDP_ERR_ALREADY_LINKED  = -4,
    SDP_ERR_TIMEOUT         = -5,
    SDP_ERR_BAD_ARG         = -6,
    SDP_ERR_HAL             = -7
} sdp_status_t;

/* ── Link state ─────────────────────────────────────────────────────────── */
typedef enum {
    SDP_STATE_SYNC_HUNT = 0,
    SDP_STATE_SYNC_ACK,
    SDP_STATE_SYNC_WAIT,
    SDP_STATE_LINKED
} sdp_link_state_t;

/* ── Frame ──────────────────────────────────────────────────────────────── */
typedef struct {
    uint8_t type;
    uint8_t flags;
    uint8_t seq;
    uint8_t ackseq;
    uint8_t len;
    uint8_t payload[SDP_MAX_PAYLOAD];
} sdp_frame_t;

/* ── HAL ────────────────────────────────────────────────────────────────── */
typedef struct {
    int      (*write)(const uint8_t *buf, uint8_t len, void *user);
    int      (*read)(void *user);
    uint32_t (*millis)(void *user);
    void     *user;
} sdp_hal_t;

/* ── Diagnostics ────────────────────────────────────────────────────────── */
typedef struct {
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t crc_errors;
    uint32_t rx_overflows;
    uint32_t drop_count;
    uint32_t nak_sent;
    uint32_t nak_received;
    uint32_t window_full;
    uint32_t hal_errors;
} sdp_diag_t;

/* Forward declarations */
typedef struct sdp_ctx_s sdp_ctx_t;

/* ── Callbacks ──────────────────────────────────────────────────────────── */
typedef void        (*sdp_on_frame_t)(sdp_ctx_t *ctx, const sdp_frame_t *frame, void *user);
typedef sdp_status_t (*sdp_on_build_announce_t)(uint8_t *payload, uint8_t *len, void *user);
typedef void        (*sdp_on_link_lost_t)(sdp_ctx_t *ctx, void *user);

/* ── Context ────────────────────────────────────────────────────────────── */
struct sdp_ctx_s {
    /* HAL and callbacks */
    const sdp_hal_t        *hal;
    sdp_on_frame_t          on_frame;
    sdp_on_build_announce_t on_build_announce;
    sdp_on_link_lost_t      on_link_lost;
    void                   *user;

    /* Link state */
    sdp_link_state_t  state;

    /* TX state */
    uint8_t  tx_seq;
    uint8_t  tx_unacked;     /* frames in flight */
    uint8_t  rx_ackseq;      /* last received SEQ from peer (for piggyback) */
    uint8_t  peer_ackseq;    /* last ACKSEQ from peer (for window tracking) */

    /* ACK pending */
    uint8_t  ack_pending;
    uint32_t last_rx_ms;     /* millis of last validated RX frame */
    uint32_t last_ack_ms;    /* millis of last outgoing piggyback/ACK */

    /* Sync hunt — only active on device side (sdp_listen sets this) */
    uint8_t  sync_hunt_enabled; /* 1 = count 0xAA bytes (device mode) */
    uint8_t  sync_count;
    uint32_t holdoff_start;

    /* RX parser */
    uint8_t  rx_buf[SDP_SLOT_SIZE];
    uint8_t  rx_pos;
    uint8_t  rx_escaped;
    uint8_t  rx_in_frame;

    /* Diagnostics */
    sdp_diag_t diag;
};

/* ── API ────────────────────────────────────────────────────────────────── */
sdp_status_t sdp_init(sdp_ctx_t               *ctx,
                       const sdp_hal_t          *hal,
                       sdp_on_frame_t            on_frame,
                       sdp_on_build_announce_t   on_build_announce,
                       void                     *user);

sdp_status_t sdp_connect(sdp_ctx_t *ctx, uint32_t timeout_ms, sdp_frame_t *out_announce);
sdp_status_t sdp_listen(sdp_ctx_t *ctx);

sdp_status_t sdp_send(sdp_ctx_t *ctx, uint8_t type, uint8_t flags,
                       const uint8_t *payload, uint8_t len);
sdp_status_t sdp_send_announce(sdp_ctx_t *ctx);

sdp_status_t sdp_rx_byte(sdp_ctx_t *ctx, uint8_t byte);
sdp_status_t sdp_poll(sdp_ctx_t *ctx);
sdp_status_t sdp_tick(sdp_ctx_t *ctx);
sdp_status_t sdp_reset(sdp_ctx_t *ctx);

sdp_status_t sdp_state(const sdp_ctx_t *ctx, sdp_link_state_t *out_state);
sdp_status_t sdp_get_diag(const sdp_ctx_t *ctx, sdp_diag_t *out);
sdp_status_t sdp_set_link_lost_cb(sdp_ctx_t *ctx, sdp_on_link_lost_t on_link_lost);

#endif /* SDP_H */
