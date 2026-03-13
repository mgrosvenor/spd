/*
 * sdp.h — Serial Datagram Protocol, single-header library
 *
 * SDP v1.1 — HDLC-framed, CRC-16/CCITT-FALSE, sequenced, flow-controlled
 * transport for low-bit-rate serial links (UART, RS-485, USB CDC).
 *
 * ── Usage ────────────────────────────────────────────────────────────────────
 *
 * In exactly ONE translation unit, before including this header:
 *
 *     #define SDP_IMPLEMENTATION
 *     #include "sdp.h"
 *
 * In all other translation units (declarations only):
 *
 *     #include "sdp.h"
 *
 * ── Configuration ────────────────────────────────────────────────────────────
 *
 * Define any of the following before the first include, or pass via -D:
 *
 *     SDP_WINDOW              Sliding window size (1–7, default 2)
 *     SDP_MAX_PAYLOAD         Max payload bytes (3–255, default 255)
 *     SDP_SILENCE_TIMEOUT_MS  Link silence timeout ms (default 30 000)
 *
 * ─────────────────────────────────────────────────────────────────────────────
 */

#ifndef SDP_H
#define SDP_H

#include <stdint.h>

/* ════════════════════════════════════════════════════════════════════════════
 * CONFIGURATION
 * ════════════════════════════════════════════════════════════════════════════ */

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

/* ════════════════════════════════════════════════════════════════════════════
 * DECLARATIONS
 * ════════════════════════════════════════════════════════════════════════════ */

/* ── Wire constants ─────────────────────────────────────────────────────── */
#define SDP_FLAG_BYTE        0x7E
#define SDP_ESC_BYTE         0x7D
#define SDP_SYNC_BYTE        0xAA
#define SDP_SEQ_NONE         0xF  /* startup sentinel */
#define SDP_FLAG_RTX         0x02
#define SDP_PROTOCOL_VERSION 1

/* ── Transport message types ────────────────────────────────────────────── */
#define SDP_T_ANNOUNCE      0x00
#define SDP_T_ACK           0x30
#define SDP_T_NAK           0x31
#define SDP_T_RESET         0x32
#define SDP_T_STATS_REQ     0x33
#define SDP_T_STATS_RSP     0x34
#define SDP_T_UNSUPPORTED   0x3C

/* ── Handshake constants ────────────────────────────────────────────────── */
#define SDP_SYNC_COUNT       8
#define SDP_SYNC_SEND        8
#define SDP_HOLDOFF_MS       40U
#define SDP_ACK_TIMEOUT_MS   50U
#define SDP_CONNECT_RETRY_MS 20U

/* ── Status codes ───────────────────────────────────────────────────────── */
typedef enum {
    SDP_OK                    =  0,
    SDP_ERR_WINDOW_FULL       = -1,
    SDP_ERR_PAYLOAD_TOO_LARGE = -2,
    SDP_ERR_NOT_LINKED        = -3,
    SDP_ERR_ALREADY_LINKED    = -4,
    SDP_ERR_TIMEOUT           = -5,
    SDP_ERR_BAD_ARG           = -6,
    SDP_ERR_HAL               = -7
} sdp_status_t;

/* ── Link state ─────────────────────────────────────────────────────────── */
typedef enum {
    SDP_STATE_SYNC_HUNT = 0,
    SDP_STATE_SYNC_ACK,
    SDP_STATE_SYNC_WAIT,
    SDP_STATE_LINKED
} sdp_link_state_t;

/* ── TX buffer slot (one per in-flight frame, for retransmission) ────────── */
typedef struct {
    uint8_t seq;
    uint8_t type;
    uint8_t flags;
    uint8_t len;
    uint8_t payload[SDP_MAX_PAYLOAD];
} sdp_tx_slot_t;

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

/* Forward declaration */
typedef struct sdp_ctx_s sdp_ctx_t;

/* ── Callbacks ──────────────────────────────────────────────────────────── */
typedef void         (*sdp_on_frame_t)(sdp_ctx_t *ctx, const sdp_frame_t *frame, void *user);
typedef sdp_status_t (*sdp_on_build_announce_t)(uint8_t *payload, uint8_t *len, void *user);
typedef void         (*sdp_on_link_lost_t)(sdp_ctx_t *ctx, void *user);

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
    uint8_t       tx_seq;
    uint8_t       rx_ackseq;      /* last received SEQ from peer (for piggyback) */
    uint8_t       peer_ackseq;    /* last ACKSEQ from peer (for window tracking) */
    sdp_tx_slot_t tx_ring[SDP_WINDOW];

    /* ACK pending */
    uint8_t  ack_pending;
    uint32_t last_rx_ms;
    uint32_t last_ack_ms;

    /* Sync hunt (device mode) */
    uint8_t  sync_hunt_enabled;
    uint8_t  sync_count;
    uint32_t holdoff_start;

    /* TX ring index (increments mod SDP_WINDOW; avoids division in sdp_send) */
    uint8_t       tx_ring_idx;

    /* Assembled RX frame (kept in ctx to avoid large stack locals) */
    sdp_frame_t   rx_frame;

    /* RX parser */
    uint8_t       rx_buf[SDP_SLOT_SIZE];
    sdp_rx_pos_t  rx_pos;
    uint8_t       rx_escaped;
    uint8_t       rx_in_frame;

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

sdp_status_t sdp_request_stats(sdp_ctx_t *ctx);
sdp_status_t sdp_diag_from_stats_rsp(const sdp_frame_t *frame, sdp_diag_t *out);

/* ════════════════════════════════════════════════════════════════════════════
 * IMPLEMENTATION
 * ════════════════════════════════════════════════════════════════════════════ */

#ifdef SDP_IMPLEMENTATION

#include <string.h>

/* ── Internal helpers ───────────────────────────────────────────────────── */

/*
 * CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF) using a 16-entry nibble
 * table. Two table lookups per byte; 32 bytes of ROM. On 8-bit MCUs this
 * is significantly cheaper than the 8-iteration bit-by-bit alternative.
 */
static const uint16_t crc16_tbl[16] = {
    0x0000u, 0x1021u, 0x2042u, 0x3063u,
    0x4084u, 0x50A5u, 0x60C6u, 0x70E7u,
    0x8108u, 0x9129u, 0xA14Au, 0xB16Bu,
    0xC18Cu, 0xD1ADu, 0xE1CEu, 0xF1EFu
};

/* len uses sdp_rx_pos_t (uint8_t or uint16_t) so it can hold values up to
 * SDP_MAX_PAYLOAD+3 (up to 258 for the default SDP_MAX_PAYLOAD=255). */
static uint16_t crc16_feed(uint16_t crc, const uint8_t *data, sdp_rx_pos_t len)
{
    sdp_rx_pos_t i;
    for (i = 0; i < len; i++) {
        crc = (uint16_t)((crc << 4) ^ crc16_tbl[(crc >> 12) ^ (data[i] >> 4)]);
        crc = (uint16_t)((crc << 4) ^ crc16_tbl[(crc >> 12) ^ (data[i] & 0x0Fu)]);
    }
    return crc;
}

#define crc16(data, len)             crc16_feed(0xFFFFu, (data), (len))
#define crc16_extend(crc, data, len) crc16_feed((crc),   (data), (len))

/* Forward declaration — defined in the Receiving section below. */
static void rx_byte_inner(sdp_ctx_t *ctx, uint8_t byte);

/*
 * Number of unacknowledged frames in flight.
 * tx_seq = next SEQ to use; peer_ackseq = last ACKSEQ received from peer.
 * Uses a conditional subtract instead of modulo — no division on 8-bit MCU.
 */
static uint8_t seq_unacked(uint8_t tx_seq, uint8_t peer_ackseq)
{
    uint8_t n;
    if (peer_ackseq == SDP_SEQ_NONE)
        return tx_seq;
    n = (uint8_t)(tx_seq + SDP_SEQ_SIZE - peer_ackseq - 1u);
    if (n >= SDP_SEQ_SIZE) n = (uint8_t)(n - SDP_SEQ_SIZE);
    return n;
}

/* Write one byte with HDLC escaping. Returns 0 on success, -1 on HAL error. */
static int write_escaped(const sdp_ctx_t *ctx, uint8_t b)
{
    if (b == SDP_FLAG_BYTE || b == SDP_ESC_BYTE) {
        uint8_t esc[2];
        esc[0] = SDP_ESC_BYTE;
        esc[1] = (uint8_t)(b ^ 0x20u);
        return ctx->hal->write(esc, 2, ctx->hal->user);
    }
    return ctx->hal->write(&b, 1, ctx->hal->user);
}

/*
 * Build and transmit a raw frame. Does NOT check link state or window.
 * Returns SDP_OK or SDP_ERR_HAL.
 */
static sdp_status_t transmit_frame(sdp_ctx_t *ctx, uint8_t type, uint8_t flags,
                                    uint8_t seq, uint8_t ackseq,
                                    const uint8_t *payload, uint8_t len)
{
    uint8_t  flag     = SDP_FLAG_BYTE;
    uint8_t  seq_ack  = (uint8_t)((seq << 4) | (ackseq & 0x0Fu));
    uint8_t  type_flg = (uint8_t)((type << 2) | (flags & 0x03u));
    uint8_t  hdr[3];
    uint16_t crc;
    uint8_t  crc_hi, crc_lo;
    uint8_t  i;

    hdr[0] = seq_ack;
    hdr[1] = type_flg;
    hdr[2] = len;
    crc = crc16(hdr, 3);
    if (payload && len > 0)
        crc = crc16_extend(crc, payload, len);

    if (ctx->hal->write(&flag, 1, ctx->hal->user) != 0) goto hal_err;
    if (write_escaped(ctx, seq_ack)  != 0) goto hal_err;
    if (write_escaped(ctx, type_flg) != 0) goto hal_err;
    if (write_escaped(ctx, len)      != 0) goto hal_err;
    for (i = 0; i < len; i++)
        if (write_escaped(ctx, payload[i]) != 0) goto hal_err;
    crc_hi = (uint8_t)(crc >> 8);
    crc_lo = (uint8_t)(crc & 0xFFu);
    if (write_escaped(ctx, crc_hi) != 0) goto hal_err;
    if (write_escaped(ctx, crc_lo) != 0) goto hal_err;
    if (ctx->hal->write(&flag, 1, ctx->hal->user) != 0) goto hal_err;

    ctx->diag.tx_frames++;
    return SDP_OK;

hal_err:
    ctx->diag.hal_errors++;
    return SDP_ERR_HAL;
}

static sdp_status_t send_ack(sdp_ctx_t *ctx)
{
    sdp_status_t st = transmit_frame(ctx, SDP_T_ACK, 0,
                                     ctx->tx_seq, ctx->rx_ackseq,
                                     NULL, 0);
    if (st == SDP_OK) {
        ctx->last_ack_ms = ctx->hal->millis(ctx->hal->user);
        ctx->ack_pending = 0;
    }
    return st;
}

static sdp_status_t send_nak(sdp_ctx_t *ctx, uint8_t bad_seq_ack)
{
    sdp_status_t st = transmit_frame(ctx, SDP_T_NAK, 0,
                                     ctx->tx_seq, ctx->rx_ackseq,
                                     &bad_seq_ack, 1);
    if (st == SDP_OK) ctx->diag.nak_sent++;
    else ctx->diag.hal_errors++;
    return st;
}

/* Process a fully assembled, unescaped frame in rx_buf[0..rx_pos-1].
 * Decoded frame is written into ctx->rx_frame (no large stack local). */
static void process_frame(sdp_ctx_t *ctx)
{
    uint8_t   len_field, seq, ackseq, type, flags;
    uint16_t  expected, actual;
    uint32_t  now;

    if (ctx->rx_pos < 5) return;

    len_field = ctx->rx_buf[2];
    if (len_field > SDP_MAX_PAYLOAD) {
        ctx->diag.crc_errors++;
        return;
    }
    if (ctx->rx_pos != (sdp_rx_pos_t)(len_field + 5u)) {
        ctx->diag.crc_errors++;
        return;
    }

    expected = ((uint16_t)ctx->rx_buf[ctx->rx_pos - 2] << 8)
             | ctx->rx_buf[ctx->rx_pos - 1];
    /* rx_pos - 2 can exceed 255 for large payloads; do not cast to uint8_t */
    actual   = crc16(ctx->rx_buf, ctx->rx_pos - 2u);

    if (actual != expected) {
        ctx->diag.crc_errors++;
        send_nak(ctx, ctx->rx_buf[0]);
        return;
    }

    seq    = ctx->rx_buf[0] >> 4;
    ackseq = ctx->rx_buf[0] & 0x0Fu;
    type   = ctx->rx_buf[1] >> 2;
    flags  = ctx->rx_buf[1] & 0x03u;

    /* Capture millis() once; reused below to avoid a second HAL call. */
    now = ctx->hal->millis(ctx->hal->user);

    ctx->peer_ackseq = ackseq;
    ctx->rx_ackseq   = seq;
    ctx->ack_pending = 1;
    ctx->last_rx_ms  = now;
    ctx->diag.rx_frames++;

    ctx->rx_frame.type   = type;
    ctx->rx_frame.flags  = flags;
    ctx->rx_frame.seq    = seq;
    ctx->rx_frame.ackseq = ackseq;
    ctx->rx_frame.len    = len_field;
    if (len_field > 0)
        memcpy(ctx->rx_frame.payload, ctx->rx_buf + 3, len_field);

    if (ctx->state == SDP_STATE_SYNC_ACK) {
        ctx->state       = SDP_STATE_LINKED;
        ctx->tx_seq      = 0;
        ctx->peer_ackseq = SDP_SEQ_NONE;
    }

    switch (type) {
    case SDP_T_ACK:
        ctx->last_ack_ms = now;
        ctx->ack_pending = 0;
        return;

    case SDP_T_RESET:
        ctx->tx_seq      = 0;
        ctx->rx_ackseq   = SDP_SEQ_NONE;
        ctx->peer_ackseq = SDP_SEQ_NONE;
        transmit_frame(ctx, SDP_T_RESET, 0, 0, SDP_SEQ_NONE, NULL, 0);
        return;

    case SDP_T_NAK: {
        /* Linear scan avoids modulo; at most SDP_WINDOW (1-7) iterations. */
        uint8_t bad_seq, i;
        ctx->diag.nak_received++;
        if (ctx->rx_frame.len >= 1) {
            bad_seq = ctx->rx_frame.payload[0] >> 4;
            for (i = 0; i < SDP_WINDOW; i++) {
                if (ctx->tx_ring[i].seq == bad_seq) {
                    transmit_frame(ctx, ctx->tx_ring[i].type,
                                   (uint8_t)(ctx->tx_ring[i].flags | SDP_FLAG_RTX),
                                   ctx->tx_ring[i].seq, ctx->rx_ackseq,
                                   ctx->tx_ring[i].payload, ctx->tx_ring[i].len);
                    break;
                }
            }
        }
        ctx->on_frame(ctx, &ctx->rx_frame, ctx->user);
        return;
    }

    case SDP_T_STATS_REQ: {
        /* Pack local diagnostics into a T_STATS_RSP payload (little-endian).
         * WR32 writes each uint32_t field via pointer increment — no intermediate
         * 36-byte stack array, no i*4 multiply in the inner loop. */
        uint8_t *p = ctx->rx_frame.payload;
        #define WR32(v) do { uint32_t _v = (v); \
            *p++ = (uint8_t)(_v);         *p++ = (uint8_t)((_v) >>  8); \
            *p++ = (uint8_t)((_v) >> 16); *p++ = (uint8_t)((_v) >> 24); \
        } while(0)
        WR32(ctx->diag.rx_frames);
        WR32(ctx->diag.tx_frames);
        WR32(ctx->diag.crc_errors);
        WR32(ctx->diag.rx_overflows);
        WR32(ctx->diag.drop_count);
        WR32(ctx->diag.nak_sent);
        WR32(ctx->diag.nak_received);
        WR32(ctx->diag.window_full);
        WR32(ctx->diag.hal_errors);
        #undef WR32
        transmit_frame(ctx, SDP_T_STATS_RSP, 0,
                       ctx->tx_seq, ctx->rx_ackseq, ctx->rx_frame.payload, 36);
        return;
    }

    case SDP_T_STATS_RSP:
        /* Delivered to on_frame; host unpacks with sdp_diag_from_stats_rsp. */
        ctx->on_frame(ctx, &ctx->rx_frame, ctx->user);
        return;

    case SDP_T_UNSUPPORTED:
        ctx->on_frame(ctx, &ctx->rx_frame, ctx->user);
        return;

    default:
        ctx->on_frame(ctx, &ctx->rx_frame, ctx->user);
        return;
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

sdp_status_t sdp_init(sdp_ctx_t               *ctx,
                       const sdp_hal_t          *hal,
                       sdp_on_frame_t            on_frame,
                       sdp_on_build_announce_t   on_build_announce,
                       void                     *user)
{
    if (!ctx || !hal || !hal->write || !hal->millis || !on_frame || !on_build_announce)
        return SDP_ERR_BAD_ARG;

    memset(ctx, 0, sizeof(*ctx));
    ctx->hal               = hal;
    ctx->on_frame          = on_frame;
    ctx->on_build_announce = on_build_announce;
    ctx->user              = user;
    ctx->state             = SDP_STATE_SYNC_HUNT;
    ctx->rx_ackseq         = SDP_SEQ_NONE;
    ctx->peer_ackseq       = SDP_SEQ_NONE;
    return SDP_OK;
}

sdp_status_t sdp_set_link_lost_cb(sdp_ctx_t *ctx, sdp_on_link_lost_t on_link_lost)
{
    if (!ctx) return SDP_ERR_BAD_ARG;
    ctx->on_link_lost = on_link_lost;
    return SDP_OK;
}

sdp_status_t sdp_state(const sdp_ctx_t *ctx, sdp_link_state_t *out_state)
{
    if (!ctx || !out_state) return SDP_ERR_BAD_ARG;
    *out_state = ctx->state;
    return SDP_OK;
}

sdp_status_t sdp_get_diag(const sdp_ctx_t *ctx, sdp_diag_t *out)
{
    if (!ctx || !out) return SDP_ERR_BAD_ARG;
    *out = ctx->diag;
    return SDP_OK;
}

/* ── Link establishment ─────────────────────────────────────────────────── */

/*
 * Build and transmit an ANNOUNCE frame.
 * Bytes 0-2 are the mandatory transport header (protocol_version, window,
 * max_payload). The on_build_announce callback fills application bytes
 * starting at byte 3.
 */
sdp_status_t sdp_send_announce(sdp_ctx_t *ctx)
{
    uint8_t      app_len = 0;
    sdp_status_t st;

    if (!ctx) return SDP_ERR_BAD_ARG;

    /* Build the announce payload directly into ctx->rx_frame.payload.
     * This is safe: the bytes are transmitted immediately and the buffer
     * is only used for reception, which cannot interleave on a single thread.
     * Eliminates a uint8_t payload[SDP_MAX_PAYLOAD] stack allocation (≤255 B). */
    ctx->rx_frame.payload[0] = SDP_PROTOCOL_VERSION;
    ctx->rx_frame.payload[1] = SDP_WINDOW;
    ctx->rx_frame.payload[2] = SDP_MAX_PAYLOAD;

    st = ctx->on_build_announce(ctx->rx_frame.payload + 3, &app_len, ctx->user);
    if (st != SDP_OK) return st;

    if ((uint16_t)app_len + 3u > SDP_MAX_PAYLOAD) return SDP_ERR_PAYLOAD_TOO_LARGE;

    return transmit_frame(ctx, SDP_T_ANNOUNCE, 0, 0, SDP_SEQ_NONE,
                          ctx->rx_frame.payload, (uint8_t)(app_len + 3u));
}

sdp_status_t sdp_listen(sdp_ctx_t *ctx)
{
    if (!ctx) return SDP_ERR_BAD_ARG;
    if (ctx->state == SDP_STATE_LINKED) return SDP_ERR_ALREADY_LINKED;

    ctx->state             = SDP_STATE_SYNC_HUNT;
    ctx->sync_hunt_enabled = 1;
    ctx->sync_count        = 0;
    ctx->rx_ackseq         = SDP_SEQ_NONE;
    ctx->peer_ackseq       = SDP_SEQ_NONE;
    ctx->tx_seq            = 0;
    return SDP_OK;
}

typedef struct {
    sdp_on_frame_t  original_on_frame;
    void           *original_user;
    sdp_frame_t    *out_announce;
    int             got_announce;
} connect_ctx_t;

static void connect_on_frame(sdp_ctx_t *ctx, const sdp_frame_t *frame, void *user)
{
    connect_ctx_t *cc = (connect_ctx_t *)user;
    if (frame->type == SDP_T_ANNOUNCE) {
        if (cc->out_announce)
            *cc->out_announce = *frame;
        cc->got_announce = 1;
        ctx->state       = SDP_STATE_LINKED;
        ctx->tx_seq      = 0;
        ctx->peer_ackseq = SDP_SEQ_NONE;
        ctx->rx_ackseq   = SDP_SEQ_NONE;
    }
    cc->original_on_frame(ctx, frame, cc->original_user);
}

sdp_status_t sdp_connect(sdp_ctx_t *ctx, uint32_t timeout_ms, sdp_frame_t *out_announce)
{
    uint8_t sync[SDP_SYNC_SEND];
    uint32_t start, last_top;
    connect_ctx_t cc;

    if (!ctx || timeout_ms == 0 || !ctx->hal->read) return SDP_ERR_BAD_ARG;
    if (ctx->state == SDP_STATE_LINKED) return SDP_ERR_ALREADY_LINKED;

    cc.original_on_frame = ctx->on_frame;
    cc.original_user     = ctx->user;
    cc.out_announce      = out_announce;
    cc.got_announce      = 0;
    ctx->on_frame = connect_on_frame;
    ctx->user     = &cc;

    memset(sync, SDP_SYNC_BYTE, SDP_SYNC_SEND);
    if (ctx->hal->write(sync, SDP_SYNC_SEND, ctx->hal->user) != 0) {
        ctx->diag.hal_errors++;
        ctx->on_frame = cc.original_on_frame;
        ctx->user     = cc.original_user;
        return SDP_ERR_HAL;
    }

    start    = ctx->hal->millis(ctx->hal->user);
    last_top = start;

    for (;;) {
        uint32_t now = ctx->hal->millis(ctx->hal->user);
        int b;

        if ((now - start) >= timeout_ms) {
            ctx->on_frame = cc.original_on_frame;
            ctx->user     = cc.original_user;
            return SDP_ERR_TIMEOUT;
        }

        b = ctx->hal->read(ctx->hal->user);
        if (b >= 0)
            rx_byte_inner(ctx, (uint8_t)b);

        if (cc.got_announce) {
            ctx->on_frame = cc.original_on_frame;
            ctx->user     = cc.original_user;
            return SDP_OK;
        }

        now = ctx->hal->millis(ctx->hal->user);
        if ((now - last_top) >= SDP_CONNECT_RETRY_MS) {
            uint8_t top = SDP_SYNC_BYTE;
            ctx->hal->write(&top, 1, ctx->hal->user);
            last_top = now;
        }
    }
}

/* ── Sending ────────────────────────────────────────────────────────────── */

sdp_status_t sdp_send(sdp_ctx_t *ctx, uint8_t type, uint8_t flags,
                       const uint8_t *payload, uint8_t len)
{
    sdp_status_t  st;
    sdp_tx_slot_t *slot;

    if (!ctx) return SDP_ERR_BAD_ARG;
    if (payload == NULL && len > 0) return SDP_ERR_BAD_ARG;
    if (len > SDP_MAX_PAYLOAD) return SDP_ERR_PAYLOAD_TOO_LARGE;
    if (ctx->state != SDP_STATE_LINKED) return SDP_ERR_NOT_LINKED;

    if (seq_unacked(ctx->tx_seq, ctx->peer_ackseq) >= SDP_WINDOW) {
        ctx->diag.window_full++;
        return SDP_ERR_WINDOW_FULL;
    }

    /* Store in TX ring for retransmission on T_NAK.
     * tx_ring_idx is a separate counter that wraps at SDP_WINDOW,
     * eliminating the tx_seq % SDP_WINDOW division on every send. */
    slot        = &ctx->tx_ring[ctx->tx_ring_idx];
    slot->seq   = ctx->tx_seq;
    slot->type  = type;
    slot->flags = flags;
    slot->len   = len;
    if (len > 0) memcpy(slot->payload, payload, len);

    st = transmit_frame(ctx, type, flags, ctx->tx_seq, ctx->rx_ackseq, payload, len);
    if (st != SDP_OK) return st;

    ctx->tx_ring_idx = (ctx->tx_ring_idx + 1u < SDP_WINDOW)
                       ? (uint8_t)(ctx->tx_ring_idx + 1u) : 0u;
    ctx->tx_seq      = (ctx->tx_seq >= SDP_SEQ_MAX) ? 0u : (uint8_t)(ctx->tx_seq + 1u);
    ctx->last_ack_ms = ctx->hal->millis(ctx->hal->user);
    ctx->ack_pending = 0;
    return SDP_OK;
}

/* ── Receiving ──────────────────────────────────────────────────────────── */

/*
 * Inner receive byte function — no null check, void return.
 * Called from sdp_poll's tight loop (inlined by compiler) and from
 * sdp_connect's handshake loop.
 * sdp_rx_byte (public API) is a thin checked wrapper around this.
 */
static void rx_byte_inner(sdp_ctx_t *ctx, uint8_t byte)
{
    if (ctx->sync_hunt_enabled) {
        if (ctx->state == SDP_STATE_SYNC_HUNT) {
            if (byte == SDP_SYNC_BYTE) {
                ctx->sync_count++;
                if (ctx->sync_count >= SDP_SYNC_COUNT) {
                    sdp_send_announce(ctx);
                    ctx->state         = SDP_STATE_SYNC_ACK;
                    ctx->holdoff_start = ctx->hal->millis(ctx->hal->user);
                    ctx->sync_count    = 0;
                }
                return;
            } else if (byte != SDP_FLAG_BYTE) {
                ctx->sync_count = 0;
                return;
            }
        } else if (ctx->state == SDP_STATE_SYNC_ACK) {
            /* Discard host sync retop-ups that arrive before the first frame.
             * Once inside a frame (rx_in_frame=1), 0xAA is legitimate payload
             * data and must not be dropped. */
            if (byte == SDP_SYNC_BYTE && !ctx->rx_in_frame) return;
        }
    }

    if (byte == SDP_FLAG_BYTE) {
        if (ctx->rx_in_frame && ctx->rx_pos >= 5)
            process_frame(ctx);
        ctx->rx_in_frame = 1;
        ctx->rx_pos      = 0;
        ctx->rx_escaped  = 0;
        return;
    }

    if (!ctx->rx_in_frame) return;

    if (byte == SDP_ESC_BYTE) {
        ctx->rx_escaped = 1;
        return;
    }
    if (ctx->rx_escaped) {
        byte = (uint8_t)(byte ^ 0x20u);
        ctx->rx_escaped = 0;
    }

    if (ctx->rx_pos < SDP_SLOT_SIZE) {
        ctx->rx_buf[ctx->rx_pos++] = byte;
    } else {
        ctx->diag.rx_overflows++;
        ctx->rx_in_frame = 0;
        ctx->rx_pos      = 0;
    }
}

/* Public entry point: validates ctx, then delegates to the inner. */
sdp_status_t sdp_rx_byte(sdp_ctx_t *ctx, uint8_t byte)
{
    if (!ctx) return SDP_ERR_BAD_ARG;
    rx_byte_inner(ctx, byte);
    return SDP_OK;
}

sdp_status_t sdp_poll(sdp_ctx_t *ctx)
{
    int b;
    if (!ctx || !ctx->hal->read) return SDP_ERR_BAD_ARG;
    while ((b = ctx->hal->read(ctx->hal->user)) >= 0)
        rx_byte_inner(ctx, (uint8_t)b);
    return SDP_OK;
}

/* ── Maintenance ────────────────────────────────────────────────────────── */

sdp_status_t sdp_tick(sdp_ctx_t *ctx)
{
    uint32_t     now;
    sdp_status_t st;

    if (!ctx) return SDP_ERR_BAD_ARG;

    now = ctx->hal->millis(ctx->hal->user);

    if (ctx->sync_hunt_enabled && ctx->state == SDP_STATE_SYNC_ACK) {
        if ((now - ctx->holdoff_start) >= SDP_HOLDOFF_MS) {
            ctx->state      = SDP_STATE_SYNC_HUNT;
            ctx->sync_count = 0;
        }
        return SDP_OK;
    }

    if (ctx->state != SDP_STATE_LINKED) return SDP_OK;

    if (ctx->last_rx_ms != 0 &&
        (now - ctx->last_rx_ms) >= SDP_SILENCE_TIMEOUT_MS) {
        ctx->state      = SDP_STATE_SYNC_HUNT;
        ctx->sync_count = 0;
        if (ctx->on_link_lost)
            ctx->on_link_lost(ctx, ctx->user);
        return SDP_OK;
    }

    if (ctx->ack_pending && (now - ctx->last_ack_ms) >= SDP_ACK_TIMEOUT_MS) {
        st = send_ack(ctx);
        if (st != SDP_OK) return st;
    }

    return SDP_OK;
}

sdp_status_t sdp_reset(sdp_ctx_t *ctx)
{
    sdp_status_t st;

    if (!ctx) return SDP_ERR_BAD_ARG;
    if (ctx->state != SDP_STATE_LINKED) return SDP_ERR_NOT_LINKED;

    st = transmit_frame(ctx, SDP_T_RESET, 0, ctx->tx_seq, ctx->rx_ackseq, NULL, 0);
    ctx->tx_seq      = 0;
    ctx->rx_ackseq   = SDP_SEQ_NONE;
    ctx->peer_ackseq = SDP_SEQ_NONE;
    return st;
}

sdp_status_t sdp_request_stats(sdp_ctx_t *ctx)
{
    if (!ctx) return SDP_ERR_BAD_ARG;
    if (ctx->state != SDP_STATE_LINKED) return SDP_ERR_NOT_LINKED;
    return transmit_frame(ctx, SDP_T_STATS_REQ, 0,
                          ctx->tx_seq, ctx->rx_ackseq, NULL, 0);
}

sdp_status_t sdp_diag_from_stats_rsp(const sdp_frame_t *frame, sdp_diag_t *out)
{
    const uint8_t *p;
    if (!frame || !out) return SDP_ERR_BAD_ARG;
    if (frame->type != SDP_T_STATS_RSP || frame->len < 36) return SDP_ERR_BAD_ARG;
    p = frame->payload;
    #define RD32(i) ((uint32_t)p[(i)*4+0]        | \
                     (uint32_t)p[(i)*4+1] <<  8  | \
                     (uint32_t)p[(i)*4+2] << 16  | \
                     (uint32_t)p[(i)*4+3] << 24)
    out->rx_frames    = RD32(0);
    out->tx_frames    = RD32(1);
    out->crc_errors   = RD32(2);
    out->rx_overflows = RD32(3);
    out->drop_count   = RD32(4);
    out->nak_sent     = RD32(5);
    out->nak_received = RD32(6);
    out->window_full  = RD32(7);
    out->hal_errors   = RD32(8);
    #undef RD32
    return SDP_OK;
}

#endif /* SDP_IMPLEMENTATION */

#endif /* SDP_H */
