#include "sdp.h"
#include <string.h>

/* ── Internal helpers ───────────────────────────────────────────────────── */

static uint16_t crc16_extend(uint16_t crc, const uint8_t *data, uint8_t len);

/*
 * Number of unacknowledged frames in flight.
 * tx_seq = next SEQ to use; peer_ackseq = last ACKSEQ received from peer.
 */
static uint8_t seq_unacked(uint8_t tx_seq, uint8_t peer_ackseq)
{
    if (peer_ackseq == SDP_SEQ_NONE)
        return tx_seq;
    return (uint8_t)((tx_seq + SDP_SEQ_SIZE - peer_ackseq - 1u) % SDP_SEQ_SIZE);
}

static uint16_t crc16(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    uint8_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (j = 0; j < 8; j++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

static uint16_t crc16_extend(uint16_t crc, const uint8_t *data, uint8_t len)
{
    uint8_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (j = 0; j < 8; j++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
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

/* Process a fully assembled, unescaped frame in rx_buf[0..rx_pos-1]. */
static void process_frame(sdp_ctx_t *ctx)
{
    uint8_t      len_field, seq, ackseq, type, flags;
    uint16_t     expected, actual;
    sdp_frame_t  frame;

    if (ctx->rx_pos < 5) return;

    len_field = ctx->rx_buf[2];
    if (len_field > SDP_MAX_PAYLOAD) {
        ctx->diag.crc_errors++;
        return;
    }
    if (ctx->rx_pos != (uint16_t)(len_field + 5u)) {
        ctx->diag.crc_errors++;
        return;
    }

    expected = ((uint16_t)ctx->rx_buf[ctx->rx_pos - 2] << 8)
             | ctx->rx_buf[ctx->rx_pos - 1];
    actual   = crc16(ctx->rx_buf, (uint8_t)(ctx->rx_pos - 2u));

    if (actual != expected) {
        ctx->diag.crc_errors++;
        send_nak(ctx, ctx->rx_buf[0]);
        return;
    }

    seq    = ctx->rx_buf[0] >> 4;
    ackseq = ctx->rx_buf[0] & 0x0Fu;
    type   = ctx->rx_buf[1] >> 2;
    flags  = ctx->rx_buf[1] & 0x03u;

    ctx->peer_ackseq = ackseq;
    ctx->rx_ackseq   = seq;
    ctx->ack_pending = 1;
    ctx->last_rx_ms  = ctx->hal->millis(ctx->hal->user);
    ctx->diag.rx_frames++;

    frame.type   = type;
    frame.flags  = flags;
    frame.seq    = seq;
    frame.ackseq = ackseq;
    frame.len    = len_field;
    if (len_field > 0)
        memcpy(frame.payload, ctx->rx_buf + 3, len_field);

    if (ctx->state == SDP_STATE_SYNC_ACK) {
        ctx->state       = SDP_STATE_LINKED;
        ctx->tx_seq      = 0;
        ctx->peer_ackseq = SDP_SEQ_NONE;
    }

    switch (type) {
    case SDP_T_ACK:
        ctx->last_ack_ms = ctx->hal->millis(ctx->hal->user);
        ctx->ack_pending = 0;
        return;

    case SDP_T_RESET:
        ctx->tx_seq      = 0;
        ctx->rx_ackseq   = SDP_SEQ_NONE;
        ctx->peer_ackseq = SDP_SEQ_NONE;
        transmit_frame(ctx, SDP_T_RESET, 0, 0, SDP_SEQ_NONE, NULL, 0);
        return;

    case SDP_T_NAK:
        ctx->diag.nak_received++;
        if (frame.len >= 1) {
            uint8_t bad_seq = frame.payload[0] >> 4;
            sdp_tx_slot_t *slot = &ctx->tx_ring[bad_seq % SDP_WINDOW];
            if (slot->seq == bad_seq)
                transmit_frame(ctx, slot->type,
                               (uint8_t)(slot->flags | SDP_FLAG_RTX),
                               slot->seq, ctx->rx_ackseq,
                               slot->payload, slot->len);
        }
        ctx->on_frame(ctx, &frame, ctx->user);
        return;

    case SDP_T_STATS_REQ: {
        /* Pack local diagnostics into a T_STATS_RSP payload (little-endian). */
        uint8_t rsp[36];
        const uint32_t counts[9] = {
            ctx->diag.rx_frames,   ctx->diag.tx_frames,
            ctx->diag.crc_errors,  ctx->diag.rx_overflows,
            ctx->diag.drop_count,  ctx->diag.nak_sent,
            ctx->diag.nak_received,ctx->diag.window_full,
            ctx->diag.hal_errors
        };
        uint8_t i;
        for (i = 0; i < 9; i++) {
            rsp[i*4+0] = (uint8_t)(counts[i]       & 0xFFu);
            rsp[i*4+1] = (uint8_t)((counts[i] >> 8)  & 0xFFu);
            rsp[i*4+2] = (uint8_t)((counts[i] >> 16) & 0xFFu);
            rsp[i*4+3] = (uint8_t)((counts[i] >> 24) & 0xFFu);
        }
        transmit_frame(ctx, SDP_T_STATS_RSP, 0,
                       ctx->tx_seq, ctx->rx_ackseq, rsp, 36);
        return;
    }

    case SDP_T_STATS_RSP:
        /* Delivered to on_frame; host unpacks with sdp_diag_from_stats_rsp. */
        ctx->on_frame(ctx, &frame, ctx->user);
        return;

    case SDP_T_UNSUPPORTED:
        ctx->on_frame(ctx, &frame, ctx->user);
        return;

    default:
        ctx->on_frame(ctx, &frame, ctx->user);
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
    uint8_t      payload[SDP_MAX_PAYLOAD];
    uint8_t      app_len = 0;
    sdp_status_t st;

    if (!ctx) return SDP_ERR_BAD_ARG;

    payload[0] = SDP_PROTOCOL_VERSION;
    payload[1] = SDP_WINDOW;
    payload[2] = SDP_MAX_PAYLOAD;

    st = ctx->on_build_announce(payload + 3, &app_len, ctx->user);
    if (st != SDP_OK) return st;

    if ((uint16_t)app_len + 3u > SDP_MAX_PAYLOAD) return SDP_ERR_PAYLOAD_TOO_LARGE;

    return transmit_frame(ctx, SDP_T_ANNOUNCE, 0, 0, SDP_SEQ_NONE,
                          payload, (uint8_t)(app_len + 3u));
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
            sdp_rx_byte(ctx, (uint8_t)b);

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

    /* Store in TX ring for retransmission on T_NAK. */
    slot        = &ctx->tx_ring[ctx->tx_seq % SDP_WINDOW];
    slot->seq   = ctx->tx_seq;
    slot->type  = type;
    slot->flags = flags;
    slot->len   = len;
    if (len > 0) memcpy(slot->payload, payload, len);

    st = transmit_frame(ctx, type, flags, ctx->tx_seq, ctx->rx_ackseq, payload, len);
    if (st != SDP_OK) return st;

    ctx->tx_seq      = (ctx->tx_seq >= SDP_SEQ_MAX) ? 0u : (uint8_t)(ctx->tx_seq + 1u);
    ctx->last_ack_ms = ctx->hal->millis(ctx->hal->user);
    ctx->ack_pending = 0;
    return SDP_OK;
}

/* ── Receiving ──────────────────────────────────────────────────────────── */

sdp_status_t sdp_rx_byte(sdp_ctx_t *ctx, uint8_t byte)
{
    if (!ctx) return SDP_ERR_BAD_ARG;

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
                return SDP_OK;
            } else if (byte != SDP_FLAG_BYTE) {
                ctx->sync_count = 0;
                return SDP_OK;
            }
        } else if (ctx->state == SDP_STATE_SYNC_ACK) {
            if (byte == SDP_SYNC_BYTE) return SDP_OK;
        }
    }

    if (byte == SDP_FLAG_BYTE) {
        if (ctx->rx_in_frame && ctx->rx_pos >= 5)
            process_frame(ctx);
        ctx->rx_in_frame = 1;
        ctx->rx_pos      = 0;
        ctx->rx_escaped  = 0;
        return SDP_OK;
    }

    if (!ctx->rx_in_frame) return SDP_OK;

    if (byte == SDP_ESC_BYTE) {
        ctx->rx_escaped = 1;
        return SDP_OK;
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
    return SDP_OK;
}

sdp_status_t sdp_poll(sdp_ctx_t *ctx)
{
    int b;
    if (!ctx || !ctx->hal->read) return SDP_ERR_BAD_ARG;
    while ((b = ctx->hal->read(ctx->hal->user)) >= 0)
        sdp_rx_byte(ctx, (uint8_t)b);
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
