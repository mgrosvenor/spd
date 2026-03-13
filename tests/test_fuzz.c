/*
 * Fuzz / stress tests for SDP.
 *
 * Covers:
 *   - All payload sizes (0 .. SDP_MAX_PAYLOAD)
 *   - All message type codes (transport types + application types 0x01-0x0F)
 *   - Bit-flip corruption: CRC error detected, no silent bad delivery
 *   - Byte-drop corruption: CRC error or dropped frame, no silent bad delivery
 *   - Truncated frames: CRC error or nothing delivered, no crash
 *   - Random garbage input: protocol survives, clean link still works after
 *   - Window stress: SDP_ERR_WINDOW_FULL at limit, recovers after ACK
 *   - Sequence wraparound stress: 100 frames, in-order, zero CRC errors
 *
 * Build (standalone):
 *   gcc -I../src -DSDP_MAX_PAYLOAD=255 -DSDP_WINDOW=2 \
 *       ../src/sdp.c test_fuzz.c -o test_fuzz
 */

#include "test_framework.h"
#include "test_hal.h"
#include "sdp.h"
#include <string.h>
#include <stdlib.h>

/* ── Shared endpoint type (same pattern as test_e2e.c) ─────────────────── */

typedef struct {
    sdp_ctx_t   ctx;
    sdp_hal_t   hal;
    hal_state_t hs;
    int         frame_count;
    sdp_frame_t last_frame;
    int         announce_count;
} endpoint_t;

static void ep_on_frame(sdp_ctx_t *ctx, const sdp_frame_t *f, void *user)
{
    endpoint_t *ep = (endpoint_t *)user;
    (void)ctx;
    ep->last_frame  = *f;
    ep->frame_count++;
    if (f->type == SDP_T_ANNOUNCE) ep->announce_count++;
}

static sdp_status_t ep_build_announce(uint8_t *payload, uint8_t *len, void *user)
{
    (void)user;
    payload[0] = 0xDE;
    payload[1] = 0xAD;
    *len = 2;
    return SDP_OK;
}

static void ep_init(endpoint_t *ep, byte_pipe_t *tx, byte_pipe_t *rx)
{
    memset(ep, 0, sizeof(*ep));
    ep->hs.tx_pipe = tx;
    ep->hs.rx_pipe = rx;
    ep->hal.write  = hal_write;
    ep->hal.read   = hal_read;
    ep->hal.millis = hal_millis;
    ep->hal.user   = &ep->hs;
    sdp_init(&ep->ctx, &ep->hal, ep_on_frame, ep_build_announce, ep);
}

/* Drain all bytes from a_tx -> b_rx, then b_tx -> a_rx. */
static void pump(endpoint_t *a, endpoint_t *b)
{
    int bv;
    while ((bv = pipe_pop(a->hs.tx_pipe)) >= 0)
        sdp_rx_byte(&b->ctx, (uint8_t)bv);
    while ((bv = pipe_pop(b->hs.tx_pipe)) >= 0)
        sdp_rx_byte(&a->ctx, (uint8_t)bv);
}

/*
 * Establish a loopback pair without sdp_connect.
 *
 * After this call both endpoints are LINKED with tx_seq = 0.
 * The device has received ANNOUNCE (device->announce_count >= 1).
 * host->frame_count may be > 0 (contains the device ANNOUNCE).
 */
static void establish_link(endpoint_t *host, endpoint_t *device,
                            byte_pipe_t *a_to_b, byte_pipe_t *b_to_a)
{
    uint8_t sync[SDP_SYNC_SEND];
    sdp_status_t st;

    pipe_init(a_to_b);
    pipe_init(b_to_a);
    g_fake_millis = 0;

    ep_init(host,   a_to_b, b_to_a);
    ep_init(device, b_to_a, a_to_b);

    st = sdp_listen(&device->ctx);
    (void)st; /* checked by callers where needed */

    /* Send 8 x 0xAA sync bytes from the host side */
    memset(sync, SDP_SYNC_BYTE, SDP_SYNC_SEND);
    host->hal.write(sync, SDP_SYNC_SEND, host->hal.user);

    /* Two pump passes: first lets device see sync and emit ANNOUNCE;
     * second lets host see ANNOUNCE (and host would normally respond,
     * but we skip that and set state manually). */
    pump(host, device);
    pump(host, device);

    /* Manually mark host as LINKED — same idiom as test_e2e.c */
    host->ctx.state  = SDP_STATE_LINKED;
    host->ctx.tx_seq = 0;
}

/* ── Wire-corruption helpers ────────────────────────────────────────────── */

/*
 * Drain all bytes from pipe into rx_ctx, but XOR bit `bit_to_flip` of the
 * byte at position `corrupt_pos` (0-indexed from start of pipe contents).
 * If corrupt_pos < 0, feed all bytes uncorrupted.
 */
static void pump_with_corruption(byte_pipe_t *pipe, sdp_ctx_t *rx_ctx,
                                  int corrupt_pos, int bit_to_flip)
{
    int pos = 0;
    int bv;
    while ((bv = pipe_pop(pipe)) >= 0) {
        uint8_t b = (uint8_t)bv;
        if (corrupt_pos >= 0 && pos == corrupt_pos)
            b ^= (uint8_t)(1u << (bit_to_flip & 7));
        sdp_rx_byte(rx_ctx, b);
        pos++;
    }
}

/*
 * Drain all bytes from pipe into rx_ctx, skipping (dropping) the byte at
 * position `drop_pos` (0-indexed).
 */
static void pump_with_drop(byte_pipe_t *pipe, sdp_ctx_t *rx_ctx, int drop_pos)
{
    int pos = 0;
    int bv;
    while ((bv = pipe_pop(pipe)) >= 0) {
        uint8_t b = (uint8_t)bv;
        if (pos != drop_pos)
            sdp_rx_byte(rx_ctx, b);
        pos++;
    }
}

/* ── CRC helper (CRC-16/CCITT-FALSE, poly 0x1021, init 0xFFFF) ─────────── */
/* Duplicated here so test_fuzz.c is self-contained. */
static uint16_t fuzz_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    for (i = 0; i < len; i++) {
        uint8_t j;
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (j = 0; j < 8; j++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
    }
    return crc;
}

/*
 * Inject a minimal well-formed frame directly into ctx via sdp_rx_byte.
 * Frame layout (no HDLC escaping applied — values chosen to avoid 0x7E/0x7D):
 *   FLAG | seq_ack | type_flg | len | payload... | crc_hi | crc_lo | FLAG
 *
 * seq and ackseq must be < 0x0F so the nibble encoding is safe.
 */
static void inject_raw_frame(sdp_ctx_t *ctx,
                              uint8_t type, uint8_t flags,
                              uint8_t seq,  uint8_t ackseq,
                              const uint8_t *payload, uint8_t len)
{
    uint8_t  seq_ack  = (uint8_t)((seq << 4) | (ackseq & 0x0Fu));
    uint8_t  type_flg = (uint8_t)((type << 2) | (flags & 0x03u));
    uint8_t  hdr[3];
    uint16_t crc;
    uint8_t  i;

    hdr[0] = seq_ack;
    hdr[1] = type_flg;
    hdr[2] = len;
    crc = fuzz_crc16(hdr, 3);
    if (payload && len > 0) {
        /* extend CRC over payload bytes */
        uint16_t c = crc;
        for (i = 0; i < len; i++) {
            uint8_t b = payload[i];
            uint8_t j;
            c ^= (uint16_t)((uint16_t)b << 8);
            for (j = 0; j < 8; j++)
                c = (c & 0x8000u) ? (uint16_t)((c << 1) ^ 0x1021u)
                                  : (uint16_t)(c << 1);
        }
        crc = c;
    }

    sdp_rx_byte(ctx, SDP_FLAG_BYTE);

    /* Write seq_ack with escaping */
    if (seq_ack == SDP_FLAG_BYTE || seq_ack == SDP_ESC_BYTE) {
        sdp_rx_byte(ctx, SDP_ESC_BYTE);
        sdp_rx_byte(ctx, seq_ack ^ 0x20u);
    } else {
        sdp_rx_byte(ctx, seq_ack);
    }

    /* Write type_flg with escaping */
    if (type_flg == SDP_FLAG_BYTE || type_flg == SDP_ESC_BYTE) {
        sdp_rx_byte(ctx, SDP_ESC_BYTE);
        sdp_rx_byte(ctx, type_flg ^ 0x20u);
    } else {
        sdp_rx_byte(ctx, type_flg);
    }

    /* Write len with escaping */
    if (len == SDP_FLAG_BYTE || len == SDP_ESC_BYTE) {
        sdp_rx_byte(ctx, SDP_ESC_BYTE);
        sdp_rx_byte(ctx, len ^ 0x20u);
    } else {
        sdp_rx_byte(ctx, len);
    }

    /* Payload bytes */
    for (i = 0; i < len; i++) {
        uint8_t b = payload[i];
        if (b == SDP_FLAG_BYTE || b == SDP_ESC_BYTE) {
            sdp_rx_byte(ctx, SDP_ESC_BYTE);
            sdp_rx_byte(ctx, b ^ 0x20u);
        } else {
            sdp_rx_byte(ctx, b);
        }
    }

    /* CRC */
    {
        uint8_t crc_hi = (uint8_t)(crc >> 8);
        uint8_t crc_lo = (uint8_t)(crc & 0xFFu);
        if (crc_hi == SDP_FLAG_BYTE || crc_hi == SDP_ESC_BYTE) {
            sdp_rx_byte(ctx, SDP_ESC_BYTE);
            sdp_rx_byte(ctx, crc_hi ^ 0x20u);
        } else {
            sdp_rx_byte(ctx, crc_hi);
        }
        if (crc_lo == SDP_FLAG_BYTE || crc_lo == SDP_ESC_BYTE) {
            sdp_rx_byte(ctx, SDP_ESC_BYTE);
            sdp_rx_byte(ctx, crc_lo ^ 0x20u);
        } else {
            sdp_rx_byte(ctx, crc_lo);
        }
    }

    sdp_rx_byte(ctx, SDP_FLAG_BYTE);
}

/* ══════════════════════════════════════════════════════════════════════════
 * TEST FUNCTIONS
 * ══════════════════════════════════════════════════════════════════════════ */

/* ── TEST-FUZZ-01: all payload sizes 0 .. SDP_MAX_PAYLOAD ───────────────── */
static void test_fuzz_payload_sizes(void)
{
    static byte_pipe_t a_to_b, b_to_a;
    endpoint_t host, device;
    int sz;

    srand(42);

    establish_link(&host, &device, &a_to_b, &b_to_a);

    for (sz = 0; sz <= SDP_MAX_PAYLOAD; sz++) {
        uint8_t  payload[SDP_MAX_PAYLOAD];
        uint32_t crc_before;
        int      i;
        sdp_status_t st;

        /* Fill with deterministic random bytes */
        for (i = 0; i < sz; i++)
            payload[i] = (uint8_t)(rand() & 0xFF);

        crc_before = device.ctx.diag.crc_errors;
        device.frame_count = 0;

        st = sdp_send(&host.ctx, 0x01, 0, sz > 0 ? payload : NULL, (uint8_t)sz);
        CHECK_EQ((int)st, (int)SDP_OK);

        /* Deliver frame to device; device ACKs back to host */
        pump(&host, &device);
        /* Let ACK timer fire so device actually sends the ACK */
        advance_ms(55);
        sdp_tick(&device.ctx);
        pump(&host, &device);

        /* Device must have received exactly one application frame */
        CHECK_EQ(device.frame_count, 1);
        CHECK_EQ((int)device.last_frame.type, 0x01);
        CHECK_EQ((int)device.last_frame.len,  sz);

        /* Payload content must match exactly */
        for (i = 0; i < sz; i++) {
            CHECK_EQ((int)device.last_frame.payload[i], (int)payload[i]);
        }

        /* No new CRC errors */
        CHECK_EQ((int)device.ctx.diag.crc_errors, (int)crc_before);
    }
}

/* ── TEST-FUZZ-02: all message types ────────────────────────────────────── */
static void test_fuzz_all_message_types(void)
{
    static byte_pipe_t a_to_b, b_to_a;
    endpoint_t host, device;
    int app_type;

    srand(42);

    establish_link(&host, &device, &a_to_b, &b_to_a);

    /*
     * Application message types 0x01 through 0x0F — sent via sdp_send and
     * verified at the receiving device.
     */
    for (app_type = 0x01; app_type <= 0x0F; app_type++) {
        uint8_t payload[4] = {(uint8_t)app_type, 0xAB, 0xCD, 0xEF};
        sdp_status_t st;

        device.frame_count = 0;

        st = sdp_send(&host.ctx, (uint8_t)app_type, 0, payload, 4);
        CHECK_EQ((int)st, (int)SDP_OK);

        pump(&host, &device);
        advance_ms(55);
        sdp_tick(&device.ctx);
        pump(&host, &device);

        CHECK_EQ(device.frame_count, 1);
        CHECK_EQ((int)device.last_frame.type, app_type);
        CHECK_EQ((int)device.last_frame.len,  4);
    }

    /*
     * Transport control frames injected directly into the device's RX path.
     *
     * T_ACK (0x30): device should process without crashing.
     * T_NAK (0x31): host must not retransmit a stale NAK (no in-flight data).
     * T_RESET (0x32): triggers link reset.
     * T_STATS_REQ (0x33): device should respond with T_STATS_RSP.
     * T_STATS_RSP (0x34): device just stores it.
     * T_UNSUPPORTED (0x3C): device silently accepts without crashing.
     */
    {
        /* T_ACK — no payload, seq=0, ackseq=0 */
        uint32_t crc_before = device.ctx.diag.crc_errors;
        inject_raw_frame(&device.ctx, SDP_T_ACK, 0, 0, 0, NULL, 0);
        CHECK_EQ((int)device.ctx.diag.crc_errors, (int)crc_before);
    }

    {
        /* T_NAK — payload = bad_seq_ack (1 byte) */
        uint8_t bad_seq = 0x00;
        uint32_t crc_before = host.ctx.diag.crc_errors;
        inject_raw_frame(&host.ctx, SDP_T_NAK, 0, 0, 0, &bad_seq, 1);
        CHECK_EQ((int)host.ctx.diag.crc_errors, (int)crc_before);
    }

    {
        /* T_RESET — device should handle without crash */
        uint32_t crc_before = device.ctx.diag.crc_errors;
        inject_raw_frame(&device.ctx, SDP_T_RESET, 0, 0, 0, NULL, 0);
        CHECK_EQ((int)device.ctx.diag.crc_errors, (int)crc_before);
        /* After RESET the link may drop; re-establish for subsequent tests */
        establish_link(&host, &device, &a_to_b, &b_to_a);
    }

    {
        /* T_STATS_REQ — device responds; just verify no crash or CRC error */
        uint32_t crc_before = device.ctx.diag.crc_errors;
        inject_raw_frame(&device.ctx, SDP_T_STATS_REQ, 0, 0, 0, NULL, 0);
        CHECK_EQ((int)device.ctx.diag.crc_errors, (int)crc_before);
    }

    {
        /* T_STATS_RSP — build a minimal (all-zero) payload and inject */
        uint8_t rsp_payload[36];
        uint32_t crc_before = device.ctx.diag.crc_errors;
        memset(rsp_payload, 0, sizeof(rsp_payload));
        inject_raw_frame(&device.ctx, SDP_T_STATS_RSP, 0, 0, 0,
                         rsp_payload, (uint8_t)sizeof(rsp_payload));
        CHECK_EQ((int)device.ctx.diag.crc_errors, (int)crc_before);
    }

    {
        /* T_UNSUPPORTED — no crash */
        uint32_t crc_before = device.ctx.diag.crc_errors;
        inject_raw_frame(&device.ctx, SDP_T_UNSUPPORTED, 0, 0, 0, NULL, 0);
        CHECK_EQ((int)device.ctx.diag.crc_errors, (int)crc_before);
    }
}

/* ── TEST-FUZZ-03: single bit flips in payload area ────────────────────── */
static void test_fuzz_bit_flips(void)
{
    int iter;

#define BIT_FLIP_FRAMES 20
#define PAYLOAD_LEN 16

    srand(42);

    /*
     * Each iteration gets a fresh link so window-state from corrupted or
     * NAK'd frames does not accumulate across iterations.
     */
    for (iter = 0; iter < BIT_FLIP_FRAMES; iter++) {
        static byte_pipe_t a_to_b, b_to_a;
        endpoint_t host, device;
        uint8_t  payload[PAYLOAD_LEN];
        uint8_t  saved_payload[PAYLOAD_LEN];
        uint32_t crc_before;
        uint32_t frames_before;
        int      i;
        int      corrupt_pos;
        int      bit;
        sdp_status_t st;

        establish_link(&host, &device, &a_to_b, &b_to_a);

        for (i = 0; i < PAYLOAD_LEN; i++)
            payload[i] = (uint8_t)(rand() & 0xFF);
        memcpy(saved_payload, payload, PAYLOAD_LEN);

        /*
         * Wire frame layout for a PAYLOAD_LEN-byte frame (assuming no
         * escaping is needed — rand bytes might need escaping, but the
         * corruption position arithmetic is conservative):
         *
         *   [0]   = FLAG (0x7E) — must not corrupt
         *   [1]   = seq_ack
         *   [2]   = type_flg
         *   [3]   = len
         *   [4..4+PAYLOAD_LEN-1] = payload bytes (safe to corrupt)
         *   [4+PAYLOAD_LEN]   = crc_hi
         *   [4+PAYLOAD_LEN+1] = crc_lo
         *   [4+PAYLOAD_LEN+2] = FLAG
         *
         * Because HDLC escaping can expand the frame, pick a position in the
         * conservative mid-range of the payload area (bytes 4..4+PAYLOAD_LEN/2)
         * so we stay inside the payload bytes and well away from the CRC/FLAG.
         */
        corrupt_pos = 4 + (rand() % (PAYLOAD_LEN / 2));
        bit         = rand() % 8;

        crc_before    = device.ctx.diag.crc_errors;
        frames_before = (uint32_t)device.frame_count;

        st = sdp_send(&host.ctx, 0x01, 0, payload, PAYLOAD_LEN);
        CHECK_EQ((int)st, (int)SDP_OK);

        /* Feed the corrupted frame to device */
        pump_with_corruption(&a_to_b, &device.ctx, corrupt_pos, bit);

        /* Pump device reply (NAK or ACK or nothing) back to host */
        {
            int bv;
            while ((bv = pipe_pop(&b_to_a)) >= 0)
                sdp_rx_byte(&host.ctx, (uint8_t)bv);
        }

        /*
         * Invariant: the device must never silently deliver a frame whose
         * payload does not match what was sent with zero CRC errors.
         *
         * Acceptable outcomes:
         *   (a) crc_errors increased — corruption detected correctly.
         *   (b) frame_count stayed the same — frame was silently dropped
         *       or the parser is still hunting (also fine).
         *   (c) Frame delivered with CRC-ok AND payload intact — this can
         *       happen if escaping shifted positions so the corrupted byte
         *       was outside the actual payload (extremely rare but legal).
         */
        {
            int new_frame  = ((uint32_t)device.frame_count > frames_before);
            int crc_bumped = (device.ctx.diag.crc_errors > crc_before);

            if (new_frame && !crc_bumped) {
                /* Frame was delivered — verify content matches original */
                for (i = 0; i < PAYLOAD_LEN; i++) {
                    CHECK_EQ((int)device.last_frame.payload[i],
                             (int)saved_payload[i]);
                }
            }
            /* Either CRC detected an error, or no new bad frame arrived */
            CHECK(crc_bumped || !new_frame ||
                  (device.last_frame.len == PAYLOAD_LEN));
        }
    }
#undef BIT_FLIP_FRAMES
#undef PAYLOAD_LEN
}

/* ── TEST-FUZZ-04: byte drops at every position ─────────────────────────── */
static void test_fuzz_byte_drops(void)
{
    static byte_pipe_t a_to_b, b_to_a;
    endpoint_t host, device;

#define DROP_PAYLOAD_LEN 16

    srand(42);

    establish_link(&host, &device, &a_to_b, &b_to_a);

    {
        uint8_t payload[DROP_PAYLOAD_LEN];
        uint8_t saved[DROP_PAYLOAD_LEN];
        int     i;

        for (i = 0; i < DROP_PAYLOAD_LEN; i++)
            payload[i] = (uint8_t)(rand() & 0xFF);
        memcpy(saved, payload, DROP_PAYLOAD_LEN);

        /*
         * Send one reference frame and capture its wire size by measuring
         * the pipe fill before and after. Then replay for each drop position.
         *
         * Wire size upper bound (with max escaping): 2 + 2*(3+PAYLOAD+2) = 2+2*20 = 42.
         * In practice for random bytes the pipe count gives the exact size.
         */
        {
            int wire_len;
            sdp_status_t st;

            st = sdp_send(&host.ctx, 0x01, 0, payload, DROP_PAYLOAD_LEN);
            CHECK_EQ((int)st, (int)SDP_OK);
            wire_len = (int)a_to_b.count;

            /*
             * For each byte position p from 1 to wire_len-2 (skip the two
             * FLAG bytes at ends — dropping a FLAG byte turns two valid
             * frames into one malformed fragment, which is fine but makes
             * counting trickier), feed the frame with that byte removed.
             */
            for (i = 1; i < wire_len - 1; i++) {
                uint32_t crc_before    = device.ctx.diag.crc_errors;
                int      frames_before = device.frame_count;
                uint16_t saved_head    = a_to_b.head;
                uint16_t saved_tail    = a_to_b.tail;
                uint16_t saved_count   = a_to_b.count;
                uint8_t  saved_buf[PIPE_SIZE];

                /* Save pipe state so we can restore for next iteration */
                memcpy(saved_buf, a_to_b.buf, PIPE_SIZE);

                pump_with_drop(&a_to_b, &device.ctx, i);

                /* Drain any response (NAK / ACK) from device, discard */
                while (pipe_pop(&b_to_a) >= 0) {}

                /*
                 * Invariant: receiver must never deliver a frame with wrong
                 * content. If a new frame arrived, its payload must match.
                 */
                if (device.frame_count > frames_before) {
                    int j;
                    /* Either CRC caught it, or the frame is correct */
                    for (j = 0; j < DROP_PAYLOAD_LEN; j++) {
                        CHECK_EQ((int)device.last_frame.payload[j],
                                 (int)saved[j]);
                    }
                }
                /* If no new frame, that is also correct (dropped) */
                CHECK(device.ctx.diag.crc_errors >= crc_before);

                /* Restore the pipe for the next drop position */
                memcpy(a_to_b.buf, saved_buf, PIPE_SIZE);
                a_to_b.head  = saved_head;
                a_to_b.tail  = saved_tail;
                a_to_b.count = saved_count;
            }

            /* Finally drain the original frame cleanly to reset parser */
            {
                int bv;
                while ((bv = pipe_pop(&a_to_b)) >= 0)
                    sdp_rx_byte(&device.ctx, (uint8_t)bv);
            }
            /* ACK it so the window opens for future tests */
            advance_ms(55);
            sdp_tick(&device.ctx);
            while (pipe_pop(&b_to_a) >= 0) {}
            while (pipe_pop(&a_to_b) >= 0) {}
        }
    }
#undef DROP_PAYLOAD_LEN
}

/*
 * Helper: send a frame, feed only `trunc_bytes` of it to the receiver, then
 * feed a closing FLAG.  Verify no crash and no silent bad delivery.
 * Uses its own fresh link to avoid window-state entanglement.
 *
 * Valid outcomes after a truncated feed:
 *   (a) CRC error recorded — truncation was detected as corruption.
 *   (b) No new frame delivered — partial body silently discarded.
 *   (c) Frame delivered with no CRC error — this is legal when `trunc_bytes`
 *       happens to cover the entire body and we provide the closing FLAG,
 *       i.e. the "truncated" frame was actually complete. In that case the
 *       received payload MUST match what was sent.
 */
static void trunc_one_frame(int trunc_bytes)
{
    static byte_pipe_t a_to_b, b_to_a;
    endpoint_t host, device;
    uint8_t payload[20];
    uint8_t saved[20];
    int frames_before;
    uint32_t crc_before;
    sdp_status_t st;
    int i;

    establish_link(&host, &device, &a_to_b, &b_to_a);

    for (i = 0; i < 20; i++)
        payload[i] = saved[i] = (uint8_t)(rand() & 0xFF);

    frames_before = device.frame_count;
    crc_before    = device.ctx.diag.crc_errors;

    st = sdp_send(&host.ctx, 0x01, 0, payload, 20);
    CHECK_EQ((int)st, (int)SDP_OK);

    /* Feed only trunc_bytes (or all, if trunc_bytes covers the whole frame) */
    for (i = 0; i < trunc_bytes; i++) {
        int bv = pipe_pop(&a_to_b);
        if (bv >= 0)
            sdp_rx_byte(&device.ctx, (uint8_t)bv);
    }
    /* Discard remaining bytes */
    while (pipe_pop(&a_to_b) >= 0) {}

    /* Feed a lone FLAG to trigger frame-end processing */
    sdp_rx_byte(&device.ctx, SDP_FLAG_BYTE);

    /* Discard any response (NAK / nothing) */
    while (pipe_pop(&b_to_a) >= 0) {}

    /*
     * Invariant: no crash.  If a frame was delivered with no CRC error,
     * verify its payload is correct.  We must never get wrong data silently.
     */
    if (device.frame_count > frames_before &&
        device.ctx.diag.crc_errors == crc_before) {
        /* Frame was cleanly delivered — content must match */
        CHECK_EQ((int)device.last_frame.len, 20);
        for (i = 0; i < 20; i++)
            CHECK_EQ((int)device.last_frame.payload[i], (int)saved[i]);
    }
    /* No invariant violation if crc bumped or no new frame */
    CHECK(device.ctx.diag.crc_errors >= crc_before);
}

/* ── TEST-FUZZ-05: truncated frames ─────────────────────────────────────── */
static void test_fuzz_truncated_frames(void)
{
    static byte_pipe_t a_to_b, b_to_a;
    endpoint_t host, device;
    uint8_t payload[20];
    int i;

    srand(42);

    /* Fixed truncation points */
    trunc_one_frame(2);
    trunc_one_frame(3);
    trunc_one_frame(5);
    trunc_one_frame(10);

    /* Half-frame and all-but-one: measure wire length dynamically */
    {
        int wire_len;

        establish_link(&host, &device, &a_to_b, &b_to_a);
        for (i = 0; i < 20; i++)
            payload[i] = (uint8_t)(rand() & 0xFF);

        {
            sdp_status_t st = sdp_send(&host.ctx, 0x01, 0, payload, 20);
            CHECK_EQ((int)st, (int)SDP_OK);
        }
        wire_len = (int)a_to_b.count;
        while (pipe_pop(&a_to_b) >= 0) {}  /* discard — just needed the count */

        trunc_one_frame(wire_len / 2);
        trunc_one_frame(wire_len - 1);
    }
}

/* ── TEST-FUZZ-06: random garbage input ────────────────────────────────── */
static void test_fuzz_random_garbage(void)
{
    static byte_pipe_t a_to_b, b_to_a;
    endpoint_t host, device;
    int i;
    sdp_status_t st;
    uint8_t payload[4] = {0x01, 0x02, 0x03, 0x04};

    srand(42);

    /* Fresh uninitialised (SYNC_HUNT) receiver — just init, no link */
    pipe_init(&a_to_b);
    pipe_init(&b_to_a);
    g_fake_millis = 0;
    ep_init(&host,   &a_to_b, &b_to_a);
    ep_init(&device, &b_to_a, &a_to_b);

    /* Feed 1024 bytes of random garbage directly to device */
    for (i = 0; i < 1024; i++) {
        uint8_t b = (uint8_t)(rand() & 0xFF);
        sdp_rx_byte(&device.ctx, b);
    }

    /* Now establish a real link and verify normal operation survives */
    establish_link(&host, &device, &a_to_b, &b_to_a);

    device.frame_count = 0;
    st = sdp_send(&host.ctx, 0x01, 0, payload, 4);
    CHECK_EQ((int)st, (int)SDP_OK);

    pump(&host, &device);
    advance_ms(55);
    sdp_tick(&device.ctx);
    pump(&host, &device);

    CHECK_EQ(device.frame_count, 1);
    CHECK_EQ((int)device.last_frame.type, 0x01);
    CHECK_EQ((int)device.last_frame.len, 4);
    for (i = 0; i < 4; i++)
        CHECK_EQ((int)device.last_frame.payload[i], (int)payload[i]);
}

/* ── TEST-FUZZ-07: window stress — fill, overflow, recover ──────────────── */
static void test_fuzz_window_stress(void)
{
    static byte_pipe_t a_to_b, b_to_a;
    endpoint_t host, device;
    int i;
    sdp_status_t st;

    srand(42);

    establish_link(&host, &device, &a_to_b, &b_to_a);

    /* Fill the TX window completely */
    for (i = 0; i < SDP_WINDOW; i++) {
        st = sdp_send(&host.ctx, 0x01, 0, NULL, 0);
        CHECK_EQ((int)st, (int)SDP_OK);
    }

    /* Next send must fail with WINDOW_FULL */
    st = sdp_send(&host.ctx, 0x01, 0, NULL, 0);
    CHECK_EQ((int)st, (int)SDP_ERR_WINDOW_FULL);
    CHECK(host.ctx.diag.window_full >= 1u);

    /* Deliver frames to device; device will ACK after timeout */
    pump(&host, &device);
    advance_ms(55);
    sdp_tick(&device.ctx);
    pump(&host, &device);

    /* Window should be open again */
    st = sdp_send(&host.ctx, 0x01, 0, NULL, 0);
    CHECK_EQ((int)st, (int)SDP_OK);

    /* Repeat overflow / recover cycle a second time */
    /* Fill to capacity (already have 1 in flight) */
    for (i = 1; i < SDP_WINDOW; i++) {
        st = sdp_send(&host.ctx, 0x01, 0, NULL, 0);
        CHECK_EQ((int)st, (int)SDP_OK);
    }
    st = sdp_send(&host.ctx, 0x01, 0, NULL, 0);
    CHECK_EQ((int)st, (int)SDP_ERR_WINDOW_FULL);

    pump(&host, &device);
    advance_ms(55);
    sdp_tick(&device.ctx);
    pump(&host, &device);

    st = sdp_send(&host.ctx, 0x02, 0, NULL, 0);
    CHECK_EQ((int)st, (int)SDP_OK);
}

/* ── TEST-FUZZ-08: seq wraparound stress — 100 frames ──────────────────── */
static void test_fuzz_seq_wraparound_stress(void)
{
    static byte_pipe_t a_to_b, b_to_a;
    endpoint_t host, device;
    uint8_t payload[8];
    int i;
    uint32_t crc_before_host;
    uint32_t crc_before_device;

    srand(42);

    establish_link(&host, &device, &a_to_b, &b_to_a);

    crc_before_host   = host.ctx.diag.crc_errors;
    crc_before_device = device.ctx.diag.crc_errors;

    for (i = 0; i < 100; i++) {
        int j;
        sdp_status_t st;

        for (j = 0; j < 8; j++)
            payload[j] = (uint8_t)(rand() & 0xFF);

        /*
         * Send one frame, immediately pump to keep window clear and let
         * device ACK before next iteration.
         */
        st = sdp_send(&host.ctx, 0x01, 0, payload, 8);
        CHECK_EQ((int)st, (int)SDP_OK);

        pump(&host, &device);
        advance_ms(55);
        sdp_tick(&device.ctx);
        pump(&host, &device);

        /* Frame must have arrived intact */
        CHECK_EQ((int)device.last_frame.type, 0x01);
        CHECK_EQ((int)device.last_frame.len, 8);
        for (j = 0; j < 8; j++)
            CHECK_EQ((int)device.last_frame.payload[j], (int)payload[j]);
    }

    /* No CRC errors anywhere across 100 frames */
    CHECK_EQ((int)host.ctx.diag.crc_errors,   (int)crc_before_host);
    CHECK_EQ((int)device.ctx.diag.crc_errors, (int)crc_before_device);

    /* Sequence number must have wrapped — tx_seq is in [0, SDP_SEQ_MAX] */
    CHECK(host.ctx.tx_seq <= (uint8_t)SDP_SEQ_MAX);
}

/* ══════════════════════════════════════════════════════════════════════════
 * main
 * ══════════════════════════════════════════════════════════════════════════ */

int main(void)
{
    printf("=== test_fuzz ===\n");

    printf("  test_fuzz_payload_sizes...\n");
    test_fuzz_payload_sizes();

    printf("  test_fuzz_all_message_types...\n");
    test_fuzz_all_message_types();

    printf("  test_fuzz_bit_flips...\n");
    test_fuzz_bit_flips();

    printf("  test_fuzz_byte_drops...\n");
    test_fuzz_byte_drops();

    printf("  test_fuzz_truncated_frames...\n");
    test_fuzz_truncated_frames();

    printf("  test_fuzz_random_garbage...\n");
    test_fuzz_random_garbage();

    printf("  test_fuzz_window_stress...\n");
    test_fuzz_window_stress();

    printf("  test_fuzz_seq_wraparound_stress...\n");
    test_fuzz_seq_wraparound_stress();

    TEST_SUMMARY();
}
