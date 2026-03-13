/*
 * Tests for CRC-16/CCITT-FALSE and basic frame framing.
 */
#include "test_framework.h"
#include "test_hal.h"
#include "sdp.h"
#include <string.h>

/* ── Helpers ────────────────────────────────────────────────────────────── */

static int g_frame_count = 0;
static sdp_frame_t g_last_frame;

static void on_frame(sdp_ctx_t *ctx, const sdp_frame_t *f, void *user)
{
    (void)ctx; (void)user;
    g_last_frame = *f;
    g_frame_count++;
}

static sdp_status_t build_announce(uint8_t *payload, uint8_t *len, void *user)
{
    (void)user;
    payload[0] = 1;
    *len = 1;
    return SDP_OK;
}

/* Build raw bytes for a frame with given payload, correctly escaped & CRC'd.
   Returns total wire bytes written into out[]. */
static uint8_t build_wire_frame(uint8_t type, uint8_t flags, uint8_t seq, uint8_t ackseq,
                                 const uint8_t *payload, uint8_t plen,
                                 uint8_t *out)
{
    uint8_t i, n = 0;
    uint8_t seq_ack  = (uint8_t)((seq << 4) | (ackseq & 0x0Fu));
    uint8_t type_flg = (uint8_t)((type << 2) | (flags & 0x03u));
    uint16_t crc = 0xFFFF;
    uint8_t raw[133];
    uint8_t rlen = 0;

    raw[rlen++] = seq_ack;
    raw[rlen++] = type_flg;
    raw[rlen++] = plen;
    for (i = 0; i < plen; i++) raw[rlen++] = payload[i];

    /* compute CRC over raw */
    for (i = 0; i < rlen; i++) {
        uint8_t j;
        crc ^= (uint16_t)raw[i] << 8;
        for (j = 0; j < 8; j++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    raw[rlen++] = (uint8_t)(crc >> 8);
    raw[rlen++] = (uint8_t)(crc & 0xFFu);

    /* write opening FLAG */
    out[n++] = 0x7E;
    /* escape and write body */
    for (i = 0; i < rlen; i++) {
        if (raw[i] == 0x7E || raw[i] == 0x7D) {
            out[n++] = 0x7D;
            out[n++] = (uint8_t)(raw[i] ^ 0x20u);
        } else {
            out[n++] = raw[i];
        }
    }
    out[n++] = 0x7E;
    return n;
}

static void feed_bytes(sdp_ctx_t *ctx, const uint8_t *buf, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++)
        sdp_rx_byte(ctx, buf[i]);
}

static void setup_linked(sdp_ctx_t *ctx, sdp_hal_t *hal, hal_state_t *hs,
                          byte_pipe_t *tx, byte_pipe_t *rx)
{
    pipe_init(tx); pipe_init(rx);
    hs->tx_pipe = tx; hs->rx_pipe = rx;
    hal->write  = hal_write;
    hal->read   = hal_read;
    hal->millis = hal_millis;
    hal->user   = hs;
    g_fake_millis = 0;
    g_frame_count = 0;
    sdp_init(ctx, hal, on_frame, build_announce, NULL);
    /* Force LINKED for send/receive tests */
    ctx->state = SDP_STATE_LINKED;
}

/* ── TEST-CRC-01: single bit flip in header ─────────────────────────────── */
static void test_crc_header_flip(void)
{
    sdp_ctx_t ctx; sdp_hal_t hal; hal_state_t hs;
    byte_pipe_t tx, rx;
    uint8_t wire[300];
    uint8_t n;
    int prev_crc_errors;

    setup_linked(&ctx, &hal, &hs, &tx, &rx);
    prev_crc_errors = (int)ctx.diag.crc_errors;
    g_frame_count = 0;

    n = build_wire_frame(0x01, 0, 0, 0xF, NULL, 0, wire);
    /* flip a bit in seq_ack byte (index 1 = first byte after opening FLAG) */
    wire[1] ^= 0x01u;
    feed_bytes(&ctx, wire, n);

    CHECK_EQ(g_frame_count, 0);
    CHECK((int)ctx.diag.crc_errors > prev_crc_errors);
}

/* ── TEST-CRC-02: bit flip in payload ──────────────────────────────────── */
static void test_crc_payload_flip(void)
{
    sdp_ctx_t ctx; sdp_hal_t hal; hal_state_t hs;
    byte_pipe_t tx, rx;
    uint8_t wire[300];
    uint8_t payload[4] = {0x01, 0x02, 0x03, 0x04};
    uint8_t n;

    setup_linked(&ctx, &hal, &hs, &tx, &rx);
    g_frame_count = 0;

    n = build_wire_frame(0x01, 0, 0, 0xF, payload, 4, wire);
    /* flip a bit in payload area (after FLAG + 3 header bytes) */
    wire[5] ^= 0x08u;
    feed_bytes(&ctx, wire, n);

    CHECK_EQ(g_frame_count, 0);
    CHECK(ctx.diag.crc_errors > 0u);
}

/* ── TEST-CRC-03: zeroed CRC ────────────────────────────────────────────── */
static void test_crc_zeroed(void)
{
    sdp_ctx_t ctx; sdp_hal_t hal; hal_state_t hs;
    byte_pipe_t tx, rx;
    uint8_t wire[300];
    uint8_t n;

    setup_linked(&ctx, &hal, &hs, &tx, &rx);
    g_frame_count = 0;

    n = build_wire_frame(0x01, 0, 0, 0xF, NULL, 0, wire);
    /* zero the last two bytes before the closing FLAG */
    wire[n - 3] = 0x00;
    wire[n - 2] = 0x00;
    feed_bytes(&ctx, wire, n);

    CHECK_EQ(g_frame_count, 0);
    CHECK(ctx.diag.crc_errors > 0u);
}

/* ── TEST-CRC-04: recovery after CRC failure ────────────────────────────── */
static void test_crc_recovery(void)
{
    sdp_ctx_t ctx; sdp_hal_t hal; hal_state_t hs;
    byte_pipe_t tx, rx;
    uint8_t wire[300];
    uint8_t n;

    setup_linked(&ctx, &hal, &hs, &tx, &rx);
    g_frame_count = 0;

    /* Bad frame */
    n = build_wire_frame(0x01, 0, 0, 0xF, NULL, 0, wire);
    wire[1] ^= 0x01u;
    feed_bytes(&ctx, wire, n);
    CHECK_EQ(g_frame_count, 0);

    /* Good frame after */
    n = build_wire_frame(SDP_T_ACK, 0, 0, 0xF, NULL, 0, wire);
    feed_bytes(&ctx, wire, n);
    /* T_ACK is consumed internally, not dispatched to on_frame — but no new CRC error */
    CHECK_EQ((int)ctx.diag.crc_errors, 1);
}

/* ── TEST-FRAME-01: minimum valid frame (T_ACK) ─────────────────────────── */
static void test_frame_min(void)
{
    sdp_ctx_t ctx; sdp_hal_t hal; hal_state_t hs;
    byte_pipe_t tx, rx;
    uint8_t wire[300];
    uint8_t n;

    setup_linked(&ctx, &hal, &hs, &tx, &rx);
    g_frame_count = 0;

    n = build_wire_frame(SDP_T_ACK, 0, 0, 0xF, NULL, 0, wire);
    feed_bytes(&ctx, wire, n);

    /* T_ACK not dispatched to on_frame, but no error */
    CHECK_EQ(g_frame_count, 0);
    CHECK_EQ((int)ctx.diag.crc_errors, 0);
}

/* ── TEST-FRAME-02: maximum valid frame (128 byte payload) ──────────────── */
static void test_frame_max(void)
{
    sdp_ctx_t ctx; sdp_hal_t hal; hal_state_t hs;
    byte_pipe_t tx, rx;
    uint8_t payload[128];
    uint8_t wire[300];
    uint8_t n;
    int i;

    for (i = 0; i < 128; i++) payload[i] = 0x55;

    setup_linked(&ctx, &hal, &hs, &tx, &rx);
    g_frame_count = 0;

    n = build_wire_frame(0x01, 0, 0, 0xF, payload, 128, wire);
    feed_bytes(&ctx, wire, n);

    CHECK_EQ(g_frame_count, 1);
    CHECK_EQ((int)g_last_frame.len, 128);
    CHECK_EQ((int)ctx.diag.crc_errors, 0);
}

/* ── TEST-FRAME-03: oversized LEN rejected ──────────────────────────────── */
static void test_frame_oversized(void)
{
    sdp_ctx_t ctx; sdp_hal_t hal; hal_state_t hs;
    byte_pipe_t tx, rx;
    uint8_t wire[300];
    uint8_t n;

    setup_linked(&ctx, &hal, &hs, &tx, &rx);
    g_frame_count = 0;

    /* Manually build a frame with LEN=0x81, bypass helper */
    {
        uint8_t seq_ack = 0x0F;
        uint8_t type_flg = (0x01u << 2);
        uint8_t len_byte = 0x81;
        uint16_t crc = 0xFFFF;
        uint8_t raw[6];
        uint8_t i, rlen = 0, wn = 0;
        raw[rlen++] = seq_ack;
        raw[rlen++] = type_flg;
        raw[rlen++] = len_byte;
        for (i = 0; i < rlen; i++) {
            uint8_t j;
            crc ^= (uint16_t)raw[i] << 8;
            for (j = 0; j < 8; j++)
                crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
        raw[rlen++] = (uint8_t)(crc >> 8);
        raw[rlen++] = (uint8_t)(crc & 0xFFu);
        wire[wn++] = 0x7E;
        for (i = 0; i < rlen; i++) {
            if (raw[i] == 0x7E || raw[i] == 0x7D) {
                wire[wn++] = 0x7D;
                wire[wn++] = (uint8_t)(raw[i] ^ 0x20u);
            } else {
                wire[wn++] = raw[i];
            }
        }
        wire[wn++] = 0x7E;
        n = wn;
    }
    feed_bytes(&ctx, wire, n);
    CHECK_EQ(g_frame_count, 0);

    /* valid frame after should still work */
    n = build_wire_frame(0x01, 0, 0, 0xF, NULL, 0, wire);
    feed_bytes(&ctx, wire, n);
    CHECK_EQ(g_frame_count, 1);
}

/* ── TEST-STUFF-01/02: byte stuffing of 0x7E and 0x7D in payload ────────── */
static void test_stuff_payload(void)
{
    sdp_ctx_t ctx; sdp_hal_t hal; hal_state_t hs;
    byte_pipe_t tx, rx;
    uint8_t payload[2] = {0x7E, 0x7D};
    uint8_t wire[300];
    uint8_t n;

    setup_linked(&ctx, &hal, &hs, &tx, &rx);
    g_frame_count = 0;

    n = build_wire_frame(0x01, 0, 0, 0xF, payload, 2, wire);
    feed_bytes(&ctx, wire, n);

    CHECK_EQ(g_frame_count, 1);
    CHECK_EQ((int)g_last_frame.len, 2);
    CHECK_EQ((int)g_last_frame.payload[0], 0x7E);
    CHECK_EQ((int)g_last_frame.payload[1], 0x7D);
}

/* ── TEST-SEQ-03: SEQ wraps at 0xE → 0x0, never uses 0xF ───────────────── */
static void test_seq_wrap(void)
{
    sdp_ctx_t ctx; sdp_hal_t hal; hal_state_t hs;
    byte_pipe_t tx, rx;
    uint8_t wire[300];
    uint8_t n;
    uint8_t seq;
    /* Feed 16 frames with SEQ 0..E then 0..1 */
    uint8_t expected[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,0,1};
    int i;

    setup_linked(&ctx, &hal, &hs, &tx, &rx);
    g_frame_count = 0;

    for (i = 0; i < 17; i++) {
        seq = (uint8_t)(i <= 14 ? i : i - 15);
        n = build_wire_frame(0x01, 0, seq, 0xF, NULL, 0, wire);
        feed_bytes(&ctx, wire, n);
        if (g_frame_count == i + 1) {
            CHECK_EQ((int)g_last_frame.seq, (int)expected[i]);
        }
    }
    CHECK_EQ(g_frame_count, 17);
}

int main(void)
{
    test_crc_header_flip();
    test_crc_payload_flip();
    test_crc_zeroed();
    test_crc_recovery();
    test_frame_min();
    test_frame_max();
    test_frame_oversized();
    test_stuff_payload();
    test_seq_wrap();
    TEST_SUMMARY();
}
