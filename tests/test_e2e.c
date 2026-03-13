/*
 * End-to-end tests using a loopback byte pipe.
 *
 * Tests cover: handshake, data exchange, window flow control, ACK timer,
 * silence timeout, session reset, NAK-triggered retransmission, and the
 * TX ring slot-recycling / seq-aliasing edge cases.
 */
#include "test_framework.h"
#include "test_hal.h"
#include "sdp.h"
#include <string.h>

/* ── Shared state ───────────────────────────────────────────────────────── */

typedef struct {
    sdp_ctx_t   ctx;
    sdp_hal_t   hal;
    hal_state_t hs;
    int         frame_count;
    sdp_frame_t last_frame;
    int         announce_count;
} endpoint_t;

static byte_pipe_t pipe_a_to_b;
static byte_pipe_t pipe_b_to_a;

static void ep_on_frame(sdp_ctx_t *ctx, const sdp_frame_t *f, void *user)
{
    endpoint_t *ep = (endpoint_t *)user;
    (void)ctx;
    ep->last_frame = *f;
    ep->frame_count++;
    if (f->type == SDP_T_ANNOUNCE) ep->announce_count++;
}

/*
 * Fills application bytes (byte 3 onwards) of ANNOUNCE.
 * Bytes 0-2 are filled by sdp_send_announce: protocol_version, window,
 * max_payload.
 */
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

/* Drain all pending bytes between the two endpoints. */
static void pump(endpoint_t *a, endpoint_t *b)
{
    int byte_val;
    while ((byte_val = pipe_pop(a->hs.tx_pipe)) >= 0)
        sdp_rx_byte(&b->ctx, (uint8_t)byte_val);
    while ((byte_val = pipe_pop(b->hs.tx_pipe)) >= 0)
        sdp_rx_byte(&a->ctx, (uint8_t)byte_val);
}

/* ── TEST-E2E-01: full handshake then data exchange ─────────────────────── */
static void test_e2e_handshake_and_data(void)
{
    endpoint_t host, device;
    sdp_status_t st;
    uint8_t payload[4] = {0x01, 0x02, 0x03, 0x04};
    int i;

    pipe_init(&pipe_a_to_b);
    pipe_init(&pipe_b_to_a);
    g_fake_millis = 0;

    ep_init(&host,   &pipe_a_to_b, &pipe_b_to_a);
    ep_init(&device, &pipe_b_to_a, &pipe_a_to_b);

    st = sdp_listen(&device.ctx);
    CHECK_EQ((int)st, (int)SDP_OK);

    {
        uint8_t sync[SDP_SYNC_SEND];
        memset(sync, 0xAA, SDP_SYNC_SEND);
        host.hal.write(sync, SDP_SYNC_SEND, host.hal.user);
    }

    pump(&host, &device);

    {
        sdp_link_state_t state;
        sdp_state(&device.ctx, &state);
        CHECK_EQ((int)state, (int)SDP_STATE_SYNC_ACK);
    }

    pump(&host, &device);

    CHECK_EQ(host.announce_count, 1);
    CHECK_EQ((int)host.last_frame.type, (int)SDP_T_ANNOUNCE);
    /* Byte 0: protocol_version */
    CHECK_EQ((int)host.last_frame.payload[0], (int)SDP_PROTOCOL_VERSION);
    /* Byte 1: window */
    CHECK_EQ((int)host.last_frame.payload[1], (int)SDP_WINDOW);
    /* Byte 2: max_payload */
    CHECK_EQ((int)host.last_frame.payload[2], (int)SDP_MAX_PAYLOAD);
    /* Bytes 3-4: application identity */
    CHECK_EQ((int)host.last_frame.payload[3], 0xDE);
    CHECK_EQ((int)host.last_frame.payload[4], 0xAD);

    host.ctx.state  = SDP_STATE_LINKED;
    host.ctx.tx_seq = 0;

    st = sdp_send(&host.ctx, 0x01, 0, payload, 4);
    CHECK_EQ((int)st, (int)SDP_OK);

    pump(&host, &device);

    {
        sdp_link_state_t state;
        sdp_state(&device.ctx, &state);
        CHECK_EQ((int)state, (int)SDP_STATE_LINKED);
    }
    CHECK_EQ(device.frame_count, 1);
    CHECK_EQ((int)device.last_frame.type, 0x01);
    CHECK_EQ((int)device.last_frame.len, 4);
    for (i = 0; i < 4; i++)
        CHECK_EQ((int)device.last_frame.payload[i], (int)payload[i]);

    {
        uint8_t reply[2] = {0xAB, 0xCD};
        st = sdp_send(&device.ctx, 0x02, 0, reply, 2);
        CHECK_EQ((int)st, (int)SDP_OK);
    }
    pump(&host, &device);

    CHECK_EQ(host.frame_count, 2);  /* ANNOUNCE + reply */
    CHECK_EQ((int)host.last_frame.type, 0x02);
    CHECK_EQ((int)host.last_frame.len, 2);
    CHECK_EQ((int)host.last_frame.payload[0], 0xAB);
    CHECK_EQ((int)host.last_frame.payload[1], 0xCD);
}

/* ── TEST-E2E-02: window flow control ───────────────────────────────────── */
static void test_e2e_window(void)
{
    endpoint_t host, device;
    sdp_status_t st;
    int i;

    pipe_init(&pipe_a_to_b);
    pipe_init(&pipe_b_to_a);
    g_fake_millis = 0;

    ep_init(&host,   &pipe_a_to_b, &pipe_b_to_a);
    ep_init(&device, &pipe_b_to_a, &pipe_a_to_b);

    host.ctx.state   = SDP_STATE_LINKED;
    device.ctx.state = SDP_STATE_LINKED;

    for (i = 0; i < SDP_WINDOW; i++) {
        uint8_t t = (uint8_t)(0x01 + i);
        st = sdp_send(&host.ctx, t, 0, NULL, 0);
        CHECK_EQ((int)st, (int)SDP_OK);
    }

    st = sdp_send(&host.ctx, 0x0F, 0, NULL, 0);
    CHECK_EQ((int)st, (int)SDP_ERR_WINDOW_FULL);
    CHECK(host.ctx.diag.window_full >= 1u);

    pump(&host, &device);
    CHECK_EQ(device.frame_count, SDP_WINDOW);

    advance_ms(55);
    sdp_tick(&device.ctx);
    pump(&host, &device);

    st = sdp_send(&host.ctx, 0x10, 0, NULL, 0);
    CHECK_EQ((int)st, (int)SDP_OK);
}

/* ── TEST-E2E-03: ACK timer ─────────────────────────────────────────────── */
static void test_e2e_ack_timer(void)
{
    endpoint_t host, device;
    uint32_t initial_tx;

    pipe_init(&pipe_a_to_b);
    pipe_init(&pipe_b_to_a);
    g_fake_millis = 0;

    ep_init(&host,   &pipe_a_to_b, &pipe_b_to_a);
    ep_init(&device, &pipe_b_to_a, &pipe_a_to_b);

    host.ctx.state   = SDP_STATE_LINKED;
    device.ctx.state = SDP_STATE_LINKED;

    sdp_send(&host.ctx, 0x01, 0, NULL, 0);
    pump(&host, &device);

    CHECK_EQ(device.frame_count, 1);
    CHECK_EQ((int)device.ctx.ack_pending, 1);

    initial_tx = device.ctx.diag.tx_frames;

    advance_ms(40);
    sdp_tick(&device.ctx);
    CHECK_EQ((int)device.ctx.diag.tx_frames, (int)initial_tx);

    advance_ms(15);
    sdp_tick(&device.ctx);
    CHECK(device.ctx.diag.tx_frames > initial_tx);
    CHECK_EQ((int)device.ctx.ack_pending, 0);
}

/* ── TEST-E2E-04: silence timeout drops link ─────────────────────────────── */
static void test_e2e_silence_timeout(void)
{
    endpoint_t host;
    sdp_hal_t hal;
    hal_state_t hs;
    byte_pipe_t tx, rx;

    pipe_init(&tx); pipe_init(&rx);
    hs.tx_pipe = &tx; hs.rx_pipe = &rx;
    hal.write  = hal_write; hal.read = hal_read;
    hal.millis = hal_millis; hal.user = &hs;
    g_fake_millis = 0;

    memset(&host, 0, sizeof(host));
    host.hs      = hs;
    host.hal     = hal;
    host.hal.user = &host.hs;
    sdp_init(&host.ctx, &host.hal, ep_on_frame, ep_build_announce, &host);

    host.ctx.state      = SDP_STATE_LINKED;
    host.ctx.last_rx_ms = 1;

    g_fake_millis = SDP_SILENCE_TIMEOUT_MS - 1u;
    sdp_tick(&host.ctx);
    {
        sdp_link_state_t state;
        sdp_state(&host.ctx, &state);
        CHECK_EQ((int)state, (int)SDP_STATE_LINKED);
    }

    g_fake_millis = SDP_SILENCE_TIMEOUT_MS + 100u;
    sdp_tick(&host.ctx);
    {
        sdp_link_state_t state;
        sdp_state(&host.ctx, &state);
        CHECK_EQ((int)state, (int)SDP_STATE_SYNC_HUNT);
    }
}

/* ── TEST-E2E-05: sdp_reset ─────────────────────────────────────────────── */
static void test_e2e_reset(void)
{
    endpoint_t host, device;
    sdp_status_t st;

    pipe_init(&pipe_a_to_b);
    pipe_init(&pipe_b_to_a);
    g_fake_millis = 0;

    ep_init(&host,   &pipe_a_to_b, &pipe_b_to_a);
    ep_init(&device, &pipe_b_to_a, &pipe_a_to_b);

    host.ctx.state   = SDP_STATE_LINKED;
    device.ctx.state = SDP_STATE_LINKED;

    sdp_send(&host.ctx, 0x01, 0, NULL, 0);
    pump(&host, &device);

    CHECK((int)host.ctx.tx_seq > 0);

    st = sdp_reset(&host.ctx);
    CHECK_EQ((int)st, (int)SDP_OK);
    CHECK_EQ((int)host.ctx.tx_seq, 0);
    CHECK_EQ((int)host.ctx.peer_ackseq, (int)SDP_SEQ_NONE);
}

/* ── TEST-E2E-06: NAK triggers retransmit with RTX flag ─────────────────── */
static void test_e2e_nak_retransmit(void)
{
    endpoint_t host, device;
    uint8_t payload[4] = {0x11, 0x22, 0x33, 0x44};
    uint32_t tx_before;

    pipe_init(&pipe_a_to_b);
    pipe_init(&pipe_b_to_a);
    g_fake_millis = 0;

    ep_init(&host,   &pipe_a_to_b, &pipe_b_to_a);
    ep_init(&device, &pipe_b_to_a, &pipe_a_to_b);

    host.ctx.state   = SDP_STATE_LINKED;
    device.ctx.state = SDP_STATE_LINKED;

    /* Host sends a frame — stored in tx_ring[0] */
    sdp_send(&host.ctx, 0x01, 0, payload, 4);
    tx_before = host.ctx.diag.tx_frames;

    /* Corrupt a payload byte before delivering to device (offset 5 into
     * the pipe: FLAG + seq_ack + type_flg + len + payload[0] + here).  */
    pipe_a_to_b.buf[(pipe_a_to_b.head + 5u) % PIPE_SIZE] ^= 0xFF;

    /* Feed corrupted frame to device — device sends T_NAK */
    {
        int b;
        while ((b = pipe_pop(&pipe_a_to_b)) >= 0)
            sdp_rx_byte(&device.ctx, (uint8_t)b);
    }
    CHECK(device.ctx.diag.nak_sent >= 1u);

    /* Feed NAK back to host — host retransmits */
    {
        int b;
        while ((b = pipe_pop(&pipe_b_to_a)) >= 0)
            sdp_rx_byte(&host.ctx, (uint8_t)b);
    }
    CHECK(host.ctx.diag.tx_frames > tx_before);

    /* Deliver retransmit to device — should arrive intact with RTX flag */
    {
        int b;
        while ((b = pipe_pop(&pipe_a_to_b)) >= 0)
            sdp_rx_byte(&device.ctx, (uint8_t)b);
    }
    CHECK_EQ((int)device.last_frame.type, 0x01);
    CHECK_EQ((int)(device.last_frame.flags & SDP_FLAG_RTX), (int)SDP_FLAG_RTX);
    CHECK_EQ((int)device.last_frame.len, 4);
    CHECK_EQ((int)device.last_frame.payload[0], 0x11);
    CHECK_EQ((int)device.last_frame.payload[3], 0x44);
}

/* ── TEST-E2E-07: stale NAK does not retransmit recycled slot ───────────── */
static void test_e2e_stale_nak_ignored(void)
{
    endpoint_t host, device;
    uint32_t tx_before;

    pipe_init(&pipe_a_to_b);
    pipe_init(&pipe_b_to_a);
    g_fake_millis = 0;

    ep_init(&host,   &pipe_a_to_b, &pipe_b_to_a);
    ep_init(&device, &pipe_b_to_a, &pipe_a_to_b);

    host.ctx.state   = SDP_STATE_LINKED;
    device.ctx.state = SDP_STATE_LINKED;

    /* Send SDP_WINDOW frames and let device ACK them all, advancing window */
    {
        int i;
        for (i = 0; i < SDP_WINDOW; i++)
            sdp_send(&host.ctx, 0x01, 0, NULL, 0);
        pump(&host, &device);
        advance_ms(55);
        sdp_tick(&device.ctx);
        pump(&host, &device);  /* deliver ACK to host */
    }

    /* Send SDP_WINDOW more frames so slot 0 is now reused with a new seq */
    {
        int i;
        for (i = 0; i < SDP_WINDOW; i++)
            sdp_send(&host.ctx, 0x02, 0, NULL, 0);
        pump(&host, &device);
    }

    tx_before = host.ctx.diag.tx_frames;

    /* Inject a stale NAK referencing seq=0, which has already been recycled */
    {
        /* Build a NAK frame: seq_ack=(0<<4)|0xF=0x0F, type_flg=(0x31<<2)=0xC4,
         * len=1, payload=0x0F (bad_seq_ack for seq=0).
         * Feed raw bytes to host including correct CRC.                    */
        uint8_t body[4];
        uint16_t crc;
        uint8_t flag = 0x7E;
        body[0] = 0x0F;          /* seq_ack */
        body[1] = (0x31u << 2);  /* type_flg: T_NAK, no flags */
        body[2] = 0x01;          /* len = 1 */
        body[3] = 0x0F;          /* payload: bad_seq_ack */
        crc = 0xFFFF;
        {
            uint8_t i, j;
            for (i = 0; i < 4; i++) {
                crc ^= (uint16_t)body[i] << 8;
                for (j = 0; j < 8; j++)
                    crc = (crc & 0x8000u)
                        ? (uint16_t)((crc << 1) ^ 0x1021u)
                        : (uint16_t)(crc << 1);
            }
        }
        sdp_rx_byte(&host.ctx, flag);
        sdp_rx_byte(&host.ctx, body[0]);
        sdp_rx_byte(&host.ctx, body[1]);
        sdp_rx_byte(&host.ctx, body[2]);
        sdp_rx_byte(&host.ctx, body[3]);
        sdp_rx_byte(&host.ctx, (uint8_t)(crc >> 8));
        sdp_rx_byte(&host.ctx, (uint8_t)(crc & 0xFF));
        sdp_rx_byte(&host.ctx, flag);
    }

    /* tx_ring[0 % SDP_WINDOW] now holds a frame with seq != 0, so no RTX */
    CHECK_EQ((int)host.ctx.diag.tx_frames, (int)tx_before);
}

/* ── TEST-E2E-08: SEQ wraparound across SDP_SEQ_MAX ────────────────────── */
static void test_e2e_seq_wraparound(void)
{
    endpoint_t host, device;
    int i;

    pipe_init(&pipe_a_to_b);
    pipe_init(&pipe_b_to_a);
    g_fake_millis = 0;

    ep_init(&host,   &pipe_a_to_b, &pipe_b_to_a);
    ep_init(&device, &pipe_b_to_a, &pipe_a_to_b);

    host.ctx.state   = SDP_STATE_LINKED;
    device.ctx.state = SDP_STATE_LINKED;

    /* Drive seq up to SDP_SEQ_MAX, one frame at a time with ACKs */
    for (i = 0; i <= (int)SDP_SEQ_MAX; i++) {
        sdp_send(&host.ctx, 0x01, 0, NULL, 0);
        pump(&host, &device);
        advance_ms(55);
        sdp_tick(&device.ctx);
        pump(&host, &device);
    }

    /* tx_seq should have wrapped back to 0 */
    CHECK_EQ((int)host.ctx.tx_seq, 0);

    /* Window should be open and a further send should succeed */
    CHECK_EQ((int)sdp_send(&host.ctx, 0x01, 0, NULL, 0), (int)SDP_OK);
}

/* ── TEST-E2E-09: multiple sequential loss/NAK/retransmit cycles ─────────── */
static void test_e2e_multi_loss(void)
{
    endpoint_t host, device;
    uint8_t payload[4] = {0x11, 0x22, 0x33, 0x44};
    int i;

    pipe_init(&pipe_a_to_b);
    pipe_init(&pipe_b_to_a);
    g_fake_millis = 0;

    ep_init(&host,   &pipe_a_to_b, &pipe_b_to_a);
    ep_init(&device, &pipe_b_to_a, &pipe_a_to_b);

    host.ctx.state   = SDP_STATE_LINKED;
    device.ctx.state = SDP_STATE_LINKED;

    /*
     * For each window slot: send a frame, corrupt payload[1] at wire offset 5
     * (safe: does not touch FLAG, seq_ack, or CRC bytes), verify NAK is sent,
     * verify host retransmits, then let the retransmit through so the ACK
     * arrives and the window slot is freed for the next iteration.
     */
    for (i = 0; i < SDP_WINDOW; i++) {
        uint32_t nak_before = device.ctx.diag.nak_sent;
        uint32_t tx_before  = host.ctx.diag.tx_frames;

        sdp_send(&host.ctx, 0x01, 0, payload, 4);

        /* Wire layout (no escaping for these payload values):
         *   [0]=FLAG  [1]=seq_ack  [2]=type_flg  [3]=len=4
         *   [4]=p0    [5]=p1       [6]=p2         [7]=p3
         *   [8]=crc_hi [9]=crc_lo  [10]=FLAG
         * Corrupt p1 — seq_ack is untouched so the NAK names the right seq. */
        pipe_a_to_b.buf[(pipe_a_to_b.head + 5u) % PIPE_SIZE] ^= 0xFF;

        /* Feed corrupted frame to device — device should NAK */
        {
            int b;
            while ((b = pipe_pop(&pipe_a_to_b)) >= 0)
                sdp_rx_byte(&device.ctx, (uint8_t)b);
        }
        CHECK(device.ctx.diag.nak_sent > nak_before);

        /* Feed NAK to host — host retransmits */
        {
            int b;
            while ((b = pipe_pop(&pipe_b_to_a)) >= 0)
                sdp_rx_byte(&host.ctx, (uint8_t)b);
        }
        CHECK(host.ctx.diag.tx_frames > tx_before);

        /* Let retransmit through and ACK so the window slot is freed */
        pump(&host, &device);
        advance_ms(55);
        sdp_tick(&device.ctx);
        pump(&host, &device);
    }
}

/* ── TEST-E2E-10: T_STATS_REQ / T_STATS_RSP round-trip ──────────────────── */
static void test_e2e_stats(void)
{
    endpoint_t host, device;
    sdp_diag_t remote;
    sdp_status_t st;

    pipe_init(&pipe_a_to_b);
    pipe_init(&pipe_b_to_a);
    g_fake_millis = 0;

    ep_init(&host,   &pipe_a_to_b, &pipe_b_to_a);
    ep_init(&device, &pipe_b_to_a, &pipe_a_to_b);

    host.ctx.state   = SDP_STATE_LINKED;
    device.ctx.state = SDP_STATE_LINKED;

    /* Generate some traffic so counters are non-trivial */
    sdp_send(&host.ctx, 0x01, 0, NULL, 0);
    pump(&host, &device);
    advance_ms(55);
    sdp_tick(&device.ctx);
    pump(&host, &device);

    /* Host requests device stats */
    st = sdp_request_stats(&host.ctx);
    CHECK_EQ((int)st, (int)SDP_OK);

    /* Feed T_STATS_REQ to device; device sends T_STATS_RSP */
    {
        int b;
        while ((b = pipe_pop(&pipe_a_to_b)) >= 0)
            sdp_rx_byte(&device.ctx, (uint8_t)b);
    }

    /* Feed T_STATS_RSP back to host */
    {
        int b;
        while ((b = pipe_pop(&pipe_b_to_a)) >= 0)
            sdp_rx_byte(&host.ctx, (uint8_t)b);
    }

    CHECK_EQ((int)host.last_frame.type, (int)SDP_T_STATS_RSP);

    st = sdp_diag_from_stats_rsp(&host.last_frame, &remote);
    CHECK_EQ((int)st, (int)SDP_OK);

    /* Device received one frame (the data frame from host) */
    CHECK(remote.rx_frames >= 1u);
    /* Device transmitted at least the T_ACK (stats_rsp is packed before send) */
    CHECK(remote.tx_frames >= 1u);
    /* No errors expected */
    CHECK_EQ((int)remote.crc_errors, 0);
}

int main(void)
{
    test_e2e_handshake_and_data();
    test_e2e_window();
    test_e2e_ack_timer();
    test_e2e_silence_timeout();
    test_e2e_reset();
    test_e2e_nak_retransmit();
    test_e2e_stale_nak_ignored();
    test_e2e_seq_wraparound();
    test_e2e_multi_loss();
    test_e2e_stats();
    TEST_SUMMARY();
}
