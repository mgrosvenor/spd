/*
 * End-to-end test using a loopback byte pipe (simulating two endpoints).
 *
 * Tests the full handshake + data exchange path without actual sockets,
 * allowing the test to run on all platforms including embedded cross-builds.
 *
 * A separate test_e2e_socket.c provides the Unix domain socket variant
 * for host platforms.
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

static sdp_status_t ep_build_announce(uint8_t *payload, uint8_t *len, void *user)
{
    (void)user;
    payload[0] = 1;  /* protocol version */
    payload[1] = 0xDE; payload[2] = 0xAD; /* device identity */
    *len = 3;
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

/* Drain all pending bytes from A's TX pipe into B's RX parser and vice versa */
static void pump(endpoint_t *a, endpoint_t *b)
{
    int byte_val;
    /* A wrote → feed to B */
    while ((byte_val = pipe_pop(a->hs.tx_pipe)) >= 0)
        sdp_rx_byte(&b->ctx, (uint8_t)byte_val);
    /* B wrote → feed to A */
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

    /* Arm device listener */
    st = sdp_listen(&device.ctx);
    CHECK_EQ((int)st, (int)SDP_OK);

    /* Host sends SDP_SYNC_SEND x 0xAA sync bytes */
    {
        uint8_t sync[SDP_SYNC_SEND];
        memset(sync, 0xAA, SDP_SYNC_SEND);
        host.hal.write(sync, SDP_SYNC_SEND, host.hal.user);
    }

    /* Feed sync bytes to device */
    pump(&host, &device);

    /* Device should now be in SYNC_ACK and have sent ANNOUNCE */
    {
        sdp_link_state_t state;
        sdp_state(&device.ctx, &state);
        CHECK_EQ((int)state, (int)SDP_STATE_SYNC_ACK);
    }

    /* Feed ANNOUNCE from device to host */
    pump(&host, &device);

    /* Host should have received the ANNOUNCE in on_frame */
    CHECK_EQ(host.announce_count, 1);
    CHECK_EQ((int)host.last_frame.type, (int)SDP_T_ANNOUNCE);
    CHECK_EQ((int)host.last_frame.payload[0], 1);  /* protocol version */

    /* Manually transition host to LINKED (simulate what sdp_connect does) */
    host.ctx.state = SDP_STATE_LINKED;
    host.ctx.tx_seq = 0;

    /* Send first command from host → device */
    st = sdp_send(&host.ctx, 0x01, 0, payload, 4);
    CHECK_EQ((int)st, (int)SDP_OK);

    /* Device receives it — this also transitions device to LINKED */
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

    /* Device replies */
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

/* ── TEST-E2E-02: window flow control ────────────────────────────────────── */
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

    /* Send SDP_WINDOW frames — all should succeed */
    for (i = 0; i < SDP_WINDOW; i++) {
        uint8_t t = (uint8_t)(0x01 + i);
        st = sdp_send(&host.ctx, t, 0, NULL, 0);
        CHECK_EQ((int)st, (int)SDP_OK);
    }

    /* Next send should fail — window full */
    st = sdp_send(&host.ctx, 0x0F, 0, NULL, 0);
    CHECK_EQ((int)st, (int)SDP_ERR_WINDOW_FULL);
    CHECK(host.ctx.diag.window_full >= 1u);

    /* Drain frames into device */
    pump(&host, &device);
    CHECK_EQ(device.frame_count, SDP_WINDOW);

    /* Device sends ACK (via tick after 50ms) */
    advance_ms(55);
    sdp_tick(&device.ctx);
    pump(&host, &device);

    /* Now host window should be open again */
    st = sdp_send(&host.ctx, 0x10, 0, NULL, 0);
    CHECK_EQ((int)st, (int)SDP_OK);
}

/* ── TEST-E2E-03: ACK timer ──────────────────────────────────────────────── */
static void test_e2e_ack_timer(void)
{
    endpoint_t host, device;
    uint16_t initial_tx;

    pipe_init(&pipe_a_to_b);
    pipe_init(&pipe_b_to_a);
    g_fake_millis = 0;

    ep_init(&host,   &pipe_a_to_b, &pipe_b_to_a);
    ep_init(&device, &pipe_b_to_a, &pipe_a_to_b);

    host.ctx.state   = SDP_STATE_LINKED;
    device.ctx.state = SDP_STATE_LINKED;

    /* Host sends a frame to device */
    sdp_send(&host.ctx, 0x01, 0, NULL, 0);
    pump(&host, &device);

    CHECK_EQ(device.frame_count, 1);
    CHECK_EQ((int)device.ctx.ack_pending, 1);

    initial_tx = (uint16_t)device.ctx.diag.tx_frames;

    /* Advance < 50ms — no ACK yet */
    advance_ms(40);
    sdp_tick(&device.ctx);
    CHECK_EQ((int)device.ctx.diag.tx_frames, (int)initial_tx);

    /* Advance past 50ms threshold */
    advance_ms(15);
    sdp_tick(&device.ctx);
    CHECK((int)device.ctx.diag.tx_frames > (int)initial_tx);
    CHECK_EQ((int)device.ctx.ack_pending, 0);
}

/* ── TEST-E2E-04: silence timeout drops link ─────────────────────────────── */
static void test_e2e_silence_timeout(void)
{
    endpoint_t host;
    sdp_hal_t hal; hal_state_t hs;
    byte_pipe_t tx, rx;
    int link_lost_called = 0;

    pipe_init(&tx); pipe_init(&rx);
    hs.tx_pipe = &tx; hs.rx_pipe = &rx;
    hal.write = hal_write; hal.read = hal_read;
    hal.millis = hal_millis; hal.user = &hs;
    g_fake_millis = 0;

    memset(&host, 0, sizeof(host));
    host.hs = hs;
    host.hal = hal;
    host.hal.user = &host.hs;
    sdp_init(&host.ctx, &host.hal, ep_on_frame, ep_build_announce, &host);

    host.ctx.state      = SDP_STATE_LINKED;
    host.ctx.last_rx_ms = 1;  /* non-zero so silence detection activates */

    /* Just before timeout — should still be linked */
    g_fake_millis = SDP_SILENCE_TIMEOUT_MS - 1u;
    sdp_tick(&host.ctx);
    {
        sdp_link_state_t state;
        sdp_state(&host.ctx, &state);
        CHECK_EQ((int)state, (int)SDP_STATE_LINKED);
    }
    (void)link_lost_called;

    /* Past timeout */
    g_fake_millis = SDP_SILENCE_TIMEOUT_MS + 100u;
    sdp_tick(&host.ctx);
    {
        sdp_link_state_t state;
        sdp_state(&host.ctx, &state);
        CHECK_EQ((int)state, (int)SDP_STATE_SYNC_HUNT);
    }
}

/* ── TEST-E2E-05: sdp_reset ──────────────────────────────────────────────── */
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

    /* Advance seq counters */
    sdp_send(&host.ctx, 0x01, 0, NULL, 0);
    pump(&host, &device);

    CHECK((int)host.ctx.tx_seq > 0);

    st = sdp_reset(&host.ctx);
    CHECK_EQ((int)st, (int)SDP_OK);
    CHECK_EQ((int)host.ctx.tx_seq, 0);
    CHECK_EQ((int)host.ctx.tx_unacked, 0);
}

int main(void)
{
    test_e2e_handshake_and_data();
    test_e2e_window();
    test_e2e_ack_timer();
    test_e2e_silence_timeout();
    test_e2e_reset();
    TEST_SUMMARY();
}
