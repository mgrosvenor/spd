/*
 * End-to-end test using Unix domain sockets.
 *
 * Two threads each own an sdp_ctx_t. They communicate over a socket pair
 * (socketpair(AF_UNIX, SOCK_STREAM, 0, fds)).
 *
 * Requires: POSIX threads, Unix domain sockets. Linux / macOS only.
 */

#if defined(__unix__) || defined(__APPLE__)

#include "test_framework.h"
#include "sdp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <pthread.h>

/* ── HAL backed by a blocking file descriptor ───────────────────────────── */

typedef struct {
    int fd;
} fd_hal_state_t;

static int fd_write(const uint8_t *buf, uint8_t len, void *user)
{
    fd_hal_state_t *s = (fd_hal_state_t *)user;
    ssize_t written = 0, total = (ssize_t)len;
    while (written < total) {
        ssize_t n = write(s->fd, buf + written, (size_t)(total - written));
        if (n < 0) return -1;
        written += n;
    }
    return 0;
}

static int fd_read(void *user)
{
    fd_hal_state_t *s = (fd_hal_state_t *)user;
    uint8_t b;
    /* Non-blocking read */
    ssize_t n = read(s->fd, &b, 1);
    return (n == 1) ? (int)b : -1;
}

static uint32_t fd_millis(void *user)
{
    (void)user;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint32_t)(tv.tv_sec * 1000u + tv.tv_usec / 1000u);
}

/* ── Test state ─────────────────────────────────────────────────────────── */

#define MSG_TYPE_PING 0x01
#define MSG_TYPE_PONG 0x02
#define PING_COUNT    5

typedef struct {
    sdp_ctx_t      ctx;
    sdp_hal_t      hal;
    fd_hal_state_t hs;
    int            pings_sent;
    int            pongs_received;
    int            ping_payloads_ok;
    volatile int   done;
} device_state_t;

typedef struct {
    sdp_ctx_t      ctx;
    sdp_hal_t      hal;
    fd_hal_state_t hs;
    sdp_frame_t    announce;
    int            got_announce;
    int            pings_received;
    int            pongs_sent;
    volatile int   done;
} host_state_t;

static void device_on_frame(sdp_ctx_t *ctx, const sdp_frame_t *f, void *user)
{
    device_state_t *dev = (device_state_t *)user;
    if (f->type == MSG_TYPE_PING) {
        /* Echo back a PONG with the same payload */
        sdp_send(ctx, MSG_TYPE_PONG, 0, f->payload, f->len);
        dev->pings_sent++;
    }
}

static sdp_status_t device_build_announce(uint8_t *payload, uint8_t *len, void *user)
{
    (void)user;
    /* Application bytes (byte 3+ of ANNOUNCE; bytes 0-2 filled by transport) */
    payload[0] = 0xC0; payload[1] = 0xFF; payload[2] = 0xEE;
    *len = 3;
    return SDP_OK;
}

static void host_on_frame(sdp_ctx_t *ctx, const sdp_frame_t *f, void *user)
{
    host_state_t *host = (host_state_t *)user;
    (void)ctx;
    if (f->type == SDP_T_ANNOUNCE) {
        host->announce    = *f;
        host->got_announce = 1;
    } else if (f->type == MSG_TYPE_PONG) {
        host->pongs_sent++;
    }
}

static sdp_status_t host_build_announce(uint8_t *payload, uint8_t *len, void *user)
{
    (void)user;
    *len = 0;  /* no application bytes beyond the mandatory transport header */
    return SDP_OK;
}

/* Device thread: listens, then polls for frames and responds */
static void *device_thread(void *arg)
{
    device_state_t *dev = (device_state_t *)arg;
    sdp_listen(&dev->ctx);

    while (!dev->done) {
        sdp_poll(&dev->ctx);
        sdp_tick(&dev->ctx);
        usleep(1000); /* 1 ms */
    }
    return NULL;
}

/* ── Tests ──────────────────────────────────────────────────────────────── */

static void test_socket_handshake_and_ping_pong(void)
{
    int fds[2];
    pthread_t dev_thread;
    device_state_t dev;
    host_state_t   host;
    sdp_frame_t    announce;
    sdp_status_t   st;
    int i;
    uint8_t ping_payload[4] = {0x11, 0x22, 0x33, 0x44};

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, fds) != 0) {
        fprintf(stderr, "socketpair failed: %s\n", strerror(errno));
        CHECK(0);
        return;
    }

    /* Set non-blocking on both ends for sdp_poll/sdp_connect */
    fcntl(fds[0], F_SETFL, O_NONBLOCK);
    fcntl(fds[1], F_SETFL, O_NONBLOCK);

    /* Initialise device */
    memset(&dev, 0, sizeof(dev));
    dev.hs.fd       = fds[1];
    dev.hal.write   = fd_write;
    dev.hal.read    = fd_read;
    dev.hal.millis  = fd_millis;
    dev.hal.user    = &dev.hs;
    sdp_init(&dev.ctx, &dev.hal, device_on_frame, device_build_announce, &dev);

    /* Initialise host */
    memset(&host, 0, sizeof(host));
    host.hs.fd      = fds[0];
    host.hal.write  = fd_write;
    host.hal.read   = fd_read;
    host.hal.millis = fd_millis;
    host.hal.user   = &host.hs;
    sdp_init(&host.ctx, &host.hal, host_on_frame, host_build_announce, &host);

    /* Start device thread */
    pthread_create(&dev_thread, NULL, device_thread, &dev);

    /* Host connects (blocking with 2 s timeout) */
    st = sdp_connect(&host.ctx, 2000, &announce);
    CHECK_EQ((int)st, (int)SDP_OK);

    if (st == SDP_OK) {
        /* Mandatory transport bytes */
        CHECK_EQ((int)announce.payload[0], (int)SDP_PROTOCOL_VERSION);
        CHECK_EQ((int)announce.payload[1], (int)SDP_WINDOW);
        CHECK_EQ((int)announce.payload[2], (int)SDP_MAX_PAYLOAD);
        /* Application bytes (0xC0, 0xFF, 0xEE) start at byte 3 */
        CHECK_EQ((int)announce.payload[3], 0xC0);
        CHECK_EQ((int)announce.len, 6);  /* 3 transport + 3 app */

        /* Send PING_COUNT pings */
        for (i = 0; i < PING_COUNT; i++) {
            ping_payload[0] = (uint8_t)i;
            /* Retry on window full */
            do {
                st = sdp_send(&host.ctx, MSG_TYPE_PING, 0, ping_payload, 4);
                if (st == SDP_ERR_WINDOW_FULL) {
                    sdp_poll(&host.ctx);
                    usleep(5000);
                }
            } while (st == SDP_ERR_WINDOW_FULL);
            CHECK_EQ((int)st, (int)SDP_OK);

            /* Poll for pong */
            {
                int waited = 0;
                int prev_pongs = host.pongs_sent;
                while (host.pongs_sent == prev_pongs && waited < 500) {
                    sdp_poll(&host.ctx);
                    sdp_tick(&host.ctx);
                    usleep(2000);
                    waited += 2;
                }
                CHECK(host.pongs_sent > prev_pongs);
            }
        }
        CHECK_EQ(host.pongs_sent, PING_COUNT);
    }

    dev.done = 1;
    pthread_join(dev_thread, NULL);

    close(fds[0]);
    close(fds[1]);
}

int main(void)
{
    test_socket_handshake_and_ping_pong();
    TEST_SUMMARY();
}

#else

#include <stdio.h>
int main(void)
{
    printf("SKIP: Unix domain sockets not available on this platform\n");
    return 0;
}

#endif /* __unix__ || __APPLE__ */
