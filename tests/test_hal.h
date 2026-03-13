#ifndef TEST_HAL_H
#define TEST_HAL_H

/*
 * Minimal test HAL for SDP unit tests.
 * Provides two byte-pipe buffers connecting two sdp_ctx_t instances,
 * plus a simple fake millisecond clock.
 */

#include <stdint.h>
#include <string.h>

#define PIPE_SIZE 4096

typedef struct {
    uint8_t  buf[PIPE_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} byte_pipe_t;

static void pipe_init(byte_pipe_t *p)
{
    memset(p, 0, sizeof(*p));
}

static int pipe_push(byte_pipe_t *p, uint8_t b)
{
    if (p->count >= PIPE_SIZE) return -1;
    p->buf[p->tail] = b;
    p->tail = (uint16_t)((p->tail + 1u) % PIPE_SIZE);
    p->count++;
    return 0;
}

static int pipe_pop(byte_pipe_t *p)
{
    uint8_t b;
    if (p->count == 0) return -1;
    b = p->buf[p->head];
    p->head = (uint16_t)((p->head + 1u) % PIPE_SIZE);
    p->count--;
    return (int)b;
}

/* ── Fake clock ─────────────────────────────────────────────────────────── */

static uint32_t g_fake_millis = 0;

#define advance_ms(ms) do { g_fake_millis += (ms); } while (0)

/* ── HAL state for one side of a loopback ──────────────────────────────── */

typedef struct {
    byte_pipe_t *tx_pipe;  /* this side writes here  */
    byte_pipe_t *rx_pipe;  /* this side reads from here */
} hal_state_t;

static int hal_write(const uint8_t *buf, uint8_t len, void *user)
{
    hal_state_t *hs = (hal_state_t *)user;
    uint8_t i;
    for (i = 0; i < len; i++)
        if (pipe_push(hs->tx_pipe, buf[i]) != 0) return -1;
    return 0;
}

static int hal_read(void *user)
{
    hal_state_t *hs = (hal_state_t *)user;
    return pipe_pop(hs->rx_pipe);
}

static uint32_t hal_millis(void *user)
{
    (void)user;
    return g_fake_millis;
}

#endif /* TEST_HAL_H */
