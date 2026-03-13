# spd - Simple Datagram Protocol

## Introduction
SPD is a simple framing protocol designed for use over slow noisey links, such as a serial link between two MCUs.
It's designed for use with slow and simple 8-bit MCUs such as AVR, ESP32, and PIC.

SDP provides:

- HDLC-inspired framing with CRC-16 integrity checking
- 4-bit sequence numbers with piggybacked acknowledgement
- Configurable sliding window flow control
- A single mandatory handshake message (ANNOUNCE) carrying a protocol version number
- An UNSUPPORTED response for unknown message types
- A three-function HAL shim enabling portability across platforms

SDP does not define application message types, payload formats, baud rate, physical layer, or any behaviour beyond the transport and handshake. Those are the responsibility of the application built on top of SDP.

The full SPD protcol specification can be found here. 

## 1. Overview

The SDP C API provides a transport-layer framing, sequencing, and flow-control library over any byte-stream transport. The application supplies a HAL struct of three function pointers; `sdp.c` never calls any platform function directly.

**Design constraints:**

- All API functions return `sdp_status_t`. Results are returned via output parameters or callbacks, never via return value.
- No dynamic memory allocation. All state lives in `sdp_ctx_t`, which the caller allocates.
- No thread safety. The caller is responsible for serialising access to a given `sdp_ctx_t`. See Section 10.
- Retransmission is manual. The library sets the RTX flag on request; it does not queue or replay sent frames automatically.
- `sdp_connect()` (host-side handshake) is blocking with a caller-supplied timeout.

---

## 2. Types and Constants

### 2.1 Status Codes

Every API function returns `sdp_status_t`. A function that has nothing meaningful to return on success still returns `SDP_OK`.

```c
typedef enum {
    SDP_OK                  =  0,  /* success                                   */
    SDP_ERR_WINDOW_FULL     = -1,  /* send failed — peer window is full         */
    SDP_ERR_PAYLOAD_TOO_LARGE = -2,/* payload exceeds SDP_MAX_PAYLOAD           */
    SDP_ERR_NOT_LINKED      = -3,  /* operation requires LINKED state           */
    SDP_ERR_ALREADY_LINKED  = -4,  /* sdp_connect/sdp_listen called while LINKED*/
    SDP_ERR_TIMEOUT         = -5,  /* sdp_connect did not complete within timeout*/
    SDP_ERR_BAD_ARG         = -6,  /* NULL pointer or invalid argument          */
    SDP_ERR_HAL             = -7,  /* HAL write returned an error               */
} sdp_status_t;
```

### 2.2 Link State

```c
typedef enum {
    SDP_STATE_SYNC_HUNT = 0,  /* waiting for 16 × 0xAA                         */
    SDP_STATE_SYNC_ACK,       /* ANNOUNCE sent, hold-off timer running          */
    SDP_STATE_SYNC_WAIT,      /* hold-off expired, no HDLC received             */
    SDP_STATE_LINKED,         /* normal operation                               */
} sdp_link_state_t;
```

### 2.3 Frame

Received frames are delivered to the application via the `on_frame` callback as a const pointer to `sdp_frame_t`. The pointer is valid only for the duration of the callback. The application must copy any fields it needs to retain.

```c
typedef struct {
    uint8_t  type;
    uint8_t  flags;                    /* SDP_FLAG_RTX etc.                      */
    uint8_t  seq;
    uint8_t  ackseq;
    uint8_t  len;
    uint8_t  payload[SDP_MAX_PAYLOAD];
} sdp_frame_t;
```

### 2.4 Compile-time Constants

```c
#define SDP_MAX_PAYLOAD  128
#define SDP_SEQ_NONE     0xF   /* startup sentinel in ACKSEQ nibble             */
#define SDP_FLAG_RTX     0x02  /* FLAGS bit 1 — retransmit                      */
```

```c
/* sdp_config.h — override before including sdp.h */
#ifndef SDP_WINDOW
#  define SDP_WINDOW 2
#endif
#if SDP_WINDOW < 1 || SDP_WINDOW > 7
#  error "SDP_WINDOW must be in range 1–7"
#endif
```

### 2.5 Reserved Type Values

```c
#define SDP_T_ANNOUNCE    0x00
#define SDP_T_ACK         0x30
#define SDP_T_NAK         0x31
#define SDP_T_RESET       0x32
#define SDP_T_UNSUPPORTED 0x3C
```

Application message types (0x01–0x2F and 0x33–0x3B) are defined by the application layer, not by this library.

---

## 3. HAL Interface

The HAL struct is provided at `sdp_init` time and must remain valid for the lifetime of the context. The library stores a pointer to it; it does not copy the struct.

```c
typedef struct {
    /*
     * Write len bytes to the transport. Must transmit all len bytes before
     * returning. Returns 0 on success, -1 on error.
     *
     * Called from: sdp_send, sdp_tick, sdp_connect, sdp_listen.
     * Must not be NULL.
     */
    int (*write)(const uint8_t *buf, uint8_t len, void *user);

    /*
     * Return the next received byte (0–255), or -1 if no byte is available.
     * Must be non-blocking.
     *
     * Called from: sdp_poll, sdp_connect.
     * May be NULL if the application feeds bytes manually via sdp_rx_byte.
     * sdp_poll and sdp_connect return SDP_ERR_BAD_ARG if hal.read is NULL.
     */
    int (*read)(void *user);

    /*
     * Return milliseconds elapsed since an arbitrary fixed point (e.g. boot).
     * Allowed to wrap. Must not be NULL.
     *
     * Called from: sdp_tick, sdp_connect.
     */
    uint32_t (*millis)(void *user);

    /*
     * Passed as the last argument to every HAL call. May be NULL.
     * Typically a pointer to a UART handle, file descriptor, or test fixture.
     */
    void *user;
} sdp_hal_t;
```

### HAL Contract

- `write` is called with contiguous byte runs. It will be called multiple times per frame (opening FLAG, header bytes, payload bytes with escaping, CRC bytes, closing FLAG). If atomic frame transmission is needed, the application should buffer at the HAL layer.
- `read` must be non-blocking. The library never calls `read` in a spin loop without yielding.
- `millis` may return the same value across multiple calls within a single tick; the library tolerates coarse resolution down to ~10 ms.
- If `write` returns -1 the library aborts the current frame and returns `SDP_ERR_HAL` to the caller. A partial frame will have been transmitted; the peer's CRC check will reject it.

---

## 4. Initialisation

### `sdp_init`

```c
sdp_status_t sdp_init(sdp_ctx_t       *ctx,
                       const sdp_hal_t *hal,
                       sdp_on_frame_t   on_frame,
                       void            *user);
```

Initialises `ctx` to a clean state. Does not perform any I/O. Safe to call multiple times on the same context (re-initialises cleanly).

**Parameters:**

| Parameter | Description |
|---|---|
| `ctx` | Caller-allocated context. Must not be NULL. |
| `hal` | HAL function table. Pointer is stored; must remain valid. Must not be NULL. `hal->write` and `hal->millis` must not be NULL. |
| `on_frame` | Called synchronously from `sdp_rx_byte` and `sdp_poll` whenever a validated frame is received. Must not be NULL. See Section 7.1. |
| `user` | Passed as the last argument to `on_frame`. May be NULL. |

**Returns:** `SDP_OK`, `SDP_ERR_BAD_ARG` (if `ctx`, `hal`, `hal->write`, `hal->millis`, or `on_frame` is NULL).

**Post-condition:** `ctx` is in `SDP_STATE_SYNC_HUNT`. No bytes transmitted.

### `sdp_on_frame_t`

```c
typedef void (*sdp_on_frame_t)(sdp_ctx_t        *ctx,
                                const sdp_frame_t *frame,
                                void             *user);
```

Called by the library when a complete, CRC-validated frame is received. The `frame` pointer is valid only for the duration of the callback — the application must copy any data it needs to retain. The callback must not call `sdp_init` or `sdp_connect`. It may call `sdp_send`.

---

## 5. Link Establishment

### 5.1 Host Side — `sdp_connect`

```c
sdp_status_t sdp_connect(sdp_ctx_t       *ctx,
                           uint32_t         timeout_ms,
                           sdp_frame_t     *out_announce);
```

Runs the host side of the SDP startup handshake. Transmits 17 × 0xAA, then waits for a valid ANNOUNCE frame. Sends top-up 0xAA bytes at 20 ms intervals until ANNOUNCE is received or `timeout_ms` elapses.

While waiting, received bytes are fed through the frame parser. If any frame other than ANNOUNCE arrives before the handshake completes it is dispatched to `on_frame` normally — this is not an error.

On receiving a valid ANNOUNCE frame the link enters `SDP_STATE_LINKED`, sequence counters are reset to 0, and the function returns.

**Parameters:**

| Parameter | Description |
|---|---|
| `ctx` | Must be initialised. Must not be in `SDP_STATE_LINKED`. |
| `timeout_ms` | Maximum time to wait for ANNOUNCE. Must be > 0. |
| `out_announce` | If non-NULL, the received ANNOUNCE frame is copied here on success. |

**Returns:**

| Status | Meaning |
|---|---|
| `SDP_OK` | Link established. `out_announce` (if non-NULL) holds the ANNOUNCE frame. |
| `SDP_ERR_TIMEOUT` | No ANNOUNCE received within `timeout_ms`. Context remains in SYNC_HUNT. |
| `SDP_ERR_ALREADY_LINKED` | Context is already in `SDP_STATE_LINKED`. |
| `SDP_ERR_BAD_ARG` | `ctx` is NULL, `timeout_ms` is 0, or `hal->read` is NULL. |
| `SDP_ERR_HAL` | `hal->write` failed during sync transmission. |

**Note:** The application must check `out_announce->payload[0]` (protocol_version) after a successful return and handle a version mismatch before issuing any commands.

### 5.2 Device Side — `sdp_listen`

```c
sdp_status_t sdp_listen(sdp_ctx_t *ctx);
```

Arms the device side of the startup handshake. The context enters `SDP_STATE_SYNC_HUNT` and begins counting 0xAA bytes. Bytes must be fed via `sdp_rx_byte` or `sdp_poll`. When 16 consecutive 0xAA bytes are received the library automatically transmits the ANNOUNCE frame (built by calling the application's registered `sdp_on_build_announce` callback — see Section 5.3), starts the 40 ms hold-off timer, and transitions to `SDP_STATE_SYNC_ACK`.

On receiving the first valid HDLC frame after ANNOUNCE is sent, the context transitions to `SDP_STATE_LINKED` and `on_frame` is called with that frame.

`sdp_listen` is non-blocking. It arms state and returns immediately.

**Returns:** `SDP_OK`, `SDP_ERR_BAD_ARG` (NULL ctx), `SDP_ERR_ALREADY_LINKED`.

### 5.3 ANNOUNCE Content — `sdp_on_build_announce_t`

```c
typedef sdp_status_t (*sdp_on_build_announce_t)(uint8_t  *payload,
                                                  uint8_t  *len,
                                                  void     *user);
```

Registered at init (alongside `on_frame`). Called by the library when it needs to transmit ANNOUNCE — once during `sdp_listen` handshake and once per `sdp_send_announce` call. The callback fills `payload` (up to `SDP_MAX_PAYLOAD` bytes) and sets `*len`. Byte 0 must be the SDP protocol version (value: 1). Bytes 1 onwards are application-defined.

This separates the library from any knowledge of what the ANNOUNCE payload contains.

> **Revised `sdp_init` signature** to include the announce builder:

```c
sdp_status_t sdp_init(sdp_ctx_t                  *ctx,
                       const sdp_hal_t             *hal,
                       sdp_on_frame_t               on_frame,
                       sdp_on_build_announce_t      on_build_announce,
                       void                        *user);
```

`on_build_announce` must not be NULL.

### 5.4 Sending ANNOUNCE on demand — `sdp_send_announce`

```c
sdp_status_t sdp_send_announce(sdp_ctx_t *ctx);
```

Calls `on_build_announce`, builds an ANNOUNCE frame, and transmits it. Used to respond to a host-side CMD_GET_ANNOUNCE request, and may be called from within `on_frame` when TYPE=CMD_GET_ANNOUNCE is received.

**Returns:** `SDP_OK`, `SDP_ERR_NOT_LINKED`, `SDP_ERR_WINDOW_FULL`, `SDP_ERR_HAL`, `SDP_ERR_BAD_ARG`.

### 5.5 Link State Query — `sdp_state`

```c
sdp_status_t sdp_state(const sdp_ctx_t *ctx, sdp_link_state_t *out_state);
```

**Returns:** `SDP_OK`, `SDP_ERR_BAD_ARG`.

---

## 6. Sending

### `sdp_send`

```c
sdp_status_t sdp_send(sdp_ctx_t     *ctx,
                       uint8_t        type,
                       uint8_t        flags,
                       const uint8_t *payload,
                       uint8_t        len);
```

Builds and transmits one SDP frame. Blocks only for the duration of `hal->write`. Does not wait for acknowledgement.

**Parameters:**

| Parameter | Description |
|---|---|
| `ctx` | Must be LINKED. |
| `type` | Message type (0x00–0x3F). The library does not restrict which types the application sends, but 0x30–0x33 and 0x3C are transport-reserved — sending them via `sdp_send` is permitted but the application takes responsibility for correctness. |
| `flags` | `SDP_FLAG_RTX` to mark as retransmit; 0 otherwise. |
| `payload` | Pointer to payload bytes. May be NULL if `len` is 0. |
| `len` | Payload length. Must be ≤ `SDP_MAX_PAYLOAD`. |

**Returns:**

| Status | Meaning |
|---|---|
| `SDP_OK` | Frame transmitted. |
| `SDP_ERR_NOT_LINKED` | Context is not in `SDP_STATE_LINKED`. |
| `SDP_ERR_WINDOW_FULL` | Peer receive window is full — caller must wait and retry. |
| `SDP_ERR_PAYLOAD_TOO_LARGE` | `len` > `SDP_MAX_PAYLOAD`. |
| `SDP_ERR_BAD_ARG` | `ctx` NULL, or `payload` NULL with `len` > 0. |
| `SDP_ERR_HAL` | `hal->write` returned -1. |

**Window management:** The library tracks unacknowledged frame count. `sdp_send` returns `SDP_ERR_WINDOW_FULL` rather than blocking. The caller may poll or spin — the window opens as piggybacked ACKSEQs arrive via `sdp_rx_byte` or `sdp_poll`.

**Retransmit:** The library does not retransmit automatically. On receiving T_NAK the application is notified via `on_frame` and may call `sdp_send` again with `flags = SDP_FLAG_RTX`.

---

## 7. Receiving

### 7.1 Byte Feed — `sdp_rx_byte`

```c
sdp_status_t sdp_rx_byte(sdp_ctx_t *ctx, uint8_t byte);
```

Feed one byte into the frame parser. Intended for use from an ISR, DMA completion callback, or a main loop that manages its own read path.

When a complete, CRC-validated frame is assembled, the library:
1. Updates the internal ACKSEQ state.
2. Handles transport-layer frames (T_ACK, T_NAK, T_RESET, UNSUPPORTED) internally.
3. Dispatches all other frames to `on_frame`.

T_NAK and UNSUPPORTED are also passed to `on_frame` so the application can take action (retransmit or log). T_ACK and T_RESET are handled silently unless the application also needs to observe them, in which case it can inspect the type inside `on_frame`.

**Returns:** `SDP_OK` on success (including partial frame — no frame yet complete). `SDP_ERR_BAD_ARG` if `ctx` is NULL. Frame parser errors (CRC failure, overflow) are reported via diagnostics (Section 9) and do not propagate as return values — the parser recovers and continues.

> **Rationale:** Returning an error for every malformed byte would force the caller into complex error handling for events that are routine (line noise, startup garbage). The diagnostics counters provide observability without polluting the call path.

### 7.2 Poll — `sdp_poll`

```c
sdp_status_t sdp_poll(sdp_ctx_t *ctx);
```

Calls `hal->read()` in a loop until it returns -1, feeding each byte to `sdp_rx_byte`. Convenience wrapper for main-loop architectures that do not manage their own read path.

**Returns:** `SDP_OK`, `SDP_ERR_BAD_ARG` (NULL ctx or NULL `hal->read`).

---

## 8. Maintenance

### `sdp_tick`

```c
sdp_status_t sdp_tick(sdp_ctx_t *ctx);
```

Performs periodic maintenance. Must be called regularly — from a main loop, a timer ISR, or an RTOS task. Recommended call rate: every 10–50 ms.

`sdp_tick` performs:

1. **ACK timer:** If a valid frame has been received and no outgoing frame has piggybacked the ACKSEQ within 50 ms, transmits a T_ACK to prevent the sender's window from stalling.
2. **Silence timeout:** If the context is LINKED and no valid frame has been received for the configured silence timeout (default 30 s, overridable via `SDP_SILENCE_TIMEOUT_MS`), transitions to `SDP_STATE_SYNC_HUNT` and calls `on_link_lost` if registered (see Section 8.1).
3. **Hold-off expiry:** If in `SDP_STATE_SYNC_ACK` and the 40 ms hold-off timer has expired with no HDLC frame received, transitions to `SDP_STATE_SYNC_WAIT` then `SDP_STATE_SYNC_HUNT`.

**Returns:** `SDP_OK`, `SDP_ERR_BAD_ARG`, `SDP_ERR_HAL` (if ACK transmission fails).

### 8.1 Link Lost Callback — `sdp_on_link_lost_t`

```c
typedef void (*sdp_on_link_lost_t)(sdp_ctx_t *ctx, void *user);
```

Optional. Registered via `sdp_set_link_lost_cb`:

```c
sdp_status_t sdp_set_link_lost_cb(sdp_ctx_t          *ctx,
                                    sdp_on_link_lost_t  on_link_lost);
```

Called by `sdp_tick` when the silence timeout fires and the link drops to SYNC_HUNT. The callback must not call `sdp_connect` or `sdp_listen` directly (it is called from within `sdp_tick`); schedule reconnection for the next main loop iteration instead.

**Returns:** `SDP_OK`, `SDP_ERR_BAD_ARG`.

### 8.2 Session Reset — `sdp_reset`

```c
sdp_status_t sdp_reset(sdp_ctx_t *ctx);
```

Transmits T_RESET, flushes internal TX state, and resets sequence counters to 0. Use for mid-session recovery. Does not re-run the startup handshake. For a full reconnect, call `sdp_connect` (host) or `sdp_listen` (device) instead.

**Returns:** `SDP_OK`, `SDP_ERR_NOT_LINKED`, `SDP_ERR_BAD_ARG`, `SDP_ERR_HAL`.

---

## 9. Diagnostics

### `sdp_get_diag`

```c
typedef struct {
    uint32_t rx_frames;       /* total frames received and CRC-validated        */
    uint32_t tx_frames;       /* total frames transmitted                       */
    uint32_t crc_errors;      /* frames discarded due to CRC failure            */
    uint32_t rx_overflows;    /* frames discarded due to buffer overflow        */
    uint32_t drop_count;      /* SEQ gaps detected (lost frames)                */
    uint32_t nak_sent;        /* T_NAK frames transmitted                       */
    uint32_t nak_received;    /* T_NAK frames received                          */
    uint32_t window_full;     /* sdp_send calls that returned SDP_ERR_WINDOW_FULL */
    uint32_t hal_errors;      /* hal->write failures                            */
} sdp_diag_t;

sdp_status_t sdp_get_diag(const sdp_ctx_t *ctx, sdp_diag_t *out);
```

Copies current diagnostic counters into `out`. Counters are cumulative since `sdp_init` and never reset by the library (the application may snapshot and diff them).

**Returns:** `SDP_OK`, `SDP_ERR_BAD_ARG`.

---

## 10. Thread Safety

`sdp_ctx_t` is not thread-safe. Concurrent calls on the same context from different threads or ISRs produce undefined behaviour.

**Typical patterns:**

- **Single-threaded / cooperative:** No locking needed. Call `sdp_rx_byte` from main loop or from within a non-preemptive ISR handler that is drained in the main loop.
- **ISR + main loop:** If `sdp_rx_byte` is called from an ISR and `sdp_send` / `sdp_tick` are called from the main loop, the application must use a critical section or double-buffer the ISR input. A common pattern is for the ISR to write raw bytes into a ring buffer and for the main loop to drain that buffer into `sdp_rx_byte` — keeping the library entirely on one execution context.
- **RTOS:** Wrap all calls on a given `sdp_ctx_t` in a mutex, or dedicate a single task to all SDP I/O for that context.

Multiple independent `sdp_ctx_t` instances on the same device are safe to use concurrently provided each instance is accessed from only one execution context at a time.

---

## 11. Usage Patterns

### 11.1 Embedded Device (ATtiny, AVR, ARM) — Polled Main Loop

```c
#include "sdp.h"

static sdp_ctx_t   g_sdp;
static sdp_hal_t   g_hal;

static sdp_status_t build_announce(uint8_t *payload, uint8_t *len, void *user) {
    (void)user;
    payload[0] = 1;                     /* SDP protocol_version             */
    /* application fills payload[1..N] with device identity                 */
    *len = 1;                           /* minimum: protocol_version only    */
    return SDP_OK;
}

static void on_frame(sdp_ctx_t *ctx, const sdp_frame_t *f, void *user) {
    (void)user;
    switch (f->type) {
        case 0x20:  /* CMD_GET_STATE — application-defined */
            /* send FULL_STATE */
            break;
        case 0x21:  /* CMD_GET_ANNOUNCE */
            sdp_send_announce(ctx);
            break;
        case SDP_T_NAK:
            /* optionally retransmit last frame with SDP_FLAG_RTX */
            break;
        default:
            break;
    }
}

int main(void) {
    g_hal.write  = uart_write;
    g_hal.read   = uart_read;
    g_hal.millis = sys_millis;
    g_hal.user   = NULL;

    sdp_init(&g_sdp, &g_hal, on_frame, build_announce, NULL);
    sdp_listen(&g_sdp);   /* arm SYNC_HUNT, await 0xAA bytes from host */

    for (;;) {
        sdp_poll(&g_sdp);   /* drain UART RX into parser */
        sdp_tick(&g_sdp);   /* ACK timer, silence timeout */
        /* application logic */
    }
}
```

### 11.2 Embedded Device — ISR-Fed

```c
/* ISR writes to a ring buffer; main loop drains it */
void USART0_RXC_vect(void) {
    uint8_t b = USART0.RXDATAL;
    ring_push(&g_rx_ring, b);      /* ring buffer — no SDP calls in ISR    */
}

/* main loop */
for (;;) {
    uint8_t b;
    while (ring_pop(&g_rx_ring, &b) == 0)
        sdp_rx_byte(&g_sdp, b);    /* SDP only touched from main loop       */
    sdp_tick(&g_sdp);
}
```

### 11.3 Host — Connect and Query

```c
sdp_ctx_t   ctx;
sdp_hal_t   hal  = { .write = ftdi_write, .read = ftdi_read,
                      .millis = pc_millis, .user = &ftdi_handle };
sdp_frame_t announce;

sdp_init(&ctx, &hal, on_frame, build_announce_host, NULL);

sdp_status_t st = sdp_connect(&ctx, 10000, &announce);
if (st != SDP_OK) {
    fprintf(stderr, "connect failed: %d\n", st);
    return 1;
}

uint8_t protocol_version = announce.payload[0];
if (protocol_version != 1) {
    fprintf(stderr, "unsupported protocol version %d\n", protocol_version);
    return 1;
}

/* request state */
sdp_send(&ctx, 0x20, 0, NULL, 0);   /* CMD_GET_STATE */

/* run event loop */
for (;;) {
    sdp_poll(&ctx);
    sdp_tick(&ctx);
}
```

### 11.4 Unit Testing — Loopback HAL

```c
/* A loopback HAL connects two sdp_ctx_t instances in the same process.   */
/* Useful for testing the full frame path without hardware.                */

typedef struct {
    uint8_t  buf[512];
    uint16_t head, tail, count;
} byte_pipe_t;

static byte_pipe_t pipe_a_to_b, pipe_b_to_a;

static int write_a(const uint8_t *buf, uint8_t len, void *user) {
    (void)user;
    for (uint8_t i = 0; i < len; i++) pipe_push(&pipe_a_to_b, buf[i]);
    return 0;
}
static int read_b(void *user) {
    (void)user;
    return pipe_pop(&pipe_b_to_a);  /* B reads what A wrote */
}
/* ... symmetric for B side ... */

/* Run both sides: */
sdp_poll(&ctx_b);   /* feed ctx_a's output into ctx_b */
sdp_poll(&ctx_a);   /* feed ctx_b's output into ctx_a */
```

---

## 12. Revision History

| Revision | Date | Changes |
|---|---|---|
| 1.0 | 2026-03-13 | Initial release. HAL struct with `write`/`read`/`millis`/`user`. All API functions return `sdp_status_t`; results via output parameters or callbacks. `sdp_init` takes `on_frame` and `on_build_announce` callbacks. `sdp_connect` blocking with timeout. `sdp_listen` non-blocking. `sdp_rx_byte` for manual byte feed; `sdp_poll` as convenience wrapper. `sdp_tick` for ACK timer and silence timeout. `sdp_reset` for mid-session recovery. `sdp_get_diag` for diagnostics. Optional `sdp_on_link_lost_t` callback. Thread safety documented. |
