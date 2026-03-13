# spd — Simple Datagram Protocol

A lightweight, framed, sequenced, and flow-controlled datagram protocol designed for low-bit-rate, unreliable physical connections between low-power, low-speed microcontrollers and a host processor (UART, RS-485, USB CDC). SDP provides the framing, integrity checking, and retransmission that these links require but do not provide natively.

## Features

- HDLC-inspired framing with CRC-16/CCITT-FALSE integrity checking
- 4-bit sequence numbers with piggybacked acknowledgement
- Configurable sliding window flow control (`SDP_WINDOW`, default 2, range 1–7)
- Single mandatory handshake (`ANNOUNCE`) carrying a protocol version number
- `UNSUPPORTED` response for unknown message types
- Three-function HAL shim (`write` / `read` / `millis`) for full portability
- Zero dynamic allocation — all state lives in a caller-supplied `sdp_ctx_t`
- C99, no platform-specific extensions

The full protocol specification is in [SPD_v1.0.md](SPD_v1.0.md).

## Platforms

Tested to compile cleanly for:

| Target | Toolchain |
|---|---|
| x86-64 Linux (native test host) | `gcc` / `clang` |
| Apple Silicon (M4, AArch64) | `clang -arch arm64 -mcpu=apple-m4` |
| ARM Cortex-M4 | `arm-none-eabi-gcc` |
| AVR ATtiny1616 | `avr-gcc -mmcu=attiny1616` |
| ESP32-U4WDH (Xtensa LX6) | `xtensa-esp32-elf-gcc` |
| PIC (XC8) | `xc8-cc` |

## Build

### Requirements

- CMake ≥ 3.13
- A C99 compiler

Install toolchains for your platform with the included setup script:

```sh
./setup.sh        # macOS (brew), Ubuntu/Debian (apt), Fedora/RHEL (dnf)
```

### Build and test

```sh
cmake -B build -S .
cmake --build build
cd build && ctest --output-on-failure
```

### Configure options

| CMake variable | Default | Description |
|---|---|---|
| `SDP_WINDOW` | `2` | Sliding window size (1–7) |
| `SDP_SILENCE_TIMEOUT_MS` | `30000` | Silence timeout before link drop (ms) |

```sh
cmake -B build -S . -DSDP_WINDOW=4
```

## Using as a dependency

### FetchContent (recommended)

```cmake
include(FetchContent)
FetchContent_Declare(spd
    GIT_REPOSITORY https://github.com/yourorg/spd.git
    GIT_TAG        v1.0.0
)
FetchContent_MakeAvailable(spd)

target_link_libraries(myapp PRIVATE sdp::sdp)
```

The `sdp::sdp` target exports its include path automatically — no `include_directories` needed.

### find_package (after install)

```sh
cmake --install build --prefix /usr/local
```

```cmake
find_package(sdp 1.0 REQUIRED)
target_link_libraries(myapp PRIVATE sdp::sdp)
```

### add_subdirectory

```cmake
add_subdirectory(third_party/spd)
target_link_libraries(myapp PRIVATE sdp::sdp)
```

Tests are only built when `spd` is the top-level project, so they will not run when included as a subdirectory.

## API summary

```c
/* Initialise context. hal->write and hal->millis must not be NULL. */
sdp_status_t sdp_init(sdp_ctx_t *ctx, const sdp_hal_t *hal,
                       sdp_on_frame_t on_frame,
                       sdp_on_build_announce_t on_build_announce,
                       void *user);

/* Host side: send sync pattern, block until ANNOUNCE received or timeout. */
sdp_status_t sdp_connect(sdp_ctx_t *ctx, uint32_t timeout_ms,
                          sdp_frame_t *out_announce);

/* Device side: arm sync-hunt state machine (non-blocking). */
sdp_status_t sdp_listen(sdp_ctx_t *ctx);

/* Send one frame. Returns SDP_ERR_WINDOW_FULL if peer window is full. */
sdp_status_t sdp_send(sdp_ctx_t *ctx, uint8_t type, uint8_t flags,
                       const uint8_t *payload, uint8_t len);

/* Feed one byte into the frame parser (ISR / DMA / main-loop safe). */
sdp_status_t sdp_rx_byte(sdp_ctx_t *ctx, uint8_t byte);

/* Drain hal->read() into sdp_rx_byte(). Convenience for polled main loops. */
sdp_status_t sdp_poll(sdp_ctx_t *ctx);

/* Call every 10–50 ms: ACK timer, silence timeout, hold-off expiry. */
sdp_status_t sdp_tick(sdp_ctx_t *ctx);

/* Transmit T_RESET and reset sequence counters. */
sdp_status_t sdp_reset(sdp_ctx_t *ctx);

/* Query link state, diagnostics, register link-lost callback. */
sdp_status_t sdp_state(const sdp_ctx_t *ctx, sdp_link_state_t *out);
sdp_status_t sdp_get_diag(const sdp_ctx_t *ctx, sdp_diag_t *out);
sdp_status_t sdp_set_link_lost_cb(sdp_ctx_t *ctx, sdp_on_link_lost_t cb);
```

Every function returns `sdp_status_t`. Results are delivered via output parameters or the `on_frame` callback — never via return value. See [SPD_v1.0.md](SPD_v1.0.md) for full API documentation.

## Quick start — embedded device (polled)

```c
#include "sdp.h"

static sdp_ctx_t g_sdp;
static sdp_hal_t g_hal;

static sdp_status_t build_announce(uint8_t *payload, uint8_t *len, void *user) {
    (void)user;
    payload[0] = 1;   /* SDP protocol version — must be first */
    *len = 1;
    return SDP_OK;
}

static void on_frame(sdp_ctx_t *ctx, const sdp_frame_t *f, void *user) {
    (void)user;
    switch (f->type) {
        case 0x01:  /* application CMD_PING */
            sdp_send(ctx, 0x02, 0, f->payload, f->len);   /* PONG */
            break;
        case SDP_T_NAK:
            /* optionally retransmit with SDP_FLAG_RTX */
            break;
        default: break;
    }
}

int main(void) {
    g_hal.write  = uart_write;
    g_hal.read   = uart_read;
    g_hal.millis = sys_millis;
    g_hal.user   = NULL;

    sdp_init(&g_sdp, &g_hal, on_frame, build_announce, NULL);
    sdp_listen(&g_sdp);

    for (;;) {
        sdp_poll(&g_sdp);
        sdp_tick(&g_sdp);
    }
}
```

## Quick start — host side (connect and send)

```c
sdp_frame_t announce;
sdp_init(&ctx, &hal, on_frame, build_announce, NULL);

if (sdp_connect(&ctx, 5000, &announce) != SDP_OK) {
    /* timeout */
    return 1;
}
if (announce.payload[0] != 1) {
    /* unsupported protocol version */
    return 1;
}

uint8_t cmd[] = { 0xDE, 0xAD };
sdp_send(&ctx, 0x01, 0, cmd, sizeof(cmd));

for (;;) {
    sdp_poll(&ctx);
    sdp_tick(&ctx);
}
```

## Startup handshake

```
Host                                Device
  → 8 × 0xAA (sync pattern)
                                    [8 consecutive 0xAA detected]
                                    ← ANNOUNCE (type=0x00, SEQ=0)
  [valid HDLC frame → link up]
  [verify protocol_version byte]
  → first command
                                    [first HDLC frame → TX enabled]
```

The host retransmits one top-up `0xAA` byte every 20 ms until ANNOUNCE is received or the `timeout_ms` budget (passed to `sdp_connect`) expires.

## HAL implementation

Implement three functions for your platform:

```c
static int my_write(const uint8_t *buf, uint8_t len, void *user) {
    /* write all len bytes; return 0 on success, -1 on error */
}
static int my_read(void *user) {
    /* return next byte (0–255) or -1 if none available (non-blocking) */
}
static uint32_t my_millis(void *user) {
    /* milliseconds since boot, allowed to wrap at 2^32 */
}
```

The `user` pointer in `sdp_hal_t` is forwarded to every HAL call — use it for a UART handle, file descriptor, or test fixture.

## Diagnostics

```c
sdp_diag_t d;
sdp_get_diag(&ctx, &d);
printf("rx=%u tx=%u crc_err=%u drop=%u nak_sent=%u win_full=%u\n",
       d.rx_frames, d.tx_frames, d.crc_errors,
       d.drop_count, d.nak_sent, d.window_full);
```

## Thread safety

`sdp_ctx_t` is not thread-safe. The recommended pattern for ISR + main-loop architectures is to have the ISR push raw bytes into a ring buffer and drain that buffer into `sdp_rx_byte` from the main loop, keeping all SDP calls on one execution context. See [SPD_v1.0.md §10](SPD_v1.0.md#10-thread-safety) for details.
