# spd — Simple Datagram Protocol

A lightweight, framed, sequenced, and flow-controlled datagram protocol designed for low-bit-rate, unreliable physical connections between low-power, low-speed microcontrollers (UART, RS-485, USB CDC). SDP provides the framing, integrity checking, and retransmission that these links require but do not provide natively.

## Features

- HDLC-inspired framing with CRC-16/CCITT-FALSE integrity checking
- 4-bit sequence numbers with piggybacked acknowledgement
- **Configurable sliding window flow control** — at most `SDP_WINDOW` (default 2, range 1–7) unacknowledged frames in flight; prevents buffer overrun on resource-constrained devices and enables pipelining at higher baud rates. An explicit T_ACK is sent within 50 ms if no outgoing frame is available to carry the acknowledgement piggyback.
- Single mandatory handshake (`ANNOUNCE`) carrying a protocol version number
- `UNSUPPORTED` response for unknown message types
- Three-function HAL shim (`write` / `read` / `millis`) for full portability
- Zero dynamic allocation — all state lives in a caller-supplied `sdp_ctx_t`
- **Single-header library** — one `#include`, no separate `.c` file to compile
- C99, no platform-specific extensions

The full protocol specification is in [SPD_v1.0.md](SPD_v1.0.md).

## Single-header usage

SDP is distributed as a single header file (`sdp.h`) following the
[STB-style](https://github.com/nothings/stb#how-do-i-use-these-files) pattern.

In **exactly one** translation unit, define `SDP_IMPLEMENTATION` before including:

```c
#define SDP_IMPLEMENTATION
#include "sdp.h"
```

In every other translation unit, include normally for declarations only:

```c
#include "sdp.h"
```

All compile-time configuration (`SDP_WINDOW`, `SDP_MAX_PAYLOAD`,
`SDP_SILENCE_TIMEOUT_MS`) must be defined before the implementation include,
or passed via the compiler (`-DSDP_WINDOW=4`).

## Platforms

Tested to compile cleanly for:

| Target | Toolchain |
|---|---|
| Apple M4 Silicon (native test host) | `clang -arch arm64 -mcpu=apple-m4` |
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
    GIT_REPOSITORY https://github.com/mgrosvenor/spd.git
    GIT_TAG        v1.1.0
)
FetchContent_MakeAvailable(spd)

target_link_libraries(myapp PRIVATE sdp::sdp)
```

The `sdp::sdp` target is an INTERFACE library — it exports its include path
automatically. Define `SDP_IMPLEMENTATION` in exactly one of your source files.

### find_package (after install)

```sh
cmake --install build --prefix /usr/local
```

```cmake
find_package(sdp 1.1 REQUIRED)
target_link_libraries(myapp PRIVATE sdp::sdp)
```

### add_subdirectory

```cmake
add_subdirectory(third_party/spd)
target_link_libraries(myapp PRIVATE sdp::sdp)
```

Tests are only built when `spd` is the top-level project, so they will not run when included as a subdirectory.

### Bare-metal / no CMake

Copy `src/sdp.h` into your project and add its directory to your include path.
No other files are required.

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
#define SDP_IMPLEMENTATION
#include "sdp.h"

static sdp_ctx_t g_sdp;
static const sdp_hal_t g_hal = { uart_write, uart_read, sys_millis, NULL };

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
        default: break;
    }
}

int main(void) {
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
#define SDP_IMPLEMENTATION
#include "sdp.h"

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

## Performance reference

### Frame overhead and efficiency

Every SDP frame carries 7 bytes of overhead (2 FLAGS + 3 header + 2 CRC) before byte stuffing. Byte stuffing adds at most one extra byte per stuffed byte; in practice negligible for arbitrary binary data.

| Payload (bytes) | Efficiency |
|---|---|
| 1 | 12.5% |
| 8 | 53% |
| 32 | 82% |
| 128 | 95% |
| 255 | 97% |

### One-way frame latency (worst-case frame, no stuffing, 8N1)

| Baud rate | 255 B payload | 128 B payload | 32 B payload |
|---|---|---|---|
| 9 600 | 274 ms | 141 ms | 41 ms |
| 19 200 | 137 ms | 70 ms | 20 ms |
| 115 200 | 22.8 ms | 11.7 ms | 3.4 ms |
| 460 800 | 5.7 ms | 2.9 ms | 0.8 ms |
| 1 000 000 | 2.6 ms | 1.35 ms | 0.39 ms |

Round-trip latency is approximately 2× one-way. The 50 ms T_ACK timer adds up to 50 ms to the effective round-trip if no piggybacked ACK is available.

### Sustained throughput (SDP_WINDOW = 2)

Maximum payload throughput at 19 200 baud with 128 B payload and window = 2:

```
throughput ≈ (2 × 128 B) / (2 × 70 ms) ≈ 1.83 kB/s  (~76 % of line rate)
```

Increasing `SDP_WINDOW` proportionally increases throughput up to the line-rate ceiling.

## Thread safety

`sdp_ctx_t` is not thread-safe. The recommended pattern for ISR + main-loop architectures is to have the ISR push raw bytes into a ring buffer and drain that buffer into `sdp_rx_byte` from the main loop, keeping all SDP calls on one execution context. See [SPD_v1.0.md §10](SPD_v1.0.md#10-thread-safety) for details.
