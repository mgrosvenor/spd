# Serial Datagram Protocol (SDP) — Specification

*Revision 1.2 — 2026-03-14*

---

## Table of Contents

1. [Overview](#1-overview)
2. [Transport Layer](#2-transport-layer)
3. [Startup Handshake](#3-startup-handshake)
4. [Message Types](#4-message-types)
5. [Sample Implementation](#5-sample-implementation)
6. [Performance Reference](#6-performance-reference)
7. [Testing Specification](#7-testing-specification)
8. [Revision History](#8-revision-history)

---

## 1. Overview

SDP is a lightweight, framed, sequenced, and flow-controlled datagram protocol designed for low-bit-rate, unreliable physical connections between low-power, low-speed microcontrollers. Target transports are UART, RS-485, and USB CDC — links that carry raw bytes with no built-in framing, integrity checking, or delivery guarantee.

SDP can be carried over TCP or UDP for testing or simulation purposes, but it adds no value in those circumstances: TCP already provides ordering and reliable delivery, and neither transport exposes the bit-error or byte-loss failure modes that SDP is designed to handle.

SDP provides:
- HDLC-inspired framing with CRC-16 integrity checking
- 4-bit sequence numbers with piggybacked acknowledgement
- **Configurable sliding window flow control** — the sender may have at most `SDP_WINDOW` (default 2, range 1–7) unacknowledged frames in flight. If the window is full, `sdp_send` returns `SDP_ERR_WINDOW_FULL` and the caller must wait. Flow control prevents buffer overrun on resource-constrained devices and allows pipelining at higher baud rates. An explicit T_ACK is sent within 50 ms if no outgoing frame is available to piggyback the acknowledgement.
- A single mandatory handshake message (ANNOUNCE) carrying a protocol version number
- An UNSUPPORTED response for unknown message types
- A three-function HAL shim enabling portability across platforms

SDP does **not** define application message types, payload formats, baud rate, physical layer, or any behaviour beyond the transport and handshake. Those are the responsibility of the application built on top of SDP.

### 1.1 Terminology

| Term | Meaning |
|---|---|
| Device | The SDP peer that transmits ANNOUNCE as the handshake acknowledgement. Typically an embedded controller. |
| Host | The SDP peer that initiates the handshake. Typically a processor or desktop process. |
| Frame | One complete unit of transmission: FLAG + header + payload + CRC + FLAG. |
| Link session | The period from a successful handshake to the next SYNC_HUNT entry. |

### 1.2 Single-Header Library Rule

The SDP transport implementation is distributed as a single header file (`sdp.h`) following the STB-style pattern. It must have zero dependency on application state. It communicates with the rest of the system only through the HAL interface (Section 5.1) and its own `sdp_ctx_t` context struct.

In exactly one translation unit, define `SDP_IMPLEMENTATION` before including to emit the implementation:

```c
#define SDP_IMPLEMENTATION
#include "sdp.h"
```

All other translation units include `sdp.h` without the define for declarations only. Any application built on SDP — including a bootloader running on the same device — uses the same unmodified `sdp.h`.

---

## 2. Transport Layer

### 2.1 Framing

Framing follows HDLC (ISO 13239). Every frame is delimited by the flag byte 0x7E at both start and end. The 4-byte header packs SEQ and ACKSEQ into nibbles and TYPE and FLAGS into a single byte.

```
┌────────┬──────────────────┬──────────────────┬────────┬────────────────────┬─────────┬────────┐
│  FLAG  │  SEQ[3:0]        │  TYPE[5:0]       │  LEN   │      PAYLOAD       │  CRC16  │  FLAG  │
│  0x7E  │  ACKSEQ[3:0]     │  FLAGS[1:0]      │ 0–0xFF │  0–SDP_MAX_PAYLOAD │  2 bytes│  0x7E  │
└────────┴──────────────────┴──────────────────┴────────┴────────────────────┴─────────┴────────┘
   1 B           1 B                 1 B           1 B           N bytes         2 bytes    1 B
```

- **Minimum frame on wire:** 8 bytes (zero-length payload)
- **Maximum frame on wire:** `SDP_MAX_PAYLOAD + 7` bytes (before escaping)
- **Buffer slot (FLAG bytes stripped):** `SDP_SLOT_SIZE` = `SDP_MAX_PAYLOAD + 5` bytes

### 2.2 Field Definitions

| Field | Size | Range | Description |
|---|---|---|---|
| FLAG | 1 B | 0x7E | HDLC frame delimiter. Must be escaped if it appears in the frame body — see Section 2.3. |
| SEQ / ACKSEQ | 1 B | nibbles 0x0–0xF | High nibble: sender SEQ (0–14, wraps 14→0; see Section 2.5). Low nibble: piggybacked ACK — SEQ of last valid frame received from the other side. 0xF = startup sentinel (no frame received yet). |
| TYPE / FLAGS | 1 B | TYPE 0x00–0x3F | High 6 bits: message type (0–63). Low 2 bits: FLAGS. Bit 1 = RTX (this frame is a retransmit). Bit 0 = reserved, always transmit 0. |
| LEN | 1 B | 0x00–0xFF | Payload length in bytes (0–`SDP_MAX_PAYLOAD`), measured after unescaping. Frames with LEN > `SDP_MAX_PAYLOAD` must be rejected. |
| PAYLOAD | N B | 0x00–0xFF | Application data. Content defined by TYPE. May be empty (LEN=0). |
| CRC16 | 2 B | 0x0000–0xFFFF | CRC-16/CCITT-FALSE over SEQ_ACKSEQ + TYPE_FLAGS + LEN + PAYLOAD (pre-escape values). Big-endian, high byte first. |

### 2.3 Byte Stuffing (HDLC Transparency)

The flag byte 0x7E and escape byte 0x7D must not appear raw in the frame body (SEQ_ACKSEQ through CRC16 inclusive). If either value occurs it is replaced by a two-byte escape sequence on transmit and restored on receipt.

| Original byte | Transmitted as | Notes |
|---|---|---|
| 0x7E | 0x7D  0x5E | Flag byte escaped |
| 0x7D | 0x7D  0x5D | Escape byte self-escaped |

CRC16 is computed on the original pre-escape bytes. Escaping is applied after CRC calculation on transmit and reversed before CRC verification on receipt.

> **NOTE** Buffer slots are fixed at 260 bytes (3-byte header + 255-byte max payload + 2-byte CRC, FLAG bytes stripped).

### 2.4 CRC-16/CCITT-FALSE

- Polynomial: 0x1021
- Initial value: 0xFFFF
- No input or output bit reflection
- No final XOR
- Coverage: SEQ_ACKSEQ byte, TYPE_FLAGS byte, LEN byte, all PAYLOAD bytes — all pre-escape values
- Transmitted big-endian (high byte first); both CRC bytes subject to byte stuffing

```c
uint16_t sdp_crc16(const uint8_t *data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}
```

### 2.5 Sequence Numbers

Each side maintains an independent 4-bit transmit sequence counter initialised to 0x0 at power-on and on every successful link establishment. The counter increments after each transmitted frame and wraps through values 0x0–`SDP_SEQ_MAX` only — value 0xF is permanently reserved as the startup sentinel and is never used as a transmit SEQ.

`SDP_SEQ_MAX` and the usable sequence space size `SDP_SEQ_SIZE` are derived from `SDP_WINDOW` at compile time:

```
SDP_SEQ_SIZE = (15 / SDP_WINDOW) * SDP_WINDOW   /* integer division */
SDP_SEQ_MAX  = SDP_SEQ_SIZE - 1
```

This yields the largest sequence space that is (a) an integer multiple of `SDP_WINDOW`, and (b) fits within the 15 usable SEQ values (0x0–0xE). For all valid window sizes (1–7) the result is at least 2× the window size, satisfying the duplicate-detection constraint. 0xF is never reached (maximum `SDP_SEQ_MAX` = 14 at `SDP_WINDOW` = 1, 3, or 5).

Tying `SDP_SEQ_SIZE` to an exact multiple of `SDP_WINDOW` ensures the TX retransmit buffer (Section 2.9) is collision-free by construction: any `SDP_WINDOW` consecutive SEQ values, when indexed as `seq % SDP_WINDOW`, always map to distinct slots.

SEQ occupies the high nibble of byte 1. ACKSEQ occupies the low nibble. ACKSEQ carries the SEQ of the last frame successfully received and CRC-validated from the other side. The startup sentinel value 0xF in ACKSEQ indicates no frame has been received yet.

Sequence numbers serve two purposes:

1. **Duplicate detection** — a frame carrying a SEQ value already processed (RTX flag set) is discarded without being dispatched to the application layer.
2. **Loss detection** — a gap in received SEQ values indicates one or more dropped frames. The receiver increments a drop counter but does not automatically request retransmission; that is an application-layer decision.

### 2.6 Flow Control

Window size is controlled by the compile-time constant `SDP_WINDOW`, defaulting to 2. Define it before the `SDP_IMPLEMENTATION` include or pass via the compiler (`-DSDP_WINDOW=4`):

```c
#define SDP_WINDOW 4          /* before SDP_IMPLEMENTATION */
#define SDP_IMPLEMENTATION
#include "sdp.h"
```

The constraint `SDP_WINDOW <= 7` ensures `SDP_SEQ_SIZE >= 2 * SDP_WINDOW` for all valid window values, which is required for correct duplicate detection: the receiver must be able to distinguish a new frame from a retransmit when the window is fully in flight.

The sender may have at most `SDP_WINDOW` unacknowledged frames in flight at any time. A frame is acknowledged when the receiver's outgoing ACKSEQ nibble equals or advances past that frame's SEQ value. If all slots are occupied the sender must wait before transmitting another frame.

```
Each side:  SDP_WINDOW receive slots × SDP_SLOT_SIZE bytes = SDP_WINDOW × SDP_SLOT_SIZE bytes
```

> **NOTE** Implementations may use receive buffers larger than `SDP_WINDOW` slots but must never send more than `SDP_WINDOW` unacknowledged frames to a peer that declares support for only `SDP_WINDOW` slots. The ANNOUNCE payload may be used by applications to advertise the window size if negotiation is needed; SDP itself does not negotiate this.

### 2.7 Explicit Acknowledgement Timing

When a side receives a valid frame and has no application data to transmit within **50 ms**, it must send a T_ACK frame (0x30) to prevent the sender's window from stalling. This 50 ms threshold applies to both sides.

### 2.8 Receiver State Machine

```
HUNT        — discard bytes until 0x7E received
  ↓ 0x7E
IN_FRAME    — accumulate bytes into SDP_SLOT_SIZE-byte slot (FLAG bytes not stored)
  ↓ 0x7D    → ESCAPED (next byte XOR 0x20, then back to IN_FRAME)
  ↓ 0x7E    → end of frame: validate LEN ≤ SDP_MAX_PAYLOAD, verify CRC16
               PASS → dispatch to message handler, update ACKSEQ nibble
               FAIL → discard frame, increment error counter, back to HUNT
  ↓ overflow → discard frame, back to HUNT
```

### 2.9 TX Retransmit Buffer

The sender maintains a statically-allocated TX ring of `SDP_WINDOW` slots. Each slot holds the type, flags, sequence number, length, and payload of one transmitted-but-unacknowledged frame.

On transmit, the frame is stored in `tx_ring[seq % SDP_WINDOW]` before being put on wire. Because `SDP_SEQ_SIZE` is an exact multiple of `SDP_WINDOW` (Section 2.5), any `SDP_WINDOW` consecutive SEQ values map to distinct slots — no aliasing is possible.

On receipt of a T_NAK (Section 4.5), the sender checks `tx_ring[bad_seq % SDP_WINDOW]`. If the stored slot's SEQ matches `bad_seq` the frame is retransmitted with the RTX flag set. A mismatch means the slot has already been recycled (the frame was acked before the NAK arrived) and no retransmit is performed.

Slots are recycled naturally: a slot at index `i` can only be overwritten when the window advances far enough to reuse that index, which requires the peer to have acknowledged the frame previously stored there.

---

## 3. Startup Handshake

A byte-level handshake establishes link readiness before any HDLC frames are exchanged. The device holds TX Hi-Z and discards all RX input until the sync pattern is detected.

### 3.1 Sync Pattern

The host transmits **17 consecutive bytes of 0xAA** (`10101010`). 0xAA is self-clocking and cannot be mistaken for an HDLC flag (0x7E). 17 bytes are sent so that even if the first byte is lost to line noise on startup, 16 consecutive bytes are guaranteed to arrive intact.

The device counts consecutive 0xAA bytes, resetting the counter on any non-0xAA byte. On 16 consecutive 0xAA bytes received, the device transmits an **ANNOUNCE frame (0x00, SEQ=0x0)** as its handshake acknowledgement. ANNOUNCE is the first and only unsolicited frame sent on link establishment — it provides the protocol version before any commands are exchanged. The host should process ANNOUNCE before issuing any commands.

### 3.2 Optimistic Path

```
Host                                   Device
[ready to connect]                     [waiting, TX Hi-Z, counting 0xAA]
  → 17 × 0xAA
                                       [16 consecutive 0xAA seen]
                                       [40 ms hold-off timer starts]
                                       [further 0xAA discarded during timer]
                                       ← ANNOUNCE (0x00, SEQ=0x0)
  [valid HDLC frame → link up]
  [check protocol_version]
  [SEQ/ACKSEQ reset to 0x0]
  → first command
                                       [first valid HDLC frame received]
                                       [TX enabled, SEQ/ACKSEQ = 0x0]
```

### 3.3 Hold-off Timer

After transmitting ANNOUNCE the device starts a **40 ms hold-off timer** during which further 0xAA bytes are discarded. If a valid HDLC frame arrives during the hold-off window the device exits sync immediately. If the timer expires with no valid HDLC frame received the device resets its counter and returns to SYNC_HUNT.

### 3.4 Host Recovery

If the host does not receive a valid HDLC frame within **20 ms** of sending the sync pattern:

1. Send 1 × 0xAA (top-up byte)
2. Wait 20 ms
3. Repeat until a valid HDLC frame is received or **10 seconds** total have elapsed
4. On timeout: log error, enter application-defined degraded mode

### 3.5 Handshake States

| State | Condition | Device action |
|---|---|---|
| SYNC_HUNT | n < 16 consecutive 0xAA | Discard all RX, TX Hi-Z |
| SYNC_ACK | n ≥ 16, hold-off running | TX ANNOUNCE (0x00) once, discard 0xAA |
| SYNC_WAIT | Hold-off expired, no HDLC | Reset counter, return to SYNC_HUNT |
| LINKED | Valid HDLC frame received | Normal protocol, TX enabled |

### 3.6 Session Termination

The host may drop from LINKED to SYNC_HUNT at any time by ceasing to send valid frames. The device drops to SYNC_HUNT after a configurable host-silence timeout (application-defined; a recommended default is 30 seconds). On re-entering SYNC_HUNT the device holds TX Hi-Z and waits for a new sync pattern.

---

## 4. Message Types

### 4.1 Type Namespace

The TYPE field is 6 bits giving 64 possible values (0x00–0x3F).

| Range | Direction | Category |
|---|---|---|
| 0x00 | Device → Host | ANNOUNCE — mandatory, sent as handshake ACK |
| 0x01–0x2F | Either | Application-defined |
| 0x30 | Either | T_ACK |
| 0x31 | Either | T_NAK |
| 0x32 | Either | T_RESET |
| 0x33 | Host → Device | T_STATS_REQ — request link diagnostics |
| 0x34 | Device → Host | T_STATS_RSP — link diagnostics response |
| 0x35–0x3B | — | Reserved for future transport extensions |
| 0x3C | Device → Host | UNSUPPORTED |
| 0x3D–0x3F | — | Reserved |

Applications built on SDP assign message types within the range 0x01–0x2F. They may also repurpose 0x35–0x3B for application-specific use provided no future SDP revision has claimed those values; applications should note any such usage clearly in their own specification.

### 4.2 ANNOUNCE — Handshake Acknowledgement

**Type:** 0x00 | **Direction:** Device → Host | **Payload:** 3–255 bytes

Transmitted by the device as the handshake acknowledgement — the first frame sent after 16 consecutive 0xAA bytes are received. Sent once per link session without being requested.

**Mandatory bytes:**

| Byte | Field | Type | Description |
|---|---|---|---|
| 0 | protocol_version | uint8 | SDP protocol version implemented by this device. Current version: **1**. |
| 1 | window | uint8 | The device's `SDP_WINDOW` value (1–7). The host must not send more than this many unacknowledged frames. |
| 2 | max_payload | uint8 | The device's `SDP_MAX_PAYLOAD` value (1–255). The host must not send frames with a payload exceeding this length. |

**Bytes 3–(SDP_MAX_PAYLOAD-1):** Application-defined. The application may use these bytes for any purpose — device identity, firmware version, capability flags, serial number, or any other data relevant to the link session. SDP places no constraints on their content or interpretation.

The host must check `protocol_version`, `window`, and `max_payload` before issuing any commands. If the version is unsupported, or if the host cannot honour the device's window and payload constraints, the host should close the session.

### 4.3 UNSUPPORTED — Unrecognised Message Type

**Type:** 0x3C | **Direction:** Device → Host | **Payload:** 1 byte

Sent by the device when it receives a valid, well-formed frame whose TYPE value it does not implement. Distinct from T_NAK, which indicates a transport-layer failure. UNSUPPORTED means the frame was delivered correctly but the message type is not handled.

| Byte | Field | Type | Description |
|---|---|---|---|
| 0 | bad_type | uint8 | The TYPE byte from the unrecognised frame. |

The host should use UNSUPPORTED responses to detect version mismatches at runtime and avoid sending the unsupported type again for the duration of the session.

### 4.4 T_ACK — Explicit Acknowledgement

**Type:** 0x30 | **Direction:** Either | **Payload:** none

Sent when no application data is available to piggyback ACKSEQ within 50 ms of receiving a frame. The ACKSEQ nibble in the header carries the SEQ being acknowledged.

### 4.5 T_NAK — Negative Acknowledgement

**Type:** 0x31 | **Direction:** Either | **Payload:** 1 byte

Sent on CRC validation failure. The sender may retransmit with the RTX flag set.

| Byte | Field | Type | Description |
|---|---|---|---|
| 0 | bad_seq_ack | uint8 | The raw SEQ_ACKSEQ byte from the rejected frame, for diagnostics. |

### 4.6 T_RESET — Session Reset

**Type:** 0x32 | **Direction:** Either | **Payload:** none

Resets sequence counters to 0x0 and flushes transmit buffers on both sides. The receiving side resets its own counters and responds with T_RESET to confirm. Use for mid-session recovery only — for power-on link establishment use the startup handshake (Section 3).

### 4.7 T_STATS_REQ — Link Diagnostics Request

**Type:** 0x33 | **Direction:** Host → Device | **Payload:** none

Requests the device's current link diagnostic counters. The device responds with a T_STATS_RSP frame. The host may send T_STATS_REQ at any time during a linked session.

### 4.8 T_STATS_RSP — Link Diagnostics Response

**Type:** 0x34 | **Direction:** Device → Host | **Payload:** 36 bytes

Sent by the device in response to T_STATS_REQ. Carries the device-side diagnostic counters so the host can observe both ends of the link in a single exchange.

| Bytes | Field | Type | Description |
|---|---|---|---|
| 0–3 | rx_frames | uint32 le | Frames received and CRC-validated |
| 4–7 | tx_frames | uint32 le | Frames transmitted (including retransmits) |
| 8–11 | crc_errors | uint32 le | Frames rejected due to CRC failure |
| 12–15 | rx_overflows | uint32 le | Frames dropped due to RX buffer overflow |
| 16–19 | drop_count | uint32 le | SEQ gaps detected (lost frames) |
| 20–23 | nak_sent | uint32 le | T_NAK frames sent |
| 24–27 | nak_received | uint32 le | T_NAK frames received |
| 28–31 | window_full | uint32 le | Send attempts rejected due to window full |
| 32–35 | hal_errors | uint32 le | HAL write errors |

All fields are 32-bit unsigned little-endian integers. The counters are cumulative since the last `sdp_init` or `sdp_reset`. The host may diff successive responses to compute rates.

---

## 5. Implementation

The reference implementation is `src/sdp.h` — a single-header library using the STB pattern (see Section 1.2). The full source is the authoritative implementation reference; this section documents the HAL interface and provides platform examples.

### 5.1 HAL Interface

The HAL is a struct of three function pointers plus a `user` context pointer forwarded to every call:

```c
typedef struct {
    int      (*write)(const uint8_t *buf, uint8_t len, void *user);
    int      (*read)(void *user);
    uint32_t (*millis)(void *user);
    void     *user;
} sdp_hal_t;
```

- **`write`** — write `len` bytes to the transport; return 0 on success, -1 on error
- **`read`** — return the next received byte (0–255) or -1 if none available (non-blocking)
- **`millis`** — return milliseconds since boot; allowed to wrap at 2³²
- **`hal->write`** and **`hal->millis`** must not be NULL; `hal->read` may be NULL for ISR-driven RX (use `sdp_rx_byte` directly instead of `sdp_poll`)

### 5.2 Platform HAL Examples

**x86 / libserialport:**
```c
#include <libserialport.h>
#include <time.h>
static struct sp_port *_port;

static int hal_write(const uint8_t *buf, uint8_t len, void *user) {
    (void)user;
    return sp_blocking_write(_port, buf, len, 100) == len ? 0 : -1;
}
static int hal_read(void *user) {
    (void)user;
    uint8_t b;
    return sp_nonblocking_read(_port, &b, 1) == 1 ? b : -1;
}
static uint32_t hal_millis(void *user) {
    (void)user;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}
static const sdp_hal_t hal = { hal_write, hal_read, hal_millis, NULL };
```

**AVR ATtiny1616 (polled UART, PIT millis):**
```c
#include <avr/io.h>
static volatile uint32_t g_millis;
ISR(RTC_PIT_vect) { RTC.PITINTFLAGS = RTC_PI_bm; g_millis++; }

static int hal_write(const uint8_t *buf, uint8_t len, void *user) {
    (void)user;
    for (uint8_t i = 0; i < len; i++) {
        while (!(USART0.STATUS & USART_DREIF_bm));
        USART0.TXDATAL = buf[i];
    }
    return 0;
}
static int hal_read(void *user) {
    (void)user;
    if (!(USART0.STATUS & USART_RXCIF_bm)) return -1;
    return USART0.RXDATAL;
}
static uint32_t hal_millis(void *user) { (void)user; return g_millis; }
static const sdp_hal_t hal = { hal_write, hal_read, hal_millis, NULL };
```

**ARM Cortex-M (CMSIS, polled):**
```c
#include "stm32xx_hal.h"
extern UART_HandleTypeDef huart1;
static int hal_write(const uint8_t *buf, uint8_t len, void *user) {
    (void)user;
    return HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, HAL_MAX_DELAY) == HAL_OK ? 0 : -1;
}
static int hal_read(void *user) {
    (void)user;
    uint8_t b;
    return HAL_UART_Receive(&huart1, &b, 1, 0) == HAL_OK ? b : -1;
}
static uint32_t hal_millis(void *user) { (void)user; return HAL_GetTick(); }
static const sdp_hal_t hal = { hal_write, hal_read, hal_millis, NULL };
```

---

## 6. Performance Reference

### 6.1 Frame Overhead and Efficiency

Every SDP frame carries 7 bytes of overhead (2 FLAGS + 3 header + 2 CRC) before byte stuffing. Efficiency at a given payload size (before stuffing) is:

```
efficiency = payload / (payload + 7)
```

| Payload (bytes) | Efficiency | Typical use |
|---|---|---|
| 1 | 12.5% | Single status byte |
| 8 | 53% | Short command |
| 32 | 82% | Small data packet |
| 128 | 95% | Medium block |
| 255 | 97% | Maximum (default) |

Byte stuffing adds at most one extra byte per stuffed byte. In the worst case (all payload bytes are 0x7E or 0x7D) frame size doubles. In practice, stuffing overhead is negligible for arbitrary binary data.

### 6.2 Latency at Common Serial Rates

One-way frame latency = time to transmit the frame at the line rate. The table below uses the maximum (worst-case) frame size at each `SDP_MAX_PAYLOAD` setting, assuming no byte stuffing, one start bit, eight data bits, one stop bit (10 bits/byte).

| Baud rate | 255 B payload (262 B frame) | 128 B payload (135 B frame) | 32 B payload (39 B frame) |
|---|---|---|---|
| 9 600 | 274 ms | 141 ms | 41 ms |
| 19 200 | 137 ms | 70 ms | 20 ms |
| 115 200 | 22.8 ms | 11.7 ms | 3.4 ms |
| 460 800 | 5.7 ms | 2.9 ms | 0.8 ms |
| 1 000 000 | 2.6 ms | 1.35 ms | 0.39 ms |
| 3 000 000 | 0.87 ms | 0.45 ms | 0.13 ms |

Round-trip latency (command + response, minimum two frames in flight) is approximately 2× the one-way values. The 50 ms T_ACK timer (Section 2.7) adds up to 50 ms to the effective round trip if no piggybacked ACK is available, so applications should aim to reply within the ACK window.

### 6.3 Throughput at Common Serial Rates

Maximum sustained throughput uses the sliding window to keep the link full. With `SDP_WINDOW` frames in flight and frame period T:

```
throughput ≈ (SDP_WINDOW × payload) / T_round_trip
```

For a 115 200 baud link, 255 B payload, SDP_WINDOW = 2, round-trip ≈ 46 ms (two max frames):

```
throughput ≈ (2 × 255) / 0.046 ≈ 11 kB/s  (≈ 88 kbit/s payload, ~76% of line rate)
```

Increasing `SDP_MAX_PAYLOAD` to 255 and `SDP_WINDOW` to 7 at 115 200 baud approaches the practical line-rate ceiling for bulk transfers. At 1 Mbit/s and above the window is typically large enough that throughput is limited by the application processing rate rather than the protocol overhead.

---

## 7. Testing Specification

These tests verify the SDP transport layer in isolation. An application protocol built on SDP should add its own test suite for application message types.

Tests are written from the perspective of a host connected to a device under test. Each test specifies what to send, what to expect, and what a failure indicates.

### 6.1 Startup Handshake

**TEST-SYNC-01 — Optimistic path**
Send 17 × 0xAA. Expect a valid HDLC ANNOUNCE frame (TYPE=0x00) within 200 ms. Confirm payload length ≥ 1 and `protocol_version` (byte 0) is non-zero. Confirms basic handshake path and that the device correctly detects 16 consecutive bytes.

**TEST-SYNC-02 — Partial sync rejected**
Send 15 × 0xAA then stop. Wait 500 ms. Expect no HDLC response. Then send 17 × 0xAA. Expect ANNOUNCE within 200 ms. Confirms the counter requires exactly 16 consecutive bytes.

**TEST-SYNC-03 — Noise before sync**
Send 8 × 0xAA, then 1 × 0x00 (interrupting the run), then 17 × 0xAA. Expect ANNOUNCE within 200 ms. Confirms counter resets on non-0xAA byte.

**TEST-SYNC-04 — Top-up path**
Send 15 × 0xAA with a 25 ms gap, then 1 more 0xAA. Expect ANNOUNCE. Confirms an interrupted run is completed by a top-up byte.

**TEST-SYNC-05 — Hold-off window**
After completing the handshake, immediately send 17 × 0xAA again within the 40 ms hold-off window. Expect no second ANNOUNCE. Confirms hold-off timer prevents re-triggering.

**TEST-SYNC-06 — Re-sync after hold-off expires**
After completing the handshake, wait 50 ms without sending any HDLC frame. Then send 17 × 0xAA. Expect ANNOUNCE. Confirms SYNC_WAIT → SYNC_HUNT transition.

**TEST-SYNC-07 — ANNOUNCE protocol_version check**
In ANNOUNCE received during TEST-SYNC-01, confirm byte 0 matches the expected SDP protocol version. Confirms version negotiation is possible.

### 6.2 Frame Framing

**TEST-FRAME-01 — Minimum valid frame**
Send a T_ACK frame (TYPE=0x30, LEN=0, no payload). Expect the device to accept it silently (ACKSEQ updates). Confirms zero-payload frame is valid.

**TEST-FRAME-02 — Maximum valid frame**
Send a frame with TYPE in the application range, LEN=0x80 (128 bytes), payload all 0x55. Expect it to be dispatched without error. Confirms maximum payload accepted.

**TEST-FRAME-03 — Oversized LEN rejected**
Send a frame with LEN=0x81 (129 bytes). Expect no response and a subsequent valid frame to be processed normally (parser returned to HUNT). Confirms LEN > 0x80 causes rejection without desync.

**TEST-FRAME-04 — Truncated frame**
Send an opening FLAG, 3 header bytes, then stop without a closing FLAG. Wait 200 ms. Send a valid T_ACK. Expect it to be processed (ACKSEQ updates). Confirms the parser correctly abandons a partial frame on the next opening FLAG.

**TEST-FRAME-05 — Consecutive frames share FLAG**
Send two back-to-back T_ACK frames sharing a single FLAG byte between them. Confirm both are processed. Confirms back-to-back frame sharing is handled.

### 6.3 Byte Stuffing

**TEST-STUFF-01 — 0x7E in payload**
Send a frame with a payload byte of 0x7E. Confirm the wire representation contains 0x7D 0x5E. Confirm the device receives the payload correctly.

**TEST-STUFF-02 — 0x7D in payload**
Send a frame with a payload byte of 0x7D. Confirm escape sequence 0x7D 0x5D on wire. Confirm correct receipt.

**TEST-STUFF-03 — 0x7E in CRC**
Construct a frame whose CRC high or low byte is 0x7E. Verify the wire representation escapes it. Verify the device accepts the frame.

**TEST-STUFF-04 — 0x7D in CRC**
As above for CRC byte value 0x7D.

**TEST-STUFF-05 — Unescaped 0x7E mid-payload**
Inject a raw 0x7E byte into the middle of a payload without escaping it. The parser interprets it as a closing FLAG, finds a malformed frame, and returns to HUNT. Expect T_NAK or no response. Send a valid frame immediately after and expect it to be processed. Confirms parser recovery.

### 6.4 CRC Integrity

**TEST-CRC-01 — Single bit flip in header**
Flip one bit in the SEQ_ACKSEQ byte of a valid frame. Expect T_NAK (TYPE=0x31) with the bad_seq_ack byte reported, and no application-layer response.

**TEST-CRC-02 — Single bit flip in payload**
Flip one bit in the middle of a payload. Expect T_NAK, no application dispatch.

**TEST-CRC-03 — Corrupted CRC bytes**
Send a valid frame with both CRC bytes zeroed. Expect T_NAK.

**TEST-CRC-04 — Recovery after CRC failure**
After a CRC failure (TEST-CRC-01), immediately send a valid T_ACK. Expect the device to process it. Confirms clean recovery.

### 6.5 Sequence Numbers

**TEST-SEQ-01 — Normal incrementing sequence**
Send five consecutive frames with SEQ 0x0 through 0x4. Confirm ACKSEQ in each device response reflects the last received SEQ correctly.

**TEST-SEQ-02 — Duplicate SEQ discarded**
Send a valid frame with SEQ=0x3 and RTX flag set, matching a previously received SEQ. Expect no duplicate application dispatch (T_ACK only). Confirms duplicate detection.

**TEST-SEQ-03 — SEQ wraparound**
Send frames with SEQ incrementing from 0xC through 0xE then back to 0x0, 0x1. Confirm ACKSEQ tracks correctly through the wraparound. Confirm SEQ value 0xF is never used as a transmit SEQ by the device.

**TEST-SEQ-04 — Startup sentinel in ANNOUNCE**
In the ANNOUNCE frame sent as the startup handshake ACK, confirm ACKSEQ nibble = 0xF (no frame received yet by the device).

### 6.6 Flow Control

**TEST-FLOW-01 — Window respected**
Send `SDP_WINDOW + 1` frames in rapid succession without waiting for acknowledgement. Confirm the first `SDP_WINDOW` are processed. The `SDP_WINDOW + 1`th must be held until a slot is freed. Confirms window enforcement.

**TEST-FLOW-02 — T_ACK within 50 ms**
Send a valid frame and withhold any outgoing frames for 60 ms. Confirm the device sends T_ACK within 50 ms of receiving the frame. Confirms the explicit ACK timer.

**TEST-FLOW-03 — Window recovery**
Fill all window slots, wait for T_ACK, confirm the window opens and a further frame is now accepted.

**TEST-FLOW-04 — SDP_WINDOW compile check**
Attempt to compile with SDP_WINDOW=8. Confirm a compile-time error is emitted. Confirms the static assertion is present and correct.

---

## 7. Revision History

| Revision | Date | Changes |
|---|---|---|
| 1.2 | 2026-03-14 | Converted to single-header library (STB pattern). `sdp.c` and `sdp_config.h` merged into `sdp.h` behind `#ifdef SDP_IMPLEMENTATION`. Section 1.2 updated (Shared Library Rule). Section 2.6 configuration snippet updated. Section 5 replaced with single-header usage, HAL interface description, and updated platform examples matching `sdp_hal_t` function-pointer struct. 19 200 baud row added to performance table. |
| 1.1 | 2026-03-13 | TX retransmit buffer: sender stores each transmitted frame in `tx_ring[seq % SDP_WINDOW]`; T_NAK triggers retransmit from ring with RTX flag (Section 2.9). SDP_SEQ_SIZE/SDP_SEQ_MAX derived from SDP_WINDOW as `(15/SDP_WINDOW)*SDP_WINDOW`, guaranteed ≥ 2× window (Section 2.5). SDP_MAX_PAYLOAD promoted to compile-time parameter (default 255, range 3–255); SDP_SLOT_SIZE derived as SDP_MAX_PAYLOAD+5. ANNOUNCE bytes 1–2 now mandatory transport fields (window, max_payload). T_STATS_REQ (0x33) / T_STATS_RSP (0x34) added for host-side link diagnostics (Section 4.7–4.8). Performance reference section added (Section 6). |
| 1.0 | 2026-03-13 | Initial release as standalone library specification. Transport layer, startup handshake, ANNOUNCE (protocol_version mandatory; remaining bytes application-defined), UNSUPPORTED (0x3C), T_ACK/T_NAK/T_RESET. Maximum payload 128 bytes (LEN 0x00–0x80). SDP_WINDOW compile-time constant defaulting to 2, static assert 1–7. Platform HAL examples: x86/libserialport, AVR, ARM Cortex-M. |
