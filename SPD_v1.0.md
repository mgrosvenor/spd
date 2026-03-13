# Serial Datagram Protocol (SDP) — Specification

*Revision 1.0 — 2026-03-13*

---

## Table of Contents

1. [Overview](#1-overview)
2. [Transport Layer](#2-transport-layer)
3. [Startup Handshake](#3-startup-handshake)
4. [Message Types](#4-message-types)
5. [Sample Implementation](#5-sample-implementation)
6. [Testing Specification](#6-testing-specification)
7. [Revision History](#7-revision-history)

---

## 1. Overview

SDP is a lightweight, framed, sequenced, and flow-controlled datagram protocol for use over any reliable byte-stream transport (UART, USB CDC, TCP socket, named pipe). It is designed for embedded systems communicating with a host process, but is implementable on any platform.

SDP provides:
- HDLC-inspired framing with CRC-16 integrity checking
- 4-bit sequence numbers with piggybacked acknowledgement
- Configurable sliding window flow control
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

### 1.2 Shared Library Rule

The SDP transport implementation (`sdp.c` / `sdp.h`) must have zero dependency on application state. It communicates with the rest of the system only through the HAL interface (Section 5.1) and its own `sdp_ctx_t` context struct. Any application built on SDP — including a bootloader running on the same device — links the same unmodified `sdp.c`.

---

## 2. Transport Layer

### 2.1 Framing

Framing follows HDLC (ISO 13239). Every frame is delimited by the flag byte 0x7E at both start and end. The 4-byte header packs SEQ and ACKSEQ into nibbles and TYPE and FLAGS into a single byte.

```
┌────────┬──────────────────┬──────────────────┬────────┬───────────────┬─────────┬────────┐
│  FLAG  │  SEQ[3:0]        │  TYPE[5:0]        │  LEN   │    PAYLOAD    │  CRC16  │  FLAG  │
│  0x7E  │  ACKSEQ[3:0]     │  FLAGS[1:0]       │ 0–0x80 │  0–128 bytes  │  2 bytes│  0x7E  │
└────────┴──────────────────┴──────────────────┴────────┴───────────────┴─────────┴────────┘
   1 B           1 B                 1 B           1 B       N bytes       2 bytes    1 B
```

- **Minimum frame on wire:** 8 bytes (zero-length payload)
- **Maximum frame on wire:** 135 bytes (128-byte payload + 7 overhead)
- **Buffer slot (FLAG bytes stripped):** 132 bytes max

### 2.2 Field Definitions

| Field | Size | Range | Description |
|---|---|---|---|
| FLAG | 1 B | 0x7E | HDLC frame delimiter. Must be escaped if it appears in the frame body — see Section 2.3. |
| SEQ / ACKSEQ | 1 B | nibbles 0x0–0xF | High nibble: sender SEQ (0–14, wraps 14→0; see Section 2.5). Low nibble: piggybacked ACK — SEQ of last valid frame received from the other side. 0xF = startup sentinel (no frame received yet). |
| TYPE / FLAGS | 1 B | TYPE 0x00–0x3F | High 6 bits: message type (0–63). Low 2 bits: FLAGS. Bit 1 = RTX (this frame is a retransmit). Bit 0 = reserved, always transmit 0. |
| LEN | 1 B | 0x00–0x80 | Payload length in bytes (0–128), measured after unescaping. Values 0x81–0xFF are reserved and must cause the frame to be rejected. |
| PAYLOAD | N B | 0x00–0xFF | Application data. Content defined by TYPE. May be empty (LEN=0). |
| CRC16 | 2 B | 0x0000–0xFFFF | CRC-16/CCITT-FALSE over SEQ_ACKSEQ + TYPE_FLAGS + LEN + PAYLOAD (pre-escape values). Big-endian, high byte first. |

### 2.3 Byte Stuffing (HDLC Transparency)

The flag byte 0x7E and escape byte 0x7D must not appear raw in the frame body (SEQ_ACKSEQ through CRC16 inclusive). If either value occurs it is replaced by a two-byte escape sequence on transmit and restored on receipt.

| Original byte | Transmitted as | Notes |
|---|---|---|
| 0x7E | 0x7D  0x5E | Flag byte escaped |
| 0x7D | 0x7D  0x5D | Escape byte self-escaped |

CRC16 is computed on the original pre-escape bytes. Escaping is applied after CRC calculation on transmit and reversed before CRC verification on receipt.

> **NOTE** Buffer slots are fixed at 132 bytes (header + max payload + CRC, FLAGS stripped). Frames with unescaped LEN > 0x80 must be rejected and the receiver returned to HUNT state.

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

Each side maintains an independent 4-bit transmit sequence counter initialised to 0x0 at power-on and on every successful link establishment. The counter increments after each transmitted frame and wraps through values 0x0–0xE only — value 0xF is permanently reserved as the startup sentinel and is never used as a transmit SEQ.

SEQ occupies the high nibble of byte 1. ACKSEQ occupies the low nibble. ACKSEQ carries the SEQ of the last frame successfully received and CRC-validated from the other side. The startup sentinel value 0xF in ACKSEQ indicates no frame has been received yet.

Sequence numbers serve two purposes:

1. **Duplicate detection** — a frame carrying a SEQ value already processed (RTX flag set) is discarded without being dispatched to the application layer.
2. **Loss detection** — a gap in received SEQ values indicates one or more dropped frames. The receiver increments a drop counter but does not automatically request retransmission; that is an application-layer decision.

### 2.6 Flow Control

Window size is controlled by the compile-time constant `SDP_WINDOW`, defaulting to 2.

```c
/* sdp_config.h */
#ifndef SDP_WINDOW
#  define SDP_WINDOW 2
#endif

#if SDP_WINDOW < 1 || SDP_WINDOW > 7
#  error "SDP_WINDOW must be in range 1–7 \
(usable sequence space is 15; window must be <= floor(15/2))"
#endif
```

The constraint `SDP_WINDOW <= 7` is required for correct duplicate detection: the usable sequence space is 15 (values 0x0–0xE; 0xF reserved), and the window must not exceed half the sequence space, otherwise the receiver cannot distinguish a new frame from a retransmit when the window is fully in flight.

The sender may have at most `SDP_WINDOW` unacknowledged frames in flight at any time. A frame is acknowledged when the receiver's outgoing ACKSEQ nibble equals or advances past that frame's SEQ value. If all slots are occupied the sender must wait before transmitting another frame.

```
Each side:  SDP_WINDOW receive slots × 132 bytes = SDP_WINDOW × 132 bytes
```

> **NOTE** Implementations may use receive buffers larger than `SDP_WINDOW` slots but must never send more than `SDP_WINDOW` unacknowledged frames to a peer that declares support for only `SDP_WINDOW` slots. The ANNOUNCE payload may be used by applications to advertise the window size if negotiation is needed; SDP itself does not negotiate this.

### 2.7 Explicit Acknowledgement Timing

When a side receives a valid frame and has no application data to transmit within **50 ms**, it must send a T_ACK frame (0x30) to prevent the sender's window from stalling. This 50 ms threshold applies to both sides.

### 2.8 Receiver State Machine

```
HUNT        — discard bytes until 0x7E received
  ↓ 0x7E
IN_FRAME    — accumulate bytes into 132-byte slot (FLAG bytes not stored)
  ↓ 0x7D    → ESCAPED (next byte XOR 0x20, then back to IN_FRAME)
  ↓ 0x7E    → end of frame: validate LEN ≤ 0x80, verify CRC16
               PASS → dispatch to message handler, update ACKSEQ nibble
               FAIL → discard frame, increment error counter, back to HUNT
  ↓ overflow → discard frame, back to HUNT
```

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
| 0x33–0x3B | — | Reserved for future transport extensions |
| 0x3C | Device → Host | UNSUPPORTED |
| 0x3D–0x3F | — | Reserved |

Applications built on SDP assign message types within the range 0x01–0x2F. They may also repurpose 0x33–0x3B for application-specific use provided no future SDP revision has claimed those values; applications should note any such usage clearly in their own specification.

### 4.2 ANNOUNCE — Handshake Acknowledgement

**Type:** 0x00 | **Direction:** Device → Host | **Payload:** 1–128 bytes

Transmitted by the device as the handshake acknowledgement — the first frame sent after 16 consecutive 0xAA bytes are received. Sent once per link session without being requested.

**Mandatory byte:**

| Byte | Field | Type | Description |
|---|---|---|---|
| 0 | protocol_version | uint8 | SDP protocol version implemented by this device. Current version: **1**. Must be the first byte. |

**Bytes 1–127:** Application-defined. The application may use these bytes for any purpose — device identity, firmware version, capability flags, serial number, or any other data relevant to the link session. SDP places no constraints on their content or interpretation. The payload may be as short as 1 byte (protocol_version only) if the application has nothing further to announce.

The host must check `protocol_version` before issuing any commands. If the version is unsupported the host should close the session.

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

---

## 5. Sample Implementation

This section provides a portable C implementation of the SDP transport layer. It compiles without modification on any platform. The caller provides a thin HAL of three functions. All application message type definitions belong in the application layer and are not present here.

### 5.1 HAL Interface

```c
/* sdp_hal.h — implement these three functions for your platform */
#ifndef SDP_HAL_H
#define SDP_HAL_H

#include <stdint.h>

/* Write len bytes to the transport. Blocks until all bytes are queued.   */
void     sdp_hal_write(const uint8_t *buf, uint8_t len);

/* Return next received byte, or -1 if none available (non-blocking).    */
int      sdp_hal_read(void);

/* Return milliseconds since boot (wraps at 2^32).                       */
uint32_t sdp_hal_millis(void);

#endif /* SDP_HAL_H */
```

### 5.2 Configuration Header

```c
/* sdp_config.h — compile-time tuning */
#ifndef SDP_CONFIG_H
#define SDP_CONFIG_H

/* Sliding window size. Default 2. Must be 1–7.                          */
#ifndef SDP_WINDOW
#  define SDP_WINDOW 2
#endif

#if SDP_WINDOW < 1 || SDP_WINDOW > 7
#  error "SDP_WINDOW must be in range 1–7 \
(usable sequence space is 15; window must be <= floor(15/2))"
#endif

#endif /* SDP_CONFIG_H */
```

### 5.3 Protocol Core

```c
/* sdp.h */
#ifndef SDP_H
#define SDP_H

#include <stdint.h>
#include <stdbool.h>
#include "sdp_config.h"

/* ── Wire constants ─────────────────────────────────────────────────────── */
#define SDP_FLAG_BYTE     0x7E
#define SDP_ESC_BYTE      0x7D
#define SDP_MAX_PAYLOAD   128
#define SDP_SLOT_SIZE     132  /* header(3) + payload(128) + crc(2) - FLAGs */
#define SDP_SEQ_NONE      0xF  /* startup sentinel in ACKSEQ nibble          */
#define SDP_SEQ_MAX       0xE  /* highest usable SEQ value (0xF reserved)    */

/* ── Transport message types (application types defined elsewhere) ───────── */
#define SDP_T_ANNOUNCE    0x00  /* handshake ACK — protocol_version in byte 0 */
#define SDP_T_ACK         0x30
#define SDP_T_NAK         0x31
#define SDP_T_RESET       0x32
#define SDP_T_UNSUPPORTED 0x3C

/* ── FLAGS bits ─────────────────────────────────────────────────────────── */
#define SDP_FLAG_RTX      0x02  /* bit 1: this frame is a retransmit          */

/* ── Frame ──────────────────────────────────────────────────────────────── */
typedef struct {
    uint8_t seq;
    uint8_t ackseq;
    uint8_t type;
    uint8_t flags;
    uint8_t len;
    uint8_t payload[SDP_MAX_PAYLOAD];
} sdp_frame_t;

/* ── Context ────────────────────────────────────────────────────────────── */
typedef struct {
    /* TX state */
    uint8_t  tx_seq;
    uint8_t  rx_ackseq;

    /* RX ring */
    sdp_frame_t rx_ring[SDP_WINDOW];
    uint8_t     rx_head;
    uint8_t     rx_tail;
    uint8_t     rx_count;

    /* RX parser */
    uint8_t  rx_buf[SDP_SLOT_SIZE];
    uint8_t  rx_pos;
    bool     rx_escaped;
    bool     rx_in_frame;

    /* Diagnostics */
    uint32_t crc_errors;
    uint32_t drop_count;
    uint32_t last_ack_ms;
} sdp_ctx_t;

/* ── API ────────────────────────────────────────────────────────────────── */

/* Initialise context. Call once before use or after T_RESET.             */
void sdp_init(sdp_ctx_t *ctx);

/* Feed one received byte into the parser.                                */
/* Returns true if a complete validated frame is now in the RX ring.      */
bool sdp_rx_byte(sdp_ctx_t *ctx, uint8_t byte);

/* Pop the oldest frame from the RX ring. Returns false if ring empty.    */
bool sdp_rx_pop(sdp_ctx_t *ctx, sdp_frame_t *out);

/* Build and transmit a frame. Returns false if window is full.           */
bool sdp_send(sdp_ctx_t *ctx, uint8_t type, uint8_t flags,
              const uint8_t *payload, uint8_t len);

/* Call periodically. Sends T_ACK if 50 ms has elapsed since last         */
/* outgoing piggyback. Safe to call from a main loop or timer ISR.        */
void sdp_tick(sdp_ctx_t *ctx);

#endif /* SDP_H */
```

```c
/* sdp.c */
#include "sdp.h"
#include "sdp_hal.h"
#include <string.h>

static uint16_t crc16(const uint8_t *data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

static void write_byte(uint8_t b) {
    if (b == SDP_FLAG_BYTE || b == SDP_ESC_BYTE) {
        uint8_t esc[2] = { SDP_ESC_BYTE, b ^ 0x20 };
        sdp_hal_write(esc, 2);
    } else {
        sdp_hal_write(&b, 1);
    }
}

void sdp_init(sdp_ctx_t *ctx) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->rx_ackseq   = SDP_SEQ_NONE;
    ctx->last_ack_ms = sdp_hal_millis();
}

bool sdp_send(sdp_ctx_t *ctx, uint8_t type, uint8_t flags,
              const uint8_t *payload, uint8_t len)
{
    if (len > SDP_MAX_PAYLOAD) return false;

    uint8_t seq_ack  = (uint8_t)((ctx->tx_seq << 4) | (ctx->rx_ackseq & 0x0F));
    uint8_t type_flg = (uint8_t)((type << 2) | (flags & 0x03));

    /* CRC over pre-escape bytes */
    uint8_t  hdr[3]  = { seq_ack, type_flg, len };
    uint16_t crc     = crc16(hdr, 3);
    for (uint8_t i = 0; i < len; i++) {
        uint8_t b = payload[i];
        crc ^= (uint16_t)b << 8;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }

    uint8_t flag = SDP_FLAG_BYTE;
    sdp_hal_write(&flag, 1);
    write_byte(seq_ack);
    write_byte(type_flg);
    write_byte(len);
    for (uint8_t i = 0; i < len; i++) write_byte(payload[i]);
    write_byte((uint8_t)(crc >> 8));
    write_byte((uint8_t)(crc & 0xFF));
    sdp_hal_write(&flag, 1);

    ctx->tx_seq = (ctx->tx_seq >= SDP_SEQ_MAX) ? 0 : ctx->tx_seq + 1;
    ctx->last_ack_ms = sdp_hal_millis();
    return true;
}

bool sdp_rx_byte(sdp_ctx_t *ctx, uint8_t byte) {
    if (byte == SDP_FLAG_BYTE) {
        if (ctx->rx_in_frame && ctx->rx_pos >= 5) {
            /* header(3) + crc(2) = 5 minimum */
            uint8_t  len      = ctx->rx_buf[2];
            uint16_t expected = ((uint16_t)ctx->rx_buf[ctx->rx_pos - 2] << 8)
                               | ctx->rx_buf[ctx->rx_pos - 1];
            uint16_t actual   = crc16(ctx->rx_buf, ctx->rx_pos - 2);

            if (len <= SDP_MAX_PAYLOAD
                && (ctx->rx_pos == (uint8_t)(len + 5))
                && actual == expected
                && ctx->rx_count < SDP_WINDOW)
            {
                sdp_frame_t *slot = &ctx->rx_ring[ctx->rx_tail];
                slot->seq    = ctx->rx_buf[0] >> 4;
                slot->ackseq = ctx->rx_buf[0] & 0x0F;
                slot->type   = ctx->rx_buf[1] >> 2;
                slot->flags  = ctx->rx_buf[1] & 0x03;
                slot->len    = len;
                memcpy(slot->payload, ctx->rx_buf + 3, len);
                ctx->rx_tail   = (ctx->rx_tail + 1) % SDP_WINDOW;
                ctx->rx_count++;
                ctx->rx_ackseq = slot->seq;
                ctx->last_ack_ms = sdp_hal_millis();
            } else {
                ctx->crc_errors++;
            }
        }
        ctx->rx_in_frame = true;
        ctx->rx_pos      = 0;
        ctx->rx_escaped  = false;
        return ctx->rx_count > 0;
    }

    if (!ctx->rx_in_frame) return false;

    if (byte == SDP_ESC_BYTE) { ctx->rx_escaped = true; return false; }
    if (ctx->rx_escaped)      { byte ^= 0x20; ctx->rx_escaped = false; }

    if (ctx->rx_pos < SDP_SLOT_SIZE)
        ctx->rx_buf[ctx->rx_pos++] = byte;
    else {
        ctx->rx_in_frame = false;
        ctx->rx_pos      = 0;
    }
    return false;
}

bool sdp_rx_pop(sdp_ctx_t *ctx, sdp_frame_t *out) {
    if (ctx->rx_count == 0) return false;
    *out = ctx->rx_ring[ctx->rx_head];
    ctx->rx_head  = (ctx->rx_head + 1) % SDP_WINDOW;
    ctx->rx_count--;
    return true;
}

void sdp_tick(sdp_ctx_t *ctx) {
    if ((sdp_hal_millis() - ctx->last_ack_ms) >= 50)
        sdp_send(ctx, SDP_T_ACK, 0, NULL, 0);
}
```

### 5.4 Startup Handshake Helper

```c
/* sdp_handshake.c — host-side handshake helper */
#include "sdp.h"
#include "sdp_hal.h"
#include <string.h>

#define SDP_SYNC_BYTE     0xAA
#define SDP_SYNC_COUNT    17
#define SDP_SYNC_TIMEOUT  10000   /* ms — total handshake budget          */
#define SDP_SYNC_RETRY_MS 20      /* ms — top-up interval                 */

/*
 * Run the host side of the SDP startup handshake.
 *
 * On success: ANNOUNCE frame is placed in out_announce, returns true.
 * The caller must verify out_announce->payload[0] (protocol_version).
 *
 * On timeout: returns false.
 */
bool sdp_handshake(sdp_ctx_t *ctx, sdp_frame_t *out_announce) {
    uint8_t sync[SDP_SYNC_COUNT];
    memset(sync, SDP_SYNC_BYTE, SDP_SYNC_COUNT);
    sdp_hal_write(sync, SDP_SYNC_COUNT);

    uint32_t start    = sdp_hal_millis();
    uint32_t last_top = sdp_hal_millis();

    while ((sdp_hal_millis() - start) < SDP_SYNC_TIMEOUT) {
        int b = sdp_hal_read();
        if (b >= 0 && sdp_rx_byte(ctx, (uint8_t)b)) {
            sdp_frame_t f;
            if (sdp_rx_pop(ctx, &f) && f.type == SDP_T_ANNOUNCE) {
                if (out_announce) *out_announce = f;
                return true;
            }
        }
        if ((sdp_hal_millis() - last_top) >= SDP_SYNC_RETRY_MS) {
            uint8_t top = SDP_SYNC_BYTE;
            sdp_hal_write(&top, 1);
            last_top = sdp_hal_millis();
        }
    }
    return false;
}
```

### 5.5 Platform HAL Examples

**x86 / libserialport:**
```c
#include <libserialport.h>
#include <time.h>
static struct sp_port *_port;

void sdp_hal_write(const uint8_t *buf, uint8_t len) {
    sp_blocking_write(_port, buf, len, 100);
}
int sdp_hal_read(void) {
    uint8_t b;
    return sp_nonblocking_read(_port, &b, 1) == 1 ? b : -1;
}
uint32_t sdp_hal_millis(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}
```

**AVR (ATtiny/ATmega, polled UART):**
```c
#include <avr/io.h>
void sdp_hal_write(const uint8_t *buf, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        while (!(USART0.STATUS & USART_DREIF_bm));
        USART0.TXDATAL = buf[i];
    }
}
int sdp_hal_read(void) {
    if (!(USART0.STATUS & USART_RXCIF_bm)) return -1;
    return USART0.RXDATAL;
}
uint32_t sdp_hal_millis(void) {
    extern volatile uint32_t g_millis_tick;  /* incremented by PIT ISR    */
    return g_millis_tick;
}
```

**ARM Cortex-M (CMSIS, polled):**
```c
#include "stm32xx_hal.h"
extern UART_HandleTypeDef huart1;
void sdp_hal_write(const uint8_t *buf, uint8_t len) {
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, HAL_MAX_DELAY);
}
int sdp_hal_read(void) {
    uint8_t b;
    return HAL_UART_Receive(&huart1, &b, 1, 0) == HAL_OK ? b : -1;
}
uint32_t sdp_hal_millis(void) { return HAL_GetTick(); }
```

---

## 6. Testing Specification

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
| 1.0 | 2026-03-13 | Initial release as standalone library specification, extracted from SmartDimmer Protocol Specification v1.4. Transport layer, startup handshake, ANNOUNCE (protocol_version mandatory; remaining bytes application-defined), UNSUPPORTED (0x3C), T_ACK/T_NAK/T_RESET. Maximum payload 128 bytes (LEN 0x00–0x80). SDP_WINDOW compile-time constant defaulting to 2, static assert 1–7. HAL renamed sdp_hal_write/read/millis. All application message types, bootloader protocol, and platform-specific content removed. Platform HAL examples: x86/libserialport, AVR, ARM Cortex-M. |
