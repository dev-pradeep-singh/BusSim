/*
 * LINSim_DUT_LIN — Arduino Nano + LINTTL3
 * LIN bus test harness for the BusSim Blue Pill's LIN side (lin_handler.c)
 *
 * What this sketch does (minimal test harness, not a full simulated network)
 * ────────────────────────────────────────────────────────────────────────
 *  • Drives a LIN master schedule for two test IDs:
 *      0x10  master-response frame, ~1 Hz, incrementing counter byte
 *            (self-contained — proves header+data generation works even
 *            with no other node on the bus)
 *      0x20  header-only frame, ~0.5 Hz — sent, then this sketch listens
 *            for a response with a short timeout. Use this to exercise
 *            the STM32 in SLAVE mode (its default boot mode — see
 *            CLAUDE.md — so LIN MODE SLAVE below is a no-op unless the
 *            STM32 was previously switched to MASTER):
 *              STM32> LIN MODE SLAVE
 *              STM32> LIN RESP 0x20 <bytes...>
 *            A valid response here proves the STM32 slave-response path.
 *            The response byte count must match what a listening bus
 *            analyzer expects (e.g. an LDF's declared frame length) or it
 *            may log a decode error even though the STM32 responded fine —
 *            see LINSim_DUT_LIN.ldf.
 *  • Passively watches the bus for a header on 0x30 that this sketch isn't
 *    itself driving, and auto-answers it with a fixed 2-byte payload (an
 *    incrementing byte + 0xA5). Use this to exercise the STM32 in MASTER
 *    mode:
 *              STM32> LIN MODE MASTER
 *              STM32> SET LINID 0x30
 *              STM32> LIN SEND 0x30 01 02          (or LIN ON)
 *            The STM32's own `LISTEN ON` output shows this sketch's reply.
 *
 *  LIN is strictly single-master — never have the STM32 in MASTER mode
 *  driving its own periodic schedule (or `LIN SEND`) for 0x10/0x20 at the
 *  same time this sketch is also driving them. The 0x30 auto-responder
 *  path is the only one meant to run concurrently with the STM32 sending.
 *
 * Protocol implementation
 * ────────────────────────
 *  There's no Arduino LIN library equivalent to arduino-mcp2515, so this
 *  sketch implements LIN 2.x framing from scratch — break, sync, PID
 *  (with parity bits), and checksum (enhanced, with the classic exception
 *  for diagnostic IDs 0x3C/0x3D) — mirroring the STM32 side's
 *  Core/Src/lin_handler.c protocol handling so the two interoperate
 *  correctly (the STM32 now generates/detects break via USART1's hardware
 *  LIN mode instead — see CLAUDE.md — this sketch still bit-bangs it since
 *  the ATmega328P has no equivalent hardware LIN mode). Break is generated
 *  by briefly disabling just the UART's transmitter (TXEN0) and driving the
 *  TX pin low as a plain GPIO — not a full Serial.end()/begin(), whose
 *  variable teardown/init cost was found to occasionally stretch the gap
 *  before the sync byte enough for a LIN analyzer to misclassify the break
 *  as a standalone wake-up pulse; break is detected on RX via the UART's
 *  framing-error flag (UCSR0A/FE0).
 *
 * Why this dedicates the hardware UART to LIN (no live Serial Monitor)
 * ─────────────────────────────────────────────────────────────────────
 *  The Nano has exactly one hardware UART, shared with the USB/Serial-
 *  Monitor link. Reliable break detection needs direct access to that same
 *  UART's framing-error flag — SoftwareSerial can't reliably tell a break
 *  apart from line noise. So after a short boot banner (visible if you
 *  have Serial Monitor open at 115200 baud right when the board resets),
 *  this sketch re-inits the UART at 19200 baud for the LIN bus exclusively
 *  and reports everything else via the onboard LED — see the blink-pattern
 *  table below. This mirrors BusSim's own PB8/PB9-vs-USB pin-conflict story,
 *  just on the Arduino side.
 *
 * Wiring — Arduino Nano ↔ LINTTL3 module
 * ───────────────────────────────────────
 *   LINTTL3   Arduino Nano
 *   TX     →  D0 (RX)             module's TTL output into the Nano's UART RX
 *   RX     ←  D1 (TX)             Nano's UART TX drives the module's TTL input
 *   GND    →  GND                 shared with the STM32 board's GND too
 *   VIN    →  external 12 V bench supply (NOT the Nano's 5 V rail)
 *   SLP    →  D7 (driven HIGH at boot for normal/awake mode)
 *   INH      leave unconnected — it's a transceiver output (drives an
 *            external regulator in real vehicle wiring), not a mode-select
 *            input; not needed on the bench
 *   LINTTL3 TX/RX are labeled from the module's own UART-transceiver
 *   perspective (confirmed against the vendor's product page), hence the
 *   cross-connect above — not straight TX-to-TX/RX-to-RX.
 *
 * LIN bus wiring
 * ──────────────
 *   LINTTL3 LIN pin  ←→  MCP2004 LBUS pin (STM32 side), same bus wire
 *   Master pull-up (1 kΩ + diode, LIN pin → VIN) needed on whichever node
 *   is acting as master for a given test — only one at a time in practice
 *   for this minimal harness, since only one master drives per test.
 *
 * Status LED (D13, onboard) — blink patterns
 * ───────────────────────────────────────────
 *   1 short blink   (30 ms)            — this sketch sent a frame (0x10/0x20/0x30 header)
 *   2 blinks        (80/80 ms)         — valid response received for 0x20 (good checksum)
 *   3 blinks        (60/60 ms)         — auto-responded to a 0x30 header from the STM32
 *   4 fast blinks   (40/40 ms)         — response received but checksum failed
 *   1 long blink    (400 ms)           — 0x20 query timed out, no response seen
 */

#include <string.h>

// ── Pins ──────────────────────────────────────────────────────────────
#define LIN_TX_PIN   1     // Nano D1 / hardware UART TX
#define SLP_PIN      7
#define STATUS_LED   13

// ── LIN bus parameters ──────────────────────────────────────────────────
static const uint32_t LIN_BAUD           = 19200UL;
static const uint16_t LIN_BREAK_US       = 750;   // >13 bit times at 19200 (677us) + margin
static const uint16_t LIN_BREAK_DELIM_US = 60;    // ~1 bit time high before sync
static const uint32_t QUERY_TIMEOUT_MS   = 20;    // response wait for 0x20

// ── Test IDs ─────────────────────────────────────────────────────────────
static const uint8_t ID_COUNTER      = 0x10;  // self-contained master-response frame
static const uint8_t ID_QUERY_SLAVE  = 0x20;  // exercises STM32 SLAVE mode
static const uint8_t ID_AUTO_RESPOND = 0x30;  // exercises STM32 MASTER mode

// Declared ahead of all function definitions — the Arduino IDE's
// auto-generated function prototypes get inserted as a block right before
// the first function in the file, so any type used in a later function's
// signature must already be visible at that point or the prototype fails
// to compile ("does not name a type").
enum LinReadResult { LIN_READ_TIMEOUT, LIN_READ_BAD_CHECKSUM, LIN_READ_OK };

// ── PID / checksum (LIN 2.x) — mirrors Core/Src/lin_handler.c ──────────
static uint8_t linPid(uint8_t id)
{
    id &= 0x3F;
    uint8_t p0 = ((id >> 0) & 1) ^ ((id >> 1) & 1) ^ ((id >> 2) & 1) ^ ((id >> 4) & 1);
    uint8_t p1 = !(((id >> 1) & 1) ^ ((id >> 3) & 1) ^ ((id >> 4) & 1) ^ ((id >> 5) & 1));
    return id | (p0 << 6) | (p1 << 7);
}

// Enhanced checksum (PID + data), except diagnostic IDs 0x3C/0x3D (classic, data-only).
static uint8_t linChecksum(uint8_t pid, const uint8_t *data, uint8_t len)
{
    uint8_t  id6 = pid & 0x3F;
    uint16_t sum = (id6 == 0x3C || id6 == 0x3D) ? 0 : pid;
    for (uint8_t i = 0; i < len; i++)
    {
        sum += data[i];
        if (sum > 0xFF) sum -= 0xFF;
    }
    return (uint8_t)(~sum & 0xFF);
}

// ── Break generation (bit-bang the TX pin directly, transmitter paused) ─
// Toggles only TXEN0 (hands the TX pin to plain GPIO and back), not a full
// Serial.end()/begin(). HardwareSerial's full init/teardown has non-trivial,
// not-perfectly-fixed cost that can stretch the recessive gap between the
// break and the sync byte unpredictably — occasionally enough for a LIN
// analyzer to time out waiting for sync and misclassify the break as a
// standalone wake-up pulse instead of a frame header, rather than any
// problem with the break's own dominant duration. Mirrors the STM32 side's
// own lin_send_break(), which for the same reason pokes BRR directly
// instead of calling HAL_UART_Init(). Leaving RXEN0 untouched also means
// the receiver stays live through the break, unlike the old Serial.end()
// which silenced RX too.
static void linSendBreak()
{
    UCSR0B &= ~_BV(TXEN0);
    pinMode(LIN_TX_PIN, OUTPUT);
    digitalWrite(LIN_TX_PIN, LOW);
    delayMicroseconds(LIN_BREAK_US);
    digitalWrite(LIN_TX_PIN, HIGH);
    delayMicroseconds(LIN_BREAK_DELIM_US);
    UCSR0B |= _BV(TXEN0);
}

static void linSendHeader(uint8_t id)
{
    uint8_t pid = linPid(id);
    linSendBreak();
    Serial.write((uint8_t)0x55);
    Serial.write(pid);
    Serial.flush();
}

static void linSendFrame(uint8_t id, const uint8_t *data, uint8_t len)
{
    uint8_t pid = linPid(id);
    uint8_t chk = linChecksum(pid, data, len);
    linSendBreak();
    Serial.write((uint8_t)0x55);
    Serial.write(pid);
    Serial.write(data, len);
    Serial.write(chk);
    Serial.flush();
}

// Drain whatever the transceiver echoes back from our own transmission
// before resuming the passive auto-responder listener.
static void linDrainEcho()
{
    delay(2);
    while (Serial.available()) Serial.read();
}

// Read a response after linSendHeader(id): each received byte resets the
// timeout window (LIN has no length field on the wire — see lin_handler.c's
// same reasoning), the last byte received is treated as the checksum.
static LinReadResult linReadResponse(uint8_t id, uint8_t *data, uint8_t *len, uint32_t timeoutMs)
{
    uint8_t  buf[9];
    uint8_t  n = 0;
    uint32_t deadline = millis() + timeoutMs;

    while ((int32_t)(deadline - millis()) > 0)
    {
        if (Serial.available())
        {
            buf[n++] = (uint8_t)Serial.read();
            deadline = millis() + timeoutMs;
            if (n >= sizeof(buf)) break;
        }
    }

    if (n < 2) return LIN_READ_TIMEOUT;   // need at least 1 data byte + checksum

    uint8_t pid  = linPid(id);
    uint8_t rlen = n - 1;
    uint8_t chk  = linChecksum(pid, buf, rlen);
    if (chk != buf[rlen]) return LIN_READ_BAD_CHECKSUM;

    memcpy(data, buf, rlen);
    *len = rlen;
    return LIN_READ_OK;
}

// ── Passive RX state machine (auto-responder for ID_AUTO_RESPOND) ──────
// Break is detected the same way lin_handler.c does: a framing error (FE0)
// paired with a received 0x00 byte.
enum LinRxState { RX_IDLE, RX_SYNC, RX_PID };
static LinRxState rxState = RX_IDLE;
static uint8_t    autoRespCounter = 0;

static bool linRxByte(uint8_t &byte, bool &frameErr)
{
    if (!(UCSR0A & _BV(RXC0))) return false;
    frameErr = (UCSR0A & _BV(FE0)) != 0;
    byte = UDR0;   // read clears RXC0/FE0
    return true;
}

static void blinkPattern(uint8_t count, uint16_t onMs, uint16_t offMs)
{
    for (uint8_t i = 0; i < count; i++)
    {
        digitalWrite(STATUS_LED, HIGH);
        delay(onMs);
        digitalWrite(STATUS_LED, LOW);
        if (offMs) delay(offMs);
    }
}

static void linPollAutoResponder()
{
    uint8_t b;
    bool    fe;

    while (linRxByte(b, fe))
    {
        if (fe && b == 0x00) { rxState = RX_SYNC; continue; }

        switch (rxState)
        {
        case RX_SYNC:
            rxState = (b == 0x55) ? RX_PID : RX_IDLE;
            break;

        case RX_PID:
        {
            uint8_t id = b & 0x3F;
            rxState = RX_IDLE;
            if (id == ID_AUTO_RESPOND)
            {
                uint8_t data[2] = { autoRespCounter++, 0xA5 };
                uint8_t chk = linChecksum(b, data, sizeof(data));
                Serial.write(data, sizeof(data));
                Serial.write(chk);
                Serial.flush();
                linDrainEcho();
                blinkPattern(3, 60, 60);
            }
            break;
        }

        default:
            break;
        }
    }
}

// ── Timing ────────────────────────────────────────────────────────────
static uint32_t tCounter = 0;
static uint32_t tQuery   = 0;
static uint8_t  counterByte = 0;

void setup()
{
    pinMode(STATUS_LED, OUTPUT);
    pinMode(SLP_PIN, OUTPUT);
    digitalWrite(SLP_PIN, HIGH);   // normal/awake mode (not sleep)

    Serial.begin(115200);
    delay(500);   // window to have Serial Monitor already open at reset
    Serial.println(F("LINSim_DUT_LIN - Arduino Nano + LINTTL3"));
    Serial.println(F("Dedicating hardware UART to the LIN bus @ 19200 now."));
    Serial.println(F("No further USB serial output - see onboard LED for status."));
    Serial.flush();
    delay(200);

    Serial.end();
    Serial.begin(LIN_BAUD);   // from here on, Serial == the LIN bus

    tCounter = tQuery = millis();
}

void loop()
{
    uint32_t now = millis();

    if (now - tCounter >= 1000)   // 1 Hz: self-contained counter frame
    {
        tCounter = now;
        uint8_t data[1] = { counterByte++ };
        linSendFrame(ID_COUNTER, data, sizeof(data));
        linDrainEcho();
        blinkPattern(1, 30, 0);
    }

    if (now - tQuery >= 2000)   // 0.5 Hz: query the STM32 (SLAVE-mode test)
    {
        tQuery = now;
        linSendHeader(ID_QUERY_SLAVE);

        uint8_t data[8], len;
        switch (linReadResponse(ID_QUERY_SLAVE, data, &len, QUERY_TIMEOUT_MS))
        {
        case LIN_READ_OK:           blinkPattern(2, 80, 80); break;   // valid response
        case LIN_READ_BAD_CHECKSUM: blinkPattern(4, 40, 40); break;   // checksum mismatch
        case LIN_READ_TIMEOUT:      blinkPattern(1, 400, 0); break;   // no response seen
        }

        linDrainEcho();
    }

    linPollAutoResponder();   // passive listen for STM32-driven 0x30 headers
}
