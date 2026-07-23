/*
 * LINSim_DUT_LIN — Arduino Nano + LINTTL3
 * LIN bus test harness for the BusSim Blue Pill's LIN side (lin_handler.c) —
 * simulates a 4-door / 4-window body-control slave module.
 *
 * Roles (Arduino_DUT is the LIN *master*, same as the original harness;
 * STM32_DUT stays in its default LIN_MODE_SLAVE the whole time — no
 * `LIN MODE MASTER` needed for this test)
 * ────────────────────────────────────────────────────────────────────────
 *  • Drives a LIN master schedule for two frames (see LINSim_DUT.ldf for
 *    the full byte layout):
 *
 *      0x10  Frame_DoorWindowStatus — self-contained, ~1 Hz. This sketch
 *            publishes its own simulated status: current door lock bitmask,
 *            the four windows' *live* positions (0-100%, animated toward
 *            their commanded targets over a few seconds — not instant, to
 *            model a real window motor), and a moving-status bitmask.
 *            `LISTEN ON` on the STM32 is enough to see it decoded; no
 *            STM32 action needed.
 *
 *      0x20  Frame_DoorWindowCmd — header-only, ~0.5 Hz; this is how you
 *            *set* signals from the STM32 side. Register a response on the
 *            STM32 CLI:
 *              STM32> LIN RESP 0x20 <lockBits> <winFL> <winFR> <winRL> <winRR>
 *            e.g. `LIN RESP 0x20 03 64 00 64 00` locks FL+FR (bits 0,1),
 *            commands FL/RL windows to 100% (closed) and FR/RR to 0% (open).
 *            This sketch reads that response on its next poll and applies
 *            it — door locks update instantly, window targets are picked up
 *            by the motion simulation in loop(). Re-issue `LIN RESP 0x20 ...`
 *            any time to change the command.
 *
 *  Unlike the original version of this sketch, there's no passive
 *  auto-responder path anymore (the old 0x30 test, which exercised the
 *  STM32 in LIN MODE MASTER) — STM32_DUT never needs to drive anything for
 *  this scenario, so that whole RX state machine and its single-master
 *  bus-contention caveats go away. If you want that STM32-masters-a-frame
 *  test back, 0x30 is free/unused here; see git history for the old sketch.
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
 * Wiring — Arduino Nano ↔ LINTTL3 module (unchanged from the original harness)
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
 *   Master pull-up (1 kΩ + diode, LIN pin → VIN) needed here since this
 *   sketch is the bus master for this scenario.
 *
 * Status LED (D13, onboard) — blink patterns
 * ───────────────────────────────────────────
 *   1 short blink   (30 ms)            — Frame_DoorWindowStatus sent
 *   2 blinks        (80/80 ms)         — valid Frame_DoorWindowCmd response received & applied
 *   4 fast blinks   (40/40 ms)         — command response received but checksum failed
 *   1 long blink    (400 ms)           — command poll timed out, no response seen
 *                                         (no `LIN RESP 0x20 ...` registered on the STM32 yet,
 *                                          or it's not connected/powered)
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
static const uint32_t QUERY_TIMEOUT_MS   = 20;    // response wait for Frame_DoorWindowCmd

// ── Frame IDs ────────────────────────────────────────────────────────────
static const uint8_t ID_STATUS = 0x10;  // Frame_DoorWindowStatus — self-published by this sketch
static const uint8_t ID_CMD    = 0x20;  // Frame_DoorWindowCmd — STM32 answers via LIN RESP

// ── Door/window simulation ──────────────────────────────────────────────
#define WINDOW_COUNT       4   // FL, FR, RL, RR (index order used throughout)
#define CMD_RESP_LEN       5   // DoorLockCmd + 4 window targets
#define STATUS_LEN         6   // DoorLockStatus + 4 window positions + moving bitmask
#define WINDOW_STEP_PCT    2   // position change per motion tick (simulated motor speed)
#define WINDOW_TICK_MS     100 // motion update interval -> ~0-100% sweep in ~5 s

static uint8_t doorLockBitmask = 0;              // last successfully commanded lock state (bit0=FL..bit3=RR)
static uint8_t winTarget[WINDOW_COUNT] = {0,0,0,0}; // commanded target position, 0-100 (0=open, 100=closed)
static uint8_t winPos[WINDOW_COUNT]    = {0,0,0,0}; // live/animating actual position, 0-100

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
// before the next loop() iteration.
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

// ── Door/window simulation ──────────────────────────────────────────────
// Applies a Frame_DoorWindowCmd response: door locks take effect instantly
// (no physical motor to animate), window targets are picked up by
// updateWindowMotion() on subsequent ticks.
static void applyCmdResponse(const uint8_t *data)
{
    doorLockBitmask = data[0];
    for (uint8_t i = 0; i < WINDOW_COUNT; i++)
    {
        uint8_t t = data[1 + i];
        winTarget[i] = (t > 100) ? 100 : t;   // clamp to a valid percentage
    }
}

// Steps each window's live position toward its commanded target by
// WINDOW_STEP_PCT per WINDOW_TICK_MS, modeling a real window motor's finite
// travel time rather than an instant jump.
static void updateWindowMotion()
{
    static uint32_t tMotion = 0;
    uint32_t now = millis();
    if (now - tMotion < WINDOW_TICK_MS) return;
    tMotion = now;

    for (uint8_t i = 0; i < WINDOW_COUNT; i++)
    {
        int16_t pos = winPos[i];
        int16_t tgt = winTarget[i];
        if (pos < tgt)      pos = min((int16_t)(pos + WINDOW_STEP_PCT), tgt);
        else if (pos > tgt) pos = max((int16_t)(pos - WINDOW_STEP_PCT), tgt);
        winPos[i] = (uint8_t)pos;
    }
}

// Builds the Frame_DoorWindowStatus payload from current simulated state.
static void buildStatusFrame(uint8_t *out)
{
    out[0] = doorLockBitmask;
    uint8_t moving = 0;
    for (uint8_t i = 0; i < WINDOW_COUNT; i++)
    {
        out[1 + i] = winPos[i];
        if (winPos[i] != winTarget[i]) moving |= (uint8_t)(1 << i);
    }
    out[5] = moving;
}

// ── Timing ────────────────────────────────────────────────────────────
static uint32_t tStatus = 0;
static uint32_t tCmd    = 0;

void setup()
{
    pinMode(STATUS_LED, OUTPUT);
    pinMode(SLP_PIN, OUTPUT);
    digitalWrite(SLP_PIN, HIGH);   // normal/awake mode (not sleep)

    Serial.begin(115200);
    delay(500);   // window to have Serial Monitor already open at reset
    Serial.println(F("LINSim_DUT_LIN - Arduino Nano + LINTTL3"));
    Serial.println(F("Door/window body-control slave simulator (LIN master role)"));
    Serial.println(F("Dedicating hardware UART to the LIN bus @ 19200 now."));
    Serial.println(F("No further USB serial output - see onboard LED for status."));
    Serial.flush();
    delay(200);

    Serial.end();
    Serial.begin(LIN_BAUD);   // from here on, Serial == the LIN bus

    tStatus = tCmd = millis();
}

void loop()
{
    updateWindowMotion();

    uint32_t now = millis();

    if (now - tStatus >= 1000)   // 1 Hz: publish current door/window status
    {
        tStatus = now;
        uint8_t data[STATUS_LEN];
        buildStatusFrame(data);
        linSendFrame(ID_STATUS, data, STATUS_LEN);
        linDrainEcho();
        blinkPattern(1, 30, 0);
    }

    if (now - tCmd >= 2000)   // 0.5 Hz: poll the STM32 for a new command (via LIN RESP)
    {
        tCmd = now;
        linSendHeader(ID_CMD);

        uint8_t data[8], len;
        switch (linReadResponse(ID_CMD, data, &len, QUERY_TIMEOUT_MS))
        {
        case LIN_READ_OK:
            if (len >= CMD_RESP_LEN) { applyCmdResponse(data); blinkPattern(2, 80, 80); }
            break;
        case LIN_READ_BAD_CHECKSUM: blinkPattern(4, 40, 40); break;
        case LIN_READ_TIMEOUT:      blinkPattern(1, 400, 0); break;
        }

        linDrainEcho();
    }
}
