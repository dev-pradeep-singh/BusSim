/*
 * CANSim_DUT — Arduino Nano + MCP2515
 * Device Under Test for the CANSim Blue Pill + demo.dbc demo
 *
 * What this sketch does
 * ─────────────────────
 *  • Transmits four periodic messages matching Tools/demo.dbc:
 *      0x100  EngineData        10 ms
 *      0x200  VehicleSpeed      10 ms
 *      0x300  TransmissionData  20 ms
 *      0x400  BatteryStatus    100 ms
 *  • Responds to 0x7DF UDS functional requests (basic service set)
 *  • Receives 0x123 CANSimHeartbeat and prints decoded Counter + Flags
 *  • Simulates a full drive cycle: idle → acceleration → cruise → decel
 *    (one cycle ~80 s, repeats automatically)
 *
 * Library required
 * ────────────────
 *  arduino-mcp2515 by autowp
 *  Install: Sketch → Include Library → Manage Libraries → search "mcp2515"
 *  Or: https://github.com/autowp/arduino-mcp2515
 *
 * Wiring — Arduino Nano ↔ MCP2515 module
 * ──────────────────────────────────────
 *   MCP2515  Arduino Nano
 *   VCC   →  5 V
 *   GND   →  GND
 *   CS    →  D10
 *   SO    →  D12  (MISO)
 *   SI    →  D11  (MOSI)
 *   SCK   →  D13
 *   INT   →  D2
 *
 * CAN bus wiring
 * ──────────────
 *   Arduino MCP2515 CANH/CANL  ←→  Blue Pill TJA1051 CANH/CANL
 *   ⚠ Both ends need 120 Ω termination resistors across CANH–CANL.
 *     Most MCP2515 modules have a solder jumper for the on-board 120 Ω —
 *     enable it on BOTH nodes (module + TJA1051 side or separate resistor).
 *
 * Crystal frequency
 * ─────────────────
 *   Change MCP_8MHZ → MCP_16MHZ if your module has a 16 MHz crystal.
 *   The cheap blue eBay/Amazon modules almost always use 8 MHz.
 */

#include <SPI.h>
#include <mcp2515.h>    // autowp/arduino-mcp2515

// ── Hardware ──────────────────────────────────────────────────────────
#define CS_PIN   10
#define INT_PIN   2

MCP2515 mcp2515(CS_PIN);

// ── CAN IDs (mirror demo.dbc) ─────────────────────────────────────────
static const uint32_t ID_ENGINE  = 0x100;
static const uint32_t ID_SPEED   = 0x200;
static const uint32_t ID_TRANS   = 0x300;
static const uint32_t ID_BATT    = 0x400;
static const uint32_t ID_DIAG_RQ = 0x7DF;
static const uint32_t ID_DIAG_RS = 0x7E8;
static const uint32_t ID_CANSIM  = 0x123;  // CANSimHeartbeat (RX only)

// ── Simulation state ──────────────────────────────────────────────────
struct State {
    // Engine
    float   rpm       = 750.f;
    float   throttle  = 4.f;    // %
    float   coolant   = 25.f;   // °C
    float   load      = 8.f;    // %
    uint8_t engStatus = 0x01;   // bit0 = running

    // Wheels / speed
    float   speed     = 0.f;    // km/h
    uint8_t gearLever = 3;      // D
    uint8_t gearActive= 4;      // 1st  (DBC val 4)
    float   txTemp    = 60.f;   // transmission °C
    uint8_t txMode    = 0;      // Normal

    // Battery
    float battV    = 12.6f;     // V
    float battI    = -2.f;      // A  (negative = discharge)
    float soc      = 82.f;      // %
    float battTemp = 26.f;      // °C
    float cellMax  = 4.10f;     // V
    float cellMin  = 4.05f;     // V
} s;

// Drive-cycle phase: 0=idle, 1=accel, 2=cruise, 3=decel
static uint8_t  phase      = 0;
static uint32_t phaseStart = 0;
static const uint32_t PHASE_MS = 20000; // 20 s per phase

// ── Timing ────────────────────────────────────────────────────────────
static uint32_t t10     = 0;
static uint32_t t20     = 0;
static uint32_t t100    = 0;
static uint32_t tSim    = 0;
static uint32_t tStatus = 0;

// ── Helpers: Intel (little-endian) byte packing ───────────────────────
static void u16le(uint8_t *b, uint8_t pos, uint16_t v)
{
    b[pos]   = (uint8_t)(v);
    b[pos+1] = (uint8_t)(v >> 8);
}

static void s16le(uint8_t *b, uint8_t pos, int16_t v)
{
    b[pos]   = (uint8_t)((uint16_t)v);
    b[pos+1] = (uint8_t)((uint16_t)v >> 8);
}

static uint8_t clampU8(float v) { return (uint8_t)constrain(v, 0.f, 255.f); }

// ── Drive-cycle simulation ────────────────────────────────────────────
static uint8_t gearForSpeed(float kmh)
{
    if (kmh < 15)  return 4;   // DBC enum 4 = 1st
    if (kmh < 32)  return 5;   // 2nd
    if (kmh < 52)  return 6;   // 3rd
    if (kmh < 72)  return 7;   // 4th
    if (kmh < 92)  return 8;   // 5th
    return 9;                  // 6th
}

static void updateSim()
{
    uint32_t now = millis();
    float t = (float)((now - phaseStart) % PHASE_MS) / (float)PHASE_MS;

    if (now - phaseStart >= PHASE_MS) {
        phaseStart = now;
        phase = (phase + 1) & 3;
        t = 0.f;
    }

    float wobble = sinf((float)now * 0.001f);

    switch (phase) {
        case 0: // Idle / warm-up
            s.rpm       = 750.f + 50.f * wobble;
            s.throttle  = 4.f   +  1.f * wobble;
            s.load      = 8.f   +  2.f * wobble;
            s.speed     = 0.f;
            s.gearLever = 3; s.gearActive = 4;
            s.battI     = -1.8f;
            break;

        case 1: // Acceleration 0 → 130 km/h
            s.rpm       = 900.f  + t * 3200.f + 80.f * wobble;
            s.throttle  = 30.f   + t *   60.f +  5.f * wobble;
            s.load      = 40.f   + t *   50.f;
            s.speed     = t * 130.f;
            s.gearLever = 3; s.gearActive = gearForSpeed(s.speed);
            s.battI     = -12.f - t * 6.f;     // peaks at −18 A
            break;

        case 2: // Cruise ~120 km/h
            s.rpm       = 2400.f + 120.f * wobble;
            s.throttle  =   28.f +   6.f * wobble;
            s.load      =   32.f +   5.f * wobble;
            s.speed     =  118.f +   4.f * wobble;
            s.gearLever = 3; s.gearActive = 9; // 6th
            s.battI     = -3.f;
            break;

        case 3: // Deceleration 130 → 0
            s.rpm       = 3800.f - t * 3050.f + 60.f * wobble;
            s.throttle  =    1.f +  1.f * wobble;
            s.load      =    6.f;
            s.speed     = 130.f * (1.f - t);
            s.gearLever = 3; s.gearActive = gearForSpeed(s.speed);
            s.battI     = -1.5f;
            break;
    }

    s.rpm      = constrain(s.rpm,      0.f, 7000.f);
    s.throttle = constrain(s.throttle, 0.f,  100.f);
    s.load     = constrain(s.load,     0.f,  100.f);
    s.speed    = constrain(s.speed,    0.f,  250.f);

    float tgtCoolant = (phase == 0) ? 88.f : 92.f;
    if (s.coolant < tgtCoolant) s.coolant += 0.03f;

    s.txTemp = constrain(55.f + s.speed * 0.2f, 55.f, 130.f);

    if (s.soc > 10.f) s.soc -= fabsf(s.battI) * (50.f / 3600000.f);
    s.soc     = constrain(s.soc, 0.f, 100.f);
    s.battV   = 11.4f + (s.soc / 100.f) * 2.6f;
    s.cellMax = 3.42f + (s.soc / 100.f) * 0.72f;
    s.cellMin = s.cellMax - 0.05f - fabsf(s.battI) * 0.003f;

    s.engStatus = 0x01;
    if (s.rpm  > 3500.f) s.engStatus |= 0x04;
    if (s.soc  < 20.f)   s.engStatus |= 0x02;
}

// ── Message senders (autowp: populate can_frame, call sendMessage) ────
static void txEngineData()
{
    struct can_frame f;
    f.can_id  = ID_ENGINE;
    f.can_dlc = 8;
    memset(f.data, 0, 8);
    u16le(f.data, 0, (uint16_t)(s.rpm      / 0.25f));   // RPM
    f.data[2] = clampU8(s.throttle / 0.392157f);        // ThrottlePos
    f.data[3] = clampU8(s.coolant  + 40.f);             // CoolantTemp
    f.data[4] = clampU8(s.load     / 0.392157f);        // EngineLoad
    f.data[5] = s.engStatus;                             // EngineStatus
    mcp2515.sendMessage(&f);
}

static void txVehicleSpeed()
{
    struct can_frame f;
    f.can_id  = ID_SPEED;
    f.can_dlc = 8;
    uint16_t sRaw = (uint16_t)(s.speed * 100.f);
    u16le(f.data, 0, sRaw);            // VehicleSpeed
    u16le(f.data, 2, sRaw);            // WheelSpeedFL
    u16le(f.data, 4, sRaw);            // WheelSpeedFR
    f.data[6] = clampU8(s.speed);      // WheelSpeedRL (factor=1)
    f.data[7] = clampU8(s.speed);      // WheelSpeedRR
    mcp2515.sendMessage(&f);
}

static void txTransmissionData()
{
    struct can_frame f;
    f.can_id  = ID_TRANS;
    f.can_dlc = 4;
    memset(f.data, 0, 8);
    f.data[0] = (s.gearLever & 0x0F) | ((s.gearActive & 0x0F) << 4);
    f.data[1] = clampU8(s.txTemp + 40.f);
    f.data[2] = s.txMode;
    mcp2515.sendMessage(&f);
}

static void txBatteryStatus()
{
    struct can_frame f;
    f.can_id  = ID_BATT;
    f.can_dlc = 8;
    u16le(f.data, 0, (uint16_t)(s.battV * 100.f));    // BattVoltage
    s16le(f.data, 2, (int16_t) (s.battI * 10.f));     // BattCurrent (signed)
    f.data[4] = clampU8(s.soc      / 0.392157f);      // BattSOC
    f.data[5] = clampU8(s.battTemp + 40.f);            // BattTemp
    f.data[6] = clampU8(s.cellMax  / 0.02f);           // BattCellVoltMax
    f.data[7] = clampU8(s.cellMin  / 0.02f);           // BattCellVoltMin
    mcp2515.sendMessage(&f);
}

// ── UDS handler (0x7DF → 0x7E8) ──────────────────────────────────────
static void handleUDS(const uint8_t *buf, uint8_t len)
{
    if (len < 1) return;
    uint8_t svc = buf[0];

    struct can_frame resp;
    resp.can_id  = ID_DIAG_RS;
    resp.can_dlc = 8;
    memset(resp.data, 0, 8);

    switch (svc) {
        case 0x10:  // DiagnosticSessionControl
            resp.data[0] = 0x02;
            resp.data[1] = 0x50;
            resp.data[2] = (len > 1) ? buf[1] : 0x01;
            break;
        case 0x3E:  // TesterPresent
            resp.data[0] = 0x02;
            resp.data[1] = 0x7E;
            resp.data[2] = 0x00;
            break;
        case 0x22:  // ReadDataByIdentifier — DID 0x0C0E = engine speed demo
            if (len >= 3 && buf[1] == 0x0C && buf[2] == 0x0E) {
                uint16_t rRpm = (uint16_t)(s.rpm / 0.25f);
                resp.data[0] = 0x05; resp.data[1] = 0x62;
                resp.data[2] = 0x0C; resp.data[3] = 0x0E;
                resp.data[4] = (uint8_t)(rRpm >> 8);
                resp.data[5] = (uint8_t)(rRpm);
            } else {
                resp.data[0] = 0x03; resp.data[1] = 0x7F;
                resp.data[2] = svc;  resp.data[3] = 0x31; // requestOutOfRange
            }
            break;
        default:
            resp.data[0] = 0x03; resp.data[1] = 0x7F;
            resp.data[2] = svc;  resp.data[3] = 0x11;     // serviceNotSupported
            break;
    }
    mcp2515.sendMessage(&resp);
}

// ── RX handler (called every loop; readMessage returns ERROR_NOMSG if empty) ──
static void handleRX()
{
    // Fast-exit if INT pin is still high (no message pending in either buffer)
    if (digitalRead(INT_PIN) != LOW) return;

    struct can_frame rx;
    if (mcp2515.readMessage(&rx) != MCP2515::ERROR_OK) return;

    uint32_t rxId = rx.can_id & CAN_EFF_MASK;  // strip EFF/RTR flags if any

    if (rxId == ID_DIAG_RQ) handleUDS(rx.data, rx.can_dlc);

    // Print raw frame to Serial
    Serial.print(F("RX 0x"));
    Serial.print(rxId, HEX);
    Serial.print(F("  ["));
    Serial.print(rx.can_dlc);
    Serial.print(F("]  "));
    for (uint8_t i = 0; i < rx.can_dlc; i++) {
        if (rx.data[i] < 0x10) Serial.print('0');
        Serial.print(rx.data[i], HEX);
        Serial.print(' ');
    }

    // Extra decode for CANSimHeartbeat
    if (rxId == ID_CANSIM && rx.can_dlc >= 4) {
        uint8_t  counter = rx.data[0];
        uint16_t testVal = (uint16_t)rx.data[1] | ((uint16_t)rx.data[2] << 8);
        uint8_t  flags   = rx.data[3];
        Serial.print(F("→ Counter="));  Serial.print(counter);
        Serial.print(F(" TestVal="));   Serial.print(testVal * 0.1f, 1);
        Serial.print(F(" Flags=0x"));   Serial.print(flags, HEX);
    }
    Serial.println();
}

// ── Periodic Serial status ────────────────────────────────────────────
static void printStatus()
{
    static const char *phaseNames[] = { "IDLE", "ACCEL", "CRUISE", "DECEL" };
    Serial.print(F("── "));    Serial.print(phaseNames[phase]);
    Serial.print(F("  RPM=")); Serial.print((int)s.rpm);
    Serial.print(F("  spd=")); Serial.print((int)s.speed);
    Serial.print(F(" km/h  gear=")); Serial.print(s.gearActive - 3);
    Serial.print(F("  SOC=")); Serial.print(s.soc, 1);
    Serial.print(F("%  Vcell=")); Serial.print(s.cellMax, 2);
    Serial.println(F(" V"));
}

// ── setup ─────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    while (!Serial) {}

    Serial.println(F("\r\n╔══════════════════════════════╗"));
    Serial.println(F(  "║  CANSim DUT  Arduino Nano    ║"));
    Serial.println(F(  "║  MCP2515  CAN @ 500 kbps     ║"));
    Serial.println(F(  "╚══════════════════════════════╝"));

    // Retry init until the MCP2515 is found on SPI
    // Change MCP_8MHZ → MCP_16MHZ if your module has a 16 MHz crystal
    mcp2515.reset();
    while (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
        Serial.println(F("MCP2515 setBitrate failed – check wiring / CS pin / crystal speed"));
        delay(1000);
        mcp2515.reset();
    }
    mcp2515.setNormalMode();

    pinMode(INT_PIN, INPUT);

    Serial.println(F("MCP2515 OK"));
    Serial.println(F("TX: 0x100 EngineData       (10 ms)"));
    Serial.println(F("TX: 0x200 VehicleSpeed     (10 ms)"));
    Serial.println(F("TX: 0x300 TransmissionData (20 ms)"));
    Serial.println(F("TX: 0x400 BatteryStatus   (100 ms)"));
    Serial.println(F("RX: 0x123 CANSimHeartbeat | 0x7DF DiagRequest"));
    Serial.println();

    phaseStart = millis();
    t10 = t20 = t100 = tSim = tStatus = millis();
}

// ── loop ──────────────────────────────────────────────────────────────
void loop()
{
    uint32_t now = millis();

    if (now - t10 >= 10) {          // 10 ms: Engine + Speed
        t10 = now;
        txEngineData();
        txVehicleSpeed();
    }
    if (now - t20 >= 20) {          // 20 ms: Transmission
        t20 = now;
        txTransmissionData();
    }
    if (now - t100 >= 100) {        // 100 ms: Battery
        t100 = now;
        txBatteryStatus();
    }
    if (now - tSim >= 50) {         // 50 ms: simulation step
        tSim = now;
        updateSim();
    }
    if (now - tStatus >= 2000) {    // 2 s: Serial status
        tStatus = now;
        printStatus();
    }

    handleRX();
}
