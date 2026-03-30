/*
 * O'CELL BMS CAN Frame Simulator — Extended Sensor Edition
 * =========================================================
 * Simulates a realistic multi-phase charge/discharge/thermal scenario
 * for an O'CELL IFS60.8-500-F-E3  19S1P LiFePO4  60.8V / 50Ah pack.
 *
 * ── EXTENDED vs ORIGINAL ───────────────────────────────────────────────────
 *  Added frames for all BOM sensors:
 *
 *  0x98D001F4  AUX VOLTAGE        — 12V auxiliary bus (MOD1150-3 isolated sensor)
 *                                   + HV bus read (C11A063 0-25V scaled to 0-60V via divider)
 *  0x98D002F4  MPPT CURRENT       — Solar charge current × 2 ch (ACS712-5A)
 *  0x98D003F4  EXT TEMPERATURES   — DS18B20 × 2 (battery enclosure), NTC × 2 (MPPT/DC-DC HS)
 *  0x98D004F4  DISCRETE I/O       — Handbrake (NPN logic), Oil level (float switch)
 *  0x98D005F4  TELEMETRY STATUS   — GSM link quality, MQTT connected flag, uplink counter
 *
 * ── CAN FRAME ID SCHEME ────────────────────────────────────────────────────
 *  All frames use extended 29-bit IDs in the format:
 *    0x98 | <sub-id> | 0x28 | 0xF4
 *  which matches the existing BMS protocol.
 *
 * ── PHYSICAL UNITS (raw encoding) ──────────────────────────────────────────
 *  Aux voltage    : uint16 LE, unit 10 mV  → divide by 100 to get V
 *  HV bus voltage : uint16 LE, unit 10 mV  → divide by 100 to get V
 *  MPPT current   : int16  LE, unit 10 mA  → divide by 100 to get A
 *  DS18B20 / NTC  : uint8  offset +40      → byte − 40 = °C  (same as BMS temps)
 *  Handbrake      : uint8  0x00=OFF 0x01=ON
 *  Oil level      : uint8  0x00=LOW 0x01=OK
 *  GSM RSSI       : uint8  0–31 (AT+CSQ scale)
 *  MQTT connected : uint8  0x00=disconnected 0x01=connected
 *  Uplink counter : uint16 LE, increments every telemetry frame
 *
 * ── SIMULATION APPROACH ────────────────────────────────────────────────────
 *  All new sensor values are driven by the same phase engine as the BMS so
 *  the data is physically coherent:
 *  • Aux 12V sags slightly under high load (phase 1,2).
 *  • MPPT solar current is zero during discharge, rises during CC/CV charge.
 *  • DS18B20 enclosure temps track the BMS thermal model.
 *  • NTC heatsink temps spike during MPPT active phases.
 *  • Handbrake toggles at phase boundaries (vehicle parked during charge).
 *  • Oil level stays OK except briefly LOW mid-discharge for realism.
 *  • GSM RSSI drifts gently; MQTT connected except during low-SOC phase.
 *
 * Board  : ESP32 (any variant)  — same wiring as original
 * Baud   : 115200
 *
 * ── STM32 PORT NOTE ────────────────────────────────────────────────────────
 *  To run on an STM32 (e.g. Nucleo or Blue Pill + MCP2515):
 *  1. Replace Serial.printf → use a HAL_UART_Transmit wrapper or printf
 *     redirected to USART2 (PA2/PA3, 115200).
 *  2. Replace millis()      → HAL_GetTick() (returns uint32_t ms).
 *  3. Replace random()      → rand() from <stdlib.h>.
 *  4. Replace randomSeed()  → srand(HAL_GetTick()).
 *  5. Replace constrain(v,lo,hi) → fminf(fmaxf(v,lo),hi) or write a macro.
 *  6. Replace analogRead(0) → HAL ADC reading on any floating pin.
 *  All frame-building and printFrame() logic is pure C99 — no ESP32-isms.
 */

// ─────────────────────────────────────────────────────────────────────────────
// Cell personalities — fixed per-cell offset in mV
// ─────────────────────────────────────────────────────────────────────────────
const int CELL_OFFSET[19] = {
   2,  0,  5, -3,   //  1-4
   1,  4, -18,  3,  //  5-8  (cell 7 = weak)
  -1,  2,  6,  0,   //  9-12
   3, -18,  1, -2,  // 13-16 (cell 14 = weak)
   4,  0,  2          // 17-19
};

// ─────────────────────────────────────────────────────────────────────────────
// Live simulation state  — BMS core
// ─────────────────────────────────────────────────────────────────────────────
float  soc       = 72.0f;    // %
float  packV     = 0.0f;     // V   (derived)
float  currentA  = 0.0f;     // A   +discharge / −charge
float  tempC[3]  = {24.0f, 23.5f, 23.0f};  // °C — BMS sensors 1,2,3
int    cellMv[19];           // computed each tick

// ─────────────────────────────────────────────────────────────────────────────
// Live simulation state  — Extended BOM sensors
// ─────────────────────────────────────────────────────────────────────────────
float  auxV        = 12.8f;  // 12V auxiliary bus voltage (V)
float  hvBusV      = 0.0f;   // HV bus voltage read by C11A063 divider (V)
float  mpptA[2]    = {0.0f, 0.0f}; // MPPT solar current channels 1 & 2 (A)
float  ds18Temp[2] = {24.0f, 23.5f}; // DS18B20: battery enclosure positions
float  ntcTemp[2]  = {24.0f, 24.0f}; // NTC 10K: MPPT & DC-DC heatsink
uint8_t handbrake  = 0x00;  // 0=released, 1=applied
uint8_t oilLevel   = 0x01;  // 0=LOW, 1=OK
uint8_t gsmRSSI    = 18;    // 0-31 AT+CSQ scale (~−83 dBm typical)
uint8_t mqttConn   = 0x01;  // 0=disconnected, 1=connected
uint16_t uplinkCnt = 0;     // wraps at 65535

// ─────────────────────────────────────────────────────────────────────────────
// Phase engine
// ─────────────────────────────────────────────────────────────────────────────
enum Phase {
  PHASE_REST = 0,
  PHASE_BULK_DISCHARGE,
  PHASE_LOW_SOC,
  PHASE_RECOVERY,
  PHASE_CC_CHARGE,
  PHASE_CV_TAPER,
  PHASE_FULL_REST,
  PHASE_COUNT
};

// Duration of each phase in simulation ticks (1 tick = 500 ms)
const int PHASE_TICKS[PHASE_COUNT] = {
  40,   // 0 REST            — 20 s
  240,  // 1 BULK DISCHARGE  — 120 s
  50,   // 2 LOW SOC         — 25 s
  30,   // 3 RECOVERY        — 15 s
  180,  // 4 CC CHARGE       — 90 s
  80,   // 5 CV TAPER        — 40 s
  40    // 6 FULL REST        — 20 s
};

Phase currentPhase = PHASE_REST;
int   phaseTick    = 0;   // tick counter within current phase

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────
void packBE16(uint8_t* buf, int o, uint16_t v) {
  buf[o] = (v >> 8) & 0xFF; buf[o+1] = v & 0xFF;
}
void packLE16(uint8_t* buf, int o, uint16_t v) {
  buf[o] = v & 0xFF; buf[o+1] = (v >> 8) & 0xFF;
}

// Signed 16-bit LE (for currents that can be negative)
void packLE16s(uint8_t* buf, int o, int16_t v) {
  buf[o] = (uint8_t)(v & 0xFF); buf[o+1] = (uint8_t)((v >> 8) & 0xFF);
}

// Gaussian-ish noise using two uniform samples (Box-Muller lite)
float randNoise(float amplitude) {
  float r = ((float)random(-1000, 1001)) / 1000.0f;
  return r * amplitude;
}

// Map SOC → nominal open-circuit cell voltage for LiFePO4
// Flat plateau 20-90%, steep ends
float socToCellOCV(float s) {
  if (s >= 90.0f) return 3.40f + (s - 90.0f) * 0.0310f;   // 3.40→3.65 (90-100%)
  if (s >= 20.0f) return 3.20f + (s - 20.0f) * 0.00286f;  // 3.20→3.40 (20-90%) flat plateau
  return 2.50f  + (s / 20.0f) * 0.70f;                    // 2.50→3.20 (0-20%)
}

void printFrame(uint32_t id, const uint8_t* data, uint8_t dlc) {
  Serial.printf("[%lums] ID: 0x%08X DLC: %d Data:", millis(), id, dlc);
  for (int i = 0; i < dlc; i++) Serial.printf(" %02X", data[i]);
  Serial.println();
}

// ─────────────────────────────────────────────────────────────────────────────
// Frame senders — BMS core (unchanged)
// ─────────────────────────────────────────────────────────────────────────────
void sendCellFrames() {
  uint8_t buf[8];
  for (int group = 0; group < 5; group++) {
    memset(buf, 0, 8);
    uint32_t id = (0x98UL << 24) | ((0xC8 + group) << 16) | (0x28 << 8) | 0xF4;
    int base = group * 4;
    for (int i = 0; i < 4; i++) {
      int idx = base + i;
      if (idx < 19) packBE16(buf, i * 2, (uint16_t)cellMv[idx]);
    }
    printFrame(id, buf, 8);
  }
}

void sendCDFrame() {
  uint8_t buf[8] = {0};
  printFrame((0x98UL<<24)|(0xCD<<16)|(0x28<<8)|0xF4, buf, 8);
}

void sendTempFrame() {
  uint8_t buf[8] = {0};
  for (int i = 0; i < 3; i++)
    buf[i] = (uint8_t)constrain(tempC[i] + 40.0f + 0.5f, 0, 255);
  printFrame((0x98UL<<24)|(0xB4<<16)|(0x28<<8)|0xF4, buf, 8);
}

void sendFFE5Frame(float chgReq) {
  uint8_t buf[8] = {0};
  packLE16(buf, 0, (uint16_t)(soc * 10.0f));
  packLE16(buf, 2, (uint16_t)(max(0.0f, chgReq) * 10.0f));
  printFrame((0x98UL<<24)|(0xFF<<16)|(0xE5<<8)|0xF4, buf, 8);
}

void sendFF28Frame(float dischLimA) {
  uint8_t buf[8] = {0};
  packLE16(buf, 0, (uint16_t)(packV * 100.0f));
  packLE16(buf, 2, (uint16_t)(dischLimA * 100.0f));
  packLE16(buf, 4, (uint16_t)(soc * 10.0f));
  printFrame((0x98UL<<24)|(0xFF<<16)|(0x28<<8)|0xF4, buf, 8);
}

void sendFE28Frame(float dischLimA) {
  uint8_t buf[8] = {0};
  int maxMv = cellMv[0], minMv = cellMv[0];
  for (int i = 1; i < 19; i++) {
    if (cellMv[i] > maxMv) maxMv = cellMv[i];
    if (cellMv[i] < minMv) minMv = cellMv[i];
  }
  packLE16(buf, 0, (uint16_t)maxMv);
  packLE16(buf, 2, (uint16_t)minMv);
  buf[4] = (uint8_t)constrain(tempC[0] + 40.0f + 0.5f, 0, 255);
  buf[5] = (uint8_t)constrain(tempC[1] + 40.0f + 0.5f, 0, 255);
  packLE16(buf, 6, (uint16_t)(dischLimA * 10.0f));
  printFrame((0x98UL<<24)|(0xFE<<16)|(0x28<<8)|0xF4, buf, 8);
}

// ─────────────────────────────────────────────────────────────────────────────
// Frame senders — Extended BOM sensors (NEW)
// ─────────────────────────────────────────────────────────────────────────────

/*
 * 0x98D001F4 — AUX & HV BUS VOLTAGES
 * ─────────────────────────────────────
 * Byte 0-1 : Aux 12V bus voltage   LE uint16  unit: 10 mV  (e.g. 1280 = 12.80 V)
 * Byte 2-3 : HV bus voltage        LE uint16  unit: 10 mV  (e.g. 6100 = 61.00 V)
 * Byte 4-7 : reserved (0x00)
 *
 * Sensors  : MOD1150-3 (isolated 12V readout), C11A063 0-25V scaled by resistor divider
 */
void sendAuxVoltageFrame() {
  uint8_t buf[8] = {0};
  packLE16(buf, 0, (uint16_t)(auxV  * 100.0f));   // 10-mV unit
  packLE16(buf, 2, (uint16_t)(hvBusV * 100.0f));  // 10-mV unit
  printFrame((0x98UL<<24)|(0xD0<<16)|(0x01<<8)|0xF4, buf, 8);
}

/*
 * 0x98D002F4 — MPPT SOLAR CURRENT (ACS712-5A × 2)
 * ─────────────────────────────────────────────────
 * Byte 0-1 : MPPT channel 1 current  LE int16  unit: 10 mA  (signed, +ve = into pack)
 * Byte 2-3 : MPPT channel 2 current  LE int16  unit: 10 mA
 * Byte 4-5 : Combined MPPT power     LE uint16 unit: 100 mW (approximate)
 * Byte 6-7 : reserved (0x00)
 *
 * Sensor   : ACS712-5A (Qty 2) — one per MPPT output wire
 */
void sendMpptCurrentFrame() {
  uint8_t buf[8] = {0};
  int16_t ch1 = (int16_t)(mpptA[0] * 100.0f);   // 10-mA unit
  int16_t ch2 = (int16_t)(mpptA[1] * 100.0f);
  packLE16s(buf, 0, ch1);
  packLE16s(buf, 2, ch2);
  // Combined power: V_pack × (ch1+ch2) in 100-mW units
  float totalW = packV * (mpptA[0] + mpptA[1]);
  packLE16(buf, 4, (uint16_t)constrain(totalW * 10.0f, 0.0f, 65535.0f));
  printFrame((0x98UL<<24)|(0xD0<<16)|(0x02<<8)|0xF4, buf, 8);
}

/*
 * 0x98D003F4 — EXTERNAL TEMPERATURES (DS18B20 × 2 + NTC 10K × 2)
 * ─────────────────────────────────────────────────────────────────
 * Byte 0 : DS18B20 sensor 1 — battery enclosure surface A   (+40 offset, same as BMS)
 * Byte 1 : DS18B20 sensor 2 — battery enclosure surface B
 * Byte 2 : NTC 10K sensor 1 — MPPT controller heatsink
 * Byte 3 : NTC 10K sensor 2 — DC-DC converter heatsink
 * Byte 4-7 : reserved (0x00)
 *
 * Formula  : raw_byte = (int)(T_celsius + 40 + 0.5)  →  T = raw_byte − 40
 * Sensors  : DS18B20 waterproof probe × 2, NTC 10K 3950 × 2
 */
void sendExtTempFrame() {
  uint8_t buf[8] = {0};
  buf[0] = (uint8_t)constrain(ds18Temp[0] + 40.0f + 0.5f, 0, 255);
  buf[1] = (uint8_t)constrain(ds18Temp[1] + 40.0f + 0.5f, 0, 255);
  buf[2] = (uint8_t)constrain(ntcTemp[0]  + 40.0f + 0.5f, 0, 255);
  buf[3] = (uint8_t)constrain(ntcTemp[1]  + 40.0f + 0.5f, 0, 255);
  printFrame((0x98UL<<24)|(0xD0<<16)|(0x03<<8)|0xF4, buf, 8);
}

/*
 * 0x98D004F4 — DISCRETE I/O STATUS
 * ─────────────────────────────────
 * Byte 0 : Handbrake status  0x00=released  0x01=applied
 * Byte 1 : Oil level status  0x00=LOW/fault 0x01=OK
 * Byte 2-7 : reserved (0x00)
 *
 * Sensors  : NPN N/O proximity sensor LJ12A3-4-Z/BX (handbrake),
 *            float-switch / resistive sender (oil level)
 */
void sendDiscreteIOFrame() {
  uint8_t buf[8] = {0};
  buf[0] = handbrake;
  buf[1] = oilLevel;
  printFrame((0x98UL<<24)|(0xD0<<16)|(0x04<<8)|0xF4, buf, 8);
}

/*
 * 0x98D005F4 — TELEMETRY / GSM STATUS (SIM800L)
 * ────────────────────────────────────────────────
 * Byte 0   : GSM RSSI   0-31 AT+CSQ scale  (99 = unknown/no signal)
 * Byte 1   : MQTT connected flag  0x00=no  0x01=yes
 * Byte 2-3 : Uplink frame counter  LE uint16  (wraps at 65535)
 * Byte 4   : Network registration  0=not registered 1=home 5=roaming
 * Byte 5-7 : reserved (0x00)
 *
 * Module   : SIM800L GSM/GPRS via UART → MQTT (Ooredoo / Orange TN 2G)
 */
void sendTelemetryFrame() {
  uint8_t buf[8] = {0};
  buf[0] = gsmRSSI;
  buf[1] = mqttConn;
  packLE16(buf, 2, uplinkCnt);
  // Network registration: assume home network (1) when RSSI is valid
  buf[4] = (gsmRSSI < 99) ? 0x01 : 0x00;
  printFrame((0x98UL<<24)|(0xD0<<16)|(0x05<<8)|0xF4, buf, 8);
  uplinkCnt++;
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase transition
// ─────────────────────────────────────────────────────────────────────────────
void nextPhase() {
  currentPhase = (Phase)((currentPhase + 1) % PHASE_COUNT);
  // Skip phase 0 (REST) when looping — go straight to discharge
  if (currentPhase == PHASE_REST && phaseTick > 0) {
    currentPhase = PHASE_BULK_DISCHARGE;
  }
  phaseTick = 0;
  Serial.printf("[INFO] ── Phase → %d ──\n", currentPhase);
}

// ─────────────────────────────────────────────────────────────────────────────
// Main simulation update — called every 500 ms
// ─────────────────────────────────────────────────────────────────────────────
void updateSimulation() {

  // ── Phase-specific state machine ─────────────────────────────────────────

  float dischLimA = 50.0f;  // BMS discharge current limit (default)
  float chgReqA   = 0.0f;   // BMS charge current request
  float loadSagMv = 0.0f;   // extra cell sag under current load (mV)

  float progress = (PHASE_TICKS[currentPhase] > 0)
                   ? (float)phaseTick / PHASE_TICKS[currentPhase]
                   : 1.0f;

  switch (currentPhase) {

    // ── 0 REST ──────────────────────────────────────────────────────────────
    case PHASE_REST:
      currentA  = 0.0f;
      chgReqA   = 5.0f;
      dischLimA = 50.0f;
      // Gentle thermal settling toward ambient
      for (int i = 0; i < 3; i++)
        tempC[i] += (23.0f - tempC[i]) * 0.04f + randNoise(0.05f);
      // Aux bus stable; MPPT off; vehicle parked → handbrake ON
      auxV       += (12.8f - auxV) * 0.10f + randNoise(0.02f);
      mpptA[0]   = 0.0f;
      mpptA[1]   = 0.0f;
      handbrake  = 0x01;
      oilLevel   = 0x01;
      // GSM stays connected at rest
      gsmRSSI    = (uint8_t)constrain(gsmRSSI + (int)randNoise(2.0f), 12, 25);
      mqttConn   = 0x01;
      break;

    // ── 1 BULK DISCHARGE ────────────────────────────────────────────────────
    case PHASE_BULK_DISCHARGE:
      currentA  = 38.0f + randNoise(1.5f);        // ~38A load
      loadSagMv = 18.0f;                           // under-load sag
      chgReqA   = 0.0f;
      // SOC drops at ~0.30%/tick (≈0.6%/s)
      soc -= 0.30f + randNoise(0.02f);
      // Temps rise: centre hottest
      tempC[0] += 0.08f + randNoise(0.03f);
      tempC[1] += 0.06f + randNoise(0.02f);
      tempC[2] += 0.03f + randNoise(0.02f);
      dischLimA = 50.0f;
      // Aux bus sags slightly under heavy load (12.8V → ~12.1V)
      auxV += (12.1f - auxV) * 0.03f + randNoise(0.03f);
      // No solar during discharge; vehicle moving → handbrake OFF
      mpptA[0]  = 0.0f;
      mpptA[1]  = 0.0f;
      handbrake = 0x00;
      // Oil level OK, may flicker at high load mid-phase
      oilLevel  = (progress > 0.6f && progress < 0.65f) ? 0x00 : 0x01;
      // GSM: slight RSSI variation from vibration
      gsmRSSI   = (uint8_t)constrain(gsmRSSI + (int)randNoise(3.0f), 10, 28);
      mqttConn  = 0x01;
      break;

    // ── 2 LOW SOC WARNING ───────────────────────────────────────────────────
    case PHASE_LOW_SOC:
      // BMS throttles current; linearly ramp down 38A → 12A
      currentA  = 38.0f - progress * 26.0f + randNoise(0.8f);
      loadSagMv = 10.0f * (1.0f - progress);
      chgReqA   = 25.0f;   // BMS asking to charge ASAP
      soc      -= 0.22f + randNoise(0.02f);
      // Discharge limit ramps down 50A → 15A
      dischLimA = 50.0f - progress * 35.0f;
      // Temps plateau then tick up slightly
      tempC[0] += 0.02f + randNoise(0.03f);
      tempC[1] += 0.01f + randNoise(0.02f);
      tempC[2] += randNoise(0.02f);
      // Aux bus strained: droops to ~11.8V
      auxV += (11.8f - auxV) * 0.05f + randNoise(0.04f);
      mpptA[0]  = 0.0f;
      mpptA[1]  = 0.0f;
      handbrake = 0x00;
      oilLevel  = 0x01;
      // GSM may drop packets under high electrical noise
      gsmRSSI   = (uint8_t)constrain(gsmRSSI - 1 + (int)randNoise(3.0f), 8, 22);
      mqttConn  = (gsmRSSI < 11) ? 0x00 : 0x01;
      break;

    // ── 3 RECOVERY / REST ───────────────────────────────────────────────────
    case PHASE_RECOVERY:
      currentA  = 0.0f;
      chgReqA   = 25.0f;
      dischLimA = 10.0f;   // BMS keeps limit low while recovering
      // Voltage bounce-back: SOC appears to tick up slightly (polarisation)
      soc      += 0.08f * (1.0f - progress);
      // Temps begin falling
      for (int i = 0; i < 3; i++)
        tempC[i] += (23.0f - tempC[i]) * 0.06f + randNoise(0.04f);
      // Aux bus recovering toward 12.5V
      auxV += (12.5f - auxV) * 0.08f + randNoise(0.03f);
      mpptA[0]  = 0.0f;
      mpptA[1]  = 0.0f;
      handbrake = 0x01;   // parked while recovering
      oilLevel  = 0x01;
      gsmRSSI   = (uint8_t)constrain(gsmRSSI + 1 + (int)randNoise(2.0f), 12, 26);
      mqttConn  = 0x01;
      break;

    // ── 4 CC CHARGE ─────────────────────────────────────────────────────────
    case PHASE_CC_CHARGE:
      currentA  = -22.0f + randNoise(0.5f);   // negative = charging
      chgReqA   = 22.0f;
      dischLimA = 50.0f;
      // SOC climbs ~0.28%/tick
      soc      += 0.28f + randNoise(0.02f);
      // Mild thermal rise at start, plateaus mid-charge
      {
        float heatFactor = (progress < 0.3f) ? (1.0f - progress/0.3f * 0.5f) : 0.5f;
        tempC[0] += 0.04f * heatFactor + randNoise(0.03f);
        tempC[1] += 0.03f * heatFactor + randNoise(0.02f);
        tempC[2] += 0.01f * heatFactor + randNoise(0.02f);
      }
      // Aux bus healthy from charger supply: 13.2V
      auxV += (13.2f - auxV) * 0.05f + randNoise(0.02f);
      // Solar panels contributing: ramp 0 → 2.5A each over phase
      mpptA[0] = 2.5f * progress + randNoise(0.15f);
      mpptA[1] = 2.2f * progress + randNoise(0.12f);
      // NTC heatsinks warm up with MPPT active
      ntcTemp[0] += 0.06f * (1.0f - progress * 0.5f) + randNoise(0.04f);
      ntcTemp[1] += 0.04f * (1.0f - progress * 0.5f) + randNoise(0.03f);
      handbrake = 0x01;   // parked and charging
      oilLevel  = 0x01;
      gsmRSSI   = (uint8_t)constrain(gsmRSSI + (int)randNoise(2.0f), 14, 28);
      mqttConn  = 0x01;
      break;

    // ── 5 CV TAPER ──────────────────────────────────────────────────────────
    case PHASE_CV_TAPER:
      // Current tapers from 22A → 2A exponentially
      currentA  = -(22.0f * (1.0f - progress) * (1.0f - progress) + 2.0f)
                  + randNoise(0.3f);
      chgReqA   = max(0.0f, 22.0f * (1.0f - progress));
      dischLimA = 50.0f;
      // SOC creeps up 80% → 100%
      soc      += 0.10f * (1.0f - progress * 0.7f) + randNoise(0.01f);
      // Temps drift back toward ambient
      for (int i = 0; i < 3; i++)
        tempC[i] += (23.0f - tempC[i]) * 0.05f + randNoise(0.03f);
      // Aux bus remains elevated from charger
      auxV += (13.1f - auxV) * 0.04f + randNoise(0.02f);
      // MPPT tapers with charge current (solar less useful near full)
      mpptA[0] = 2.5f * (1.0f - progress) + randNoise(0.10f);
      mpptA[1] = 2.2f * (1.0f - progress) + randNoise(0.08f);
      // NTC heatsinks cool as MPPT power reduces
      ntcTemp[0] += (28.0f - ntcTemp[0]) * 0.04f + randNoise(0.04f);
      ntcTemp[1] += (27.0f - ntcTemp[1]) * 0.03f + randNoise(0.03f);
      handbrake = 0x01;
      oilLevel  = 0x01;
      gsmRSSI   = (uint8_t)constrain(gsmRSSI + (int)randNoise(2.0f), 14, 28);
      mqttConn  = 0x01;
      break;

    // ── 6 FULL REST ─────────────────────────────────────────────────────────
    case PHASE_FULL_REST:
      currentA  = 0.0f;
      chgReqA   = 0.0f;
      dischLimA = 50.0f;
      soc       = constrain(soc, 99.5f, 100.0f);
      for (int i = 0; i < 3; i++)
        tempC[i] += (23.0f - tempC[i]) * 0.07f + randNoise(0.03f);
      // Back to float charge from charger, stable at 13.4V
      auxV += (13.4f - auxV) * 0.06f + randNoise(0.02f);
      mpptA[0]  = 0.0f;  // No MPPT needed at 100% SOC
      mpptA[1]  = 0.0f;
      // Heatsinks cooling toward ambient
      ntcTemp[0] += (24.0f - ntcTemp[0]) * 0.06f + randNoise(0.03f);
      ntcTemp[1] += (24.0f - ntcTemp[1]) * 0.05f + randNoise(0.03f);
      handbrake = 0x01;
      oilLevel  = 0x01;
      gsmRSSI   = (uint8_t)constrain(gsmRSSI + (int)randNoise(2.0f), 15, 28);
      mqttConn  = 0x01;
      break;

    default: break;
  }

  // ── Clamp SOC & temps ────────────────────────────────────────────────────
  soc = constrain(soc, 5.0f, 100.0f);
  for (int i = 0; i < 3; i++)
    tempC[i] = constrain(tempC[i], 15.0f, 58.0f);

  // Clamp extended sensor values
  auxV        = constrain(auxV, 10.0f, 14.5f);
  mpptA[0]    = constrain(mpptA[0], 0.0f, 5.0f);
  mpptA[1]    = constrain(mpptA[1], 0.0f, 5.0f);
  ds18Temp[0] = constrain(tempC[0] - 1.5f + randNoise(0.5f), 15.0f, 60.0f);  // tracks BMS closely
  ds18Temp[1] = constrain(tempC[1] - 1.0f + randNoise(0.5f), 15.0f, 60.0f);
  ntcTemp[0]  = constrain(ntcTemp[0], 20.0f, 70.0f);
  ntcTemp[1]  = constrain(ntcTemp[1], 20.0f, 65.0f);

  // ── Cell voltages ─────────────────────────────────────────────────────────
  float baseOCV = socToCellOCV(soc) * 1000.0f;  // → mV

  for (int i = 0; i < 19; i++) {
    float offset  = (float)CELL_OFFSET[i];
    float weakSag = 0.0f;
    if ((i == 6 || i == 13) && currentA > 0)
      weakSag = loadSagMv * 0.7f;
    float loadSag = (currentA > 0) ? (currentA * 0.25f + loadSagMv) : 0.0f;
    float noise   = randNoise(2.5f);
    float mv      = baseOCV + offset - loadSag - weakSag + noise;
    cellMv[i]     = (int)constrain(mv, 2480.0f, 3720.0f);
  }

  // ── Pack voltage ─────────────────────────────────────────────────────────
  float sumMv = 0;
  for (int i = 0; i < 19; i++) sumMv += cellMv[i];
  packV   = (sumMv / 1000.0f) + randNoise(0.03f);
  hvBusV  = packV + randNoise(0.05f);   // HV bus ≈ pack voltage (same rail)

  // ── Phase advance ─────────────────────────────────────────────────────────
  phaseTick++;
  if (phaseTick >= PHASE_TICKS[currentPhase]) nextPhase();

  // ── Send all frames ───────────────────────────────────────────────────────
  // BMS core frames (original)
  sendCellFrames();
  sendCDFrame();
  sendTempFrame();
  sendFFE5Frame(chgReqA);
  sendFF28Frame(dischLimA);
  sendFE28Frame(dischLimA);

  // Extended BOM sensor frames (new)
  sendAuxVoltageFrame();
  sendMpptCurrentFrame();
  sendExtTempFrame();
  sendDiscreteIOFrame();
  sendTelemetryFrame();
}

// ─────────────────────────────────────────────────────────────────────────────
// Timing
// ─────────────────────────────────────────────────────────────────────────────
unsigned long lastSlowMs = 0;
unsigned long lastFastMs = 0;

// Cached limits for the fast cycle (updated on slow tick)
float _fastDischLim = 50.0f;
float _fastChgReq   = 0.0f;

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("============================================================");
  Serial.println("  O'CELL BMS CAN SIMULATOR — EXTENDED SENSOR EDITION");
  Serial.println("  19S1P LiFePO4  60.8V / 50Ah  |  7-Phase Cycle");
  Serial.println("  Extended frames: Aux V, MPPT I, Ext Temp, I/O, GSM");
  Serial.println("============================================================");
  Serial.println("[INFO] Phase 0: REST — pack idle at 72% SOC");
  randomSeed(analogRead(0));
}

void loop() {
  unsigned long now = millis();

  // ── Slow cycle (500ms): full simulation update + all frames ─────────────
  if (now - lastSlowMs >= 500) {
    lastSlowMs = now;
    updateSimulation();
  }

  // ── Fast cycle (100ms): pack summary + aux voltage only ─────────────────
  if (now - lastFastMs >= 100) {
    lastFastMs = now;
    if (now - lastSlowMs > 10) {
      sendFF28Frame(_fastDischLim);
      sendFE28Frame(_fastDischLim);
      sendAuxVoltageFrame();   // aux voltage also useful at fast rate
    }
  }
}
