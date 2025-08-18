/*
  Frequency and dutycycle measurement
  This example shows how to configure HardwareTimer to measure external signal frequency and dutycycle.
  The input pin will be connected to 2 channel of the timer, one for rising edge the other for falling edge.
  Each time a rising edge is detected on the input pin, hardware will save counter value into one of the CaptureCompare register.
  Each time a falling edge is detected on the input pin, hardware will save counter value into the other CaptureCompare register.
  External signal (signal generator for example) should be connected to `D2`.

*/
#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
#error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

#define pin PB6
//---- UART Protocol
HardwareSerial Serial2(PA3, PA2);

// ---- Framing constants ----
static const uint8_t SYNC1 = 0x55;
static const uint8_t SYNC2 = 0xAA;

enum RxState : uint8_t { HUNT_SYNC1, HUNT_SYNC2, READ_LABEL, READ_VALUE, READ_CRC };
static RxState rx_state = HUNT_SYNC1;

//----Timeout helpers ----
// ---------- PARAM TIMEOUT TRACKING ----------

// Keep these consistent with the Sender
const uint8_t L_BATT = 0x01;
const uint8_t L_TEMP = 0x02;
const uint8_t L_RPM  = 0x03;
const uint8_t L_DCME  = 0x04;

static const uint32_t PARAM_TIMEOUT_MS = 10000;

struct ParamState {
  bool     has = false;        // true = value is valid; false = "NULL"
  uint8_t  val = 0;            // last seen value (ignored if has == false)
  uint32_t last_ms = 0;        // millis() when we last updated it
  bool     announced_timeout = false; // to avoid spamming Serial on every loop
};

static ParamState g_batt, g_temp, g_rpm, g_dutycycle;

static const char* labelToName(uint8_t label) {
  switch (label) {
    case L_BATT: return "Battery";
    case L_TEMP: return "Temperature";
    case L_RPM:  return "RPM";
    case L_DCME: return "DutyCycle";
    default:     return "Unknown";
  }
}

static ParamState& stateForLabel(uint8_t label) {
  switch (label) {
    case L_BATT: return g_batt;
    case L_TEMP: return g_temp;
    case L_RPM:  return g_rpm;
    case L_DCME: return g_dutycycle;
    default:     return g_batt; // safe fallback, won't be used logically
  }
}
// Call this whenever we successfully parse a frame
static void noteParamSeen(uint8_t label, uint8_t value) {
  ParamState& ps = stateForLabel(label);
  ps.val = value;
  ps.has = true;
  ps.last_ms = millis();
  if (ps.announced_timeout) {
    // One-time "recovered" message after a timeout
    Serial.print(millis());
    Serial.print(" --> ");
    Serial.print(labelToName(label));
    Serial.println(" recovered");
    ps.announced_timeout = false;
  }
}

// Call this from loop() to invalidate stale values
static void paramTimeouts_step() {
  const uint32_t now = millis();

  auto check = [&](const char* name, ParamState& ps) {
    if (ps.has && (now - ps.last_ms >= PARAM_TIMEOUT_MS)) {
      // Invalidate -> interpret as "NULL"
      ps.has = false;
      ps.announced_timeout = true;
      Serial.print(millis());
      Serial.print(" --> ");
      Serial.print(name);
      Serial.println(" timeout -> NULL");
    }
  };

  check("Battery",     g_batt);
  check("Temperature", g_temp);
  check("RPM",         g_rpm);
}

// (Optional) helper you can call wherever you render to OLED / LCD / etc.
static void debugPrintCurrentIfYouLike() {
  auto pr = [&](const char* name, const ParamState& ps) {
    Serial.print(name);
    Serial.print(": ");
    if (ps.has) Serial.println(ps.val);
    else        Serial.println("NULL");
  };
  pr("Battery", g_batt);
  pr("Temperature", g_temp);
  pr("RPM", g_rpm);
}
//----Timeout helpers ----
// Define the frame TYPE *before* any function that uses it
struct RxFrame { uint8_t label; uint8_t value; };

// Manual prototype prevents Arduino from generating a wrong one
static bool uart_rx_step(RxFrame &out);

// ---- Small CRC-8 (poly 0x07, init 0x00) ----
static uint8_t crc8_07(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static inline const char* labelName(uint8_t label) {
  switch (label) {
    case 0x01: return "Battery";
    case 0x02: return "Temp";
    case 0x03: return "RPM";
    default:   return "Unknown";
  }
}

static uint8_t rx_label = 0, rx_value = 0;

// Consume at most ONE byte per call; return true only when a full, CRC-valid frame is ready.
static bool uart_rx_step(RxFrame &out) {
  if (!Serial2.available()) return false;   // <= 1 byte per loop iteration

  uint8_t byteIn = (uint8_t)Serial2.read();

  switch (rx_state) {
    case HUNT_SYNC1:
      if (byteIn == SYNC1) rx_state = HUNT_SYNC2;
      return false;

    case HUNT_SYNC2:
      if (byteIn == SYNC2) {
        rx_state = READ_LABEL;
      } else if (byteIn != SYNC1) {
        rx_state = HUNT_SYNC1;
      }
      return false;

    case READ_LABEL:
      rx_label = byteIn;
      rx_state = READ_VALUE;
      return false;

    case READ_VALUE:
      rx_value = byteIn;
      rx_state = READ_CRC;
      return false;

    case READ_CRC: {
      uint8_t calc = crc8_07((uint8_t[]){rx_label, rx_value}, 2);
      bool ok = (calc == byteIn);
      rx_state = HUNT_SYNC1;     // always return to hunt
      if (ok) { out = {rx_label, rx_value}; return true; }
      return false;
    }
  }
  // Safety fallback
  rx_state = HUNT_SYNC1;
  return false;
}
//---- UART Protocol

// ===================== OLED CONFIG (U8g2) =====================
// Hardware SPI pins on Blue Pill: SCK=PA5, MOSI=PA7 (MISO not used by OLED)
// Choose your control pins below (change if needed):
static const uint8_t OLED_CS   = PB12;  // Chip Select
static const uint8_t OLED_DC   = PB1;   // Data/Command
static const uint8_t OLED_RST  = PB0;   // Reset

// SSD1309 128x64 over 4-wire HW SPI (common for many 2.42"/2.7" OLEDs)
U8G2_SSD1309_128X64_NONAME2_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ OLED_CS, /* dc=*/ OLED_DC, /* reset=*/ OLED_RST);

const uint32_t BAUD = 115200;
const uint8_t LEDPIN = PC13;   // BlackPill LED (active-low on many boards)

uint32_t channelRising, channelFalling;
volatile uint32_t FrequencyMeasured, DutycycleMeasured, LastPeriodCapture = 0, CurrentCapture, HighStateMeasured;
uint32_t input_freq = 0;
volatile uint32_t rolloverCompareCount = 0;
HardwareTimer *MyTim;

/**
    @brief  Input capture interrupt callback : Compute frequency and dutycycle of input signal

*/
void TIMINPUT_Capture_Rising_IT_callback(void)
{
  CurrentCapture = MyTim->getCaptureCompare(channelRising);
  /* frequency computation */
  if (CurrentCapture > LastPeriodCapture)
  {
    FrequencyMeasured = input_freq / (CurrentCapture - LastPeriodCapture);
    DutycycleMeasured = (HighStateMeasured * 100) / (CurrentCapture - LastPeriodCapture);
  }
  else if (CurrentCapture <= LastPeriodCapture)
  {
    /* 0x10000 is max overflow value */
    FrequencyMeasured = input_freq / (0x10000 + CurrentCapture - LastPeriodCapture);
    DutycycleMeasured = (HighStateMeasured * 100) / (0x10000 + CurrentCapture - LastPeriodCapture);
  }

  LastPeriodCapture = CurrentCapture;
  rolloverCompareCount = 0;
}

/* In case of timer rollover, frequency is too low to be measured set values to 0
   To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
void Rollover_IT_callback(void)
{
  rolloverCompareCount++;

  if (rolloverCompareCount > 1)
  {
    FrequencyMeasured = 0;
    DutycycleMeasured = 0;
  }
}

/**
    @brief  Input capture interrupt callback : Compute frequency and dutycycle of input signal

*/
void TIMINPUT_Capture_Falling_IT_callback(void)
{
  /* prepare DutyCycle computation */
  CurrentCapture = MyTim->getCaptureCompare(channelFalling);

  if (CurrentCapture > LastPeriodCapture)
  {
    HighStateMeasured = CurrentCapture - LastPeriodCapture;
  }
  else if (CurrentCapture <= LastPeriodCapture)
  {
    /* 0x1000 is max overflow value */
    HighStateMeasured = 0x10000 + CurrentCapture - LastPeriodCapture;
  }
}

// ---------- Tile helpers (8x8 tiles) ----------
static inline uint8_t px2tile(uint8_t px)       { return px >> 3; }          // floor(px/8)
static inline uint8_t px2tile_ceil(uint8_t px)  { return (px + 7) >> 3; }     // ceil(px/8)

// replaces pushArea()
static inline void pushAreaPx(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
  uint8_t tx = x >> 3;                         // tile x
  uint8_t ty = y >> 3;                         // tile y
  uint8_t tw = ((x & 7) + w + 7) >> 3;         // tile width  (ceil((x%8 + w)/8))
  uint8_t th = ((y & 7) + h + 7) >> 3;         // tile height (ceil((y%8 + h)/8))
  u8g2.updateDisplayArea(tx, ty, tw, th);
}

// ---------- Layout (adjust to taste) ----------
static const uint8_t W = 128, H = 64;

// Top row (two small fields)
static const uint8_t TMP_X=2,  TMP_Y=0,  TMP_W=62, TMP_H=16;     // "T: 23.4C"
static const uint8_t BAT_X=66, BAT_Y=0,  BAT_W=60, BAT_H=16;     // "Bat: 85%"

// Middle/bottom rows
static const uint8_t RPM_X=2,  RPM_Y=18, RPM_W=124, RPM_H=16;    // "RPM: 1234"
static const uint8_t DC_X=128-56,   DC_Y=64-16, DC_W=56, DC_H=16;       // triangle area

// ---------- Common helpers ----------
static void clearRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
  u8g2.setDrawColor(0);
  u8g2.drawBox(x, y, w, h);
  u8g2.setDrawColor(1);  // restore for normal drawing
}

// Choose a small, readable font once per widget draw.
// (Change to any u8g2 font you prefer.)
static void setSmallFont() { u8g2.setFont(u8g2_font_6x12_tf); }

// Center a string horizontally inside a rect; returns baseline y centered vertically for given rect.
static int16_t centeredBaselineY(uint8_t y, uint8_t h) {
  // Using font ascent for baseline; visually centered for 6x12 class fonts
  return y + (h + u8g2.getAscent())/2 - 1;
}

// ============= 1) Duty Cycle =============
void renderDutyCycle(uint8_t duty /*0..100*/) {
  if (duty > 100) duty = 100;

  // ---- Geometry (half-width triangle) ----
  const uint8_t triW  = 54;                   // half of 128 px
  const uint8_t barH  = 14;                   // tile-friendly height
  const uint8_t barY  = DC_Y + (DC_H - barH)/2;
  const uint8_t triX  = DC_X;                 // left-aligned; center if you prefer:
  // const uint8_t triX = DC_X + (DC_W - triW)/2;

  // Clear only this region
  clearRect(triX, barY, triW, barH);
  u8g2.setDrawColor(1);

  // ---- Fill: left -> right, mirrored triangle ----
  // x_cut scales with duty (width-proportional). For area-proportional, use sqrt mapping.
  uint16_t x_cut = (uint16_t)((uint32_t)triW * duty / 100);

  const uint8_t y_bottom = barY + barH - 1;
  for (uint16_t dx = 0; dx < x_cut; ++dx) {
    // Hypotenuse runs from (triX+triW-1, barY) to (triX, y_bottom)
    // For a given x = triX + dx, the top y is higher when dx is small.
    uint8_t y_top = barY + (uint16_t)((uint32_t)barH * (triW - 1 - dx) / triW);
    u8g2.drawVLine(triX + dx, y_top, y_bottom - y_top + 1);
  }

  // ---- Outline (right-angled at the right) ----
  u8g2.drawVLine(triX + triW - 1, barY, barH);             // right side
  u8g2.drawHLine(triX, y_bottom+1, triW);                    // bottom
  u8g2.drawLine (triX + triW - 1, barY, triX, y_bottom);   // hypotenuse

  // Push only this area
  pushAreaPx(triX, barY, triW, barH);
}


// ============= 2) Battery % =============
void renderBattery(ParamState p) {
  clearRect(BAT_X, BAT_Y, BAT_W, BAT_H);
  setSmallFont();

  char s[16];

  if (p.has) {
    snprintf(s, sizeof(s), "Bat: %u%%", p.val);
  } else {
    snprintf(s, sizeof(s), "Bat: ??%%");
  }

  int16_t ty = centeredBaselineY(BAT_Y, BAT_H);

  u8g2.drawStr(BAT_X, ty, s);
  pushAreaPx(BAT_X, BAT_Y, BAT_W, BAT_H);
}

// ============= 3) Temperature (°C) =============
void renderTemperature(ParamState t) {
  clearRect(TMP_X, TMP_Y, TMP_W, TMP_H);
  setSmallFont();

  char s[20];
  
  if (t.has) {
    snprintf(s, sizeof(s), "T: %uC", t.val);
  } else {
    snprintf(s, sizeof(s), "T: ???");
  }
  int16_t ty = centeredBaselineY(TMP_Y, TMP_H);

  u8g2.drawStr(TMP_X, ty, s);
  pushAreaPx(TMP_X, TMP_Y, TMP_W, TMP_H);
}

// ============= 4) RPM =============
void renderRPM(ParamState r) {
  clearRect(RPM_X, RPM_Y, RPM_W, RPM_H);
  setSmallFont();

  char s[24];

  if (r.has) {
    uint16_t Scaledrpm = r.val * 15;
    snprintf(s, sizeof(s), "RPM: %u", Scaledrpm);
  } else {
    snprintf(s, sizeof(s), "RPM: ???");
  }
  int16_t ty = centeredBaselineY(RPM_Y, RPM_H);

  u8g2.drawStr(RPM_X, ty, s);
  pushAreaPx(RPM_X, RPM_Y, RPM_W, RPM_H);
}

// Packs and sends one frame: [0x55, 0xAA, L_DCME, payload, crc]
static void sendDC_MotorEnable(uint8_t duty_0_100, bool motorEnable) {
  // Saturate duty into 0..100 and pack into 7 bits
  if (duty_0_100 > 100) duty_0_100 = 100;

  uint8_t payload = (uint8_t)(duty_0_100 & 0x7F);
  if (motorEnable) payload |= 0x80;  // set MSB if enabled

  // Compute CRC over [label, value] with your existing crc8_07()
  uint8_t label = L_DCME;
  uint8_t buf[2] = { label, payload };
  uint8_t crc = crc8_07(buf, 2);

  // Emit the frame over the same UART you already use
  uint8_t frame[5] = { SYNC1, SYNC2, label, payload, crc };
  Serial2.write(frame, sizeof(frame));

  Serial.print(millis());
  Serial.print(" --> ");
  Serial.print("Sent frame: DC=");
  Serial.print( payload & 0x7F );
  Serial.print("Motor: ");
  Serial.println( (payload & 0b10000000)?"Enabled":"Disabled");
}

void setup() {
  // OLED init
  u8g2.begin();
  u8g2.setBusClock(8000000);  // SPI example
  u8g2.clearBuffer();
  u8g2.sendBuffer();          // blank once

  Serial.begin(115200);        // USB debug
  Serial2.begin(115200);       // UART from Board B
  delay(500);
  Serial.println("Receiver is up");

  // Automatically retrieve TIM instance and channelRising associated to pin
  // This is used to be compatible with all STM32 series automatically.
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);
  channelRising = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));

  // channelRisings come by pair for TIMER_INPUT_FREQ_DUTY_MEASUREMENT mode:
  // channelRising1 is associated to channelFalling and channelRising3 is associated with channelRising4
  switch (channelRising) {
    case 1:
      channelFalling = 2;
      break;
    case 2:
      channelFalling = 1;
      break;
    case 3:
      channelFalling = 4;
      break;
    case 4:
      channelFalling = 3;
      break;
  }

  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
  MyTim = new HardwareTimer(Instance);

  // Configure rising edge detection to measure frequency
  MyTim->setMode(channelRising, TIMER_INPUT_FREQ_DUTY_MEASUREMENT, pin);

  // With a PrescalerFactor = 1, the minimum frequency value to measure is : TIM counter clock / CCR MAX
  //  = (SystemCoreClock) / 65535
  // Example on Nucleo_L476RG with systemClock at 80MHz, the minimum frequency is around 1,2 khz
  // To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision.
  // The maximum frequency depends on processing of both interruptions and thus depend on board used
  // Example on Nucleo_L476RG with systemClock at 80MHz the interruptions processing is around 10 microseconds and thus Max frequency is around 100kHz
  uint32_t PrescalerFactor = 2;
  MyTim->setPrescaleFactor(PrescalerFactor);
  MyTim->setOverflow(0x10000); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
  MyTim->attachInterrupt(channelRising, TIMINPUT_Capture_Rising_IT_callback);
  MyTim->attachInterrupt(channelFalling, TIMINPUT_Capture_Falling_IT_callback);
  MyTim->attachInterrupt(Rollover_IT_callback);

  MyTim->resume();

  // Compute this scale factor only once
  input_freq = MyTim->getTimerClkFreq() / MyTim->getPrescaleFactor();

  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW); // LED on (active low)

  // Initial splash
  renderDutyCycle(0);
}

uint32_t PreviousDutycycle = 0;
uint32_t LastDCDebugPrintTimeStamp = 0;
uint32_t LastDCOLEDTimeStamp = 0;

static void sendDC_MotorEnable(uint8_t duty_0_100, bool motorEnable);

void loop() {

  if ( (millis() - LastDCDebugPrintTimeStamp) > 500UL ) {
    Serial.print("D = ");
    Serial.println(DutycycleMeasured);
    digitalWrite(LEDPIN, !digitalRead(LEDPIN)); // Toggle LED
    LastDCDebugPrintTimeStamp = millis();
  }
  
  if ( millis() - LastDCOLEDTimeStamp > 5 ) {
    if (DutycycleMeasured != PreviousDutycycle) {
      renderDutyCycle(DutycycleMeasured);
    }
    sendDC_MotorEnable(DutycycleMeasured, 1);
    LastDCOLEDTimeStamp = millis();
  }

  // 1) Drain at most ONE byte of UART work this iteration
  RxFrame f;
  if (uart_rx_step(f)) {
    Serial.print(millis());
    Serial.print(" --> ");
    Serial.print(labelName(f.label));
    Serial.print(" = ");
    Serial.println(f.value);

    // Record that we saw this parameter now
    noteParamSeen(f.label, f.value);
  }
  paramTimeouts_step(); // invalidate stale values -> becomes "NULL"

  renderBattery(g_batt);
  renderTemperature(g_temp);
  renderRPM(g_rpm);

  //delay( 10 );
  }
