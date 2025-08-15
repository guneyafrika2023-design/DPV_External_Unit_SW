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

// ===================== OLED CONFIG (U8g2) =====================
// Hardware SPI pins on Blue Pill: SCK=PA5, MOSI=PA7 (MISO not used by OLED)
// Choose your control pins below (change if needed):
static const uint8_t OLED_CS   = PB12;  // Chip Select
static const uint8_t OLED_DC   = PB1;   // Data/Command
static const uint8_t OLED_RST  = PB0;   // Reset

// SSD1309 128x64 over 4-wire HW SPI (common for many 2.42"/2.7" OLEDs)
U8G2_SSD1309_128X64_NONAME2_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ OLED_CS, /* dc=*/ OLED_DC, /* reset=*/ OLED_RST);

// Force a UART on USART2 pins: RX=PA3, TX=PA2
HardwareSerial Serial2(PA3, PA2);

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

void setup() {
  Serial2.begin(BAUD);                   // Start USART2 on PA2/PA3
  delay(100);                            // Give the terminal a moment
  Serial2.println("\r\nUSART2 ready on PA2/PA3 @115200 8N1");
  
  // OLED init
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x12_tf); // Compact readable font

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
  drawOLED(0);
}

void loop() {
  uint32_t PreviousDutycycle = 0;

  //Serial2.print((String)"F = " + FrequencyMeasured);
  //Serial2.println((String)"    D = " + DutycycleMeasured + "%\n");
  //digitalWrite(LEDPIN, !digitalRead(LEDPIN)); // Toggle LED
  
  if (DutycycleMeasured != PreviousDutycycle) {
    drawOLED(DutycycleMeasured);
  }
  delay(2);
}

// ===================== OLED RENDERING =====================
void drawOLED(uint16_t duty) {
  // Progress bar geometry
  const int16_t W = 128;
  const int16_t H = 64;
  const int16_t marginX = 8;
  const int16_t barY = 40;
  const int16_t barH = 14;
  const int16_t barW = W - 2 * marginX;
  int16_t fillW = (int16_t)(( (uint32_t)barW * duty ) / 100);

  u8g2.firstPage();
  do {
    // Duty text
    u8g2.setCursor(8, 28);
    u8g2.print("DC: ");
    u8g2.print(duty);
    u8g2.print("%");
    // Bar frame
    u8g2.drawFrame(marginX, barY, barW, barH);
    // Fill (stretch/shrink with duty)
    if (fillW > 0) {
      u8g2.drawBox(marginX + 1, barY + 1, fillW - (fillW>1?1:0), barH - 2);
    }

    // End caps as simple chevrons
    u8g2.drawTriangle(marginX-3, barY+barH/2, marginX, barY, marginX, barY+barH);
    u8g2.drawTriangle(marginX+barW+3, barY+barH/2, marginX+barW, barY, marginX+barW, barY+barH);
  } while (u8g2.nextPage());
}
