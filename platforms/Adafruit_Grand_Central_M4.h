/*
 * This file contains definitions for running the IcosaLogic inverter on the Adafruit Grand
 * Central M4 board.
 */
 
#ifdef _VARIANT_GRAND_CENTRAL_M4_
#define INVERTER_TIMER 3
  // Metro Grand Central
  // Uses a 128-pin ATSAMD51P20A
  
  const InverterTimerCfg MetGcTcc02Cfg    = {
    TCC0, 0, 2, true, 2, 4, 4, 8,  // TCC0 IOSET2
    {48, 51, 52, 53, 50, 22, 16, 17},
    {true, true, true, true, true, true, true, true},
    {true, true, true, true},
    {true, true, true, true, true, true},
    {true, true}};
    
  const InverterTimerCfg MetGcTcc03Cfg    = {
    TCC0, 0, 3, false, 2, 4, 4, 8,  // TCC0 IOSET3 -- Conflict with TCC1 IOSET1
    {45, 44, 41, 40, 43, 42, 35, 34},
    {true, true, true, true, true, true, true, true},
    {true, true, true, true},
    {true, true, true, true, true, true},
    {true, true}};
    
  const InverterTimerCfg MetGcTcc11Cfg    = {
    TCC1, 1, 1, true, 2, 4, 4, 8,  // TCC1 IOSET1 -- Conflict with TCC0 IOSET3
    {37, 36, 35, 34, 33, 32, 31, 30},
    {true, true, true, true, true, true, true, true},
    {true, true, true, true},
    {true, true, true, true, true, true},
    {true, true}};

  const InverterTimerCfg* timerCfg[5] = {NULL, &MetGcTcc02Cfg, &MetGcTcc03Cfg, &MetGcTcc11Cfg, NULL};
  
  // !NEG below flags pins that cannot be used as a negative pin for a differential ADC reading.
  // Only pins with a channel number <= 7 can be used as the negative pin.
  
  static const int numAdcConfigs = 18;
  I20PinData InverterAdcCfg[numAdcConfigs] = {
                                            {"PA02", 67, PIO_ANALOG,     0,  0},  // A0  : ADC0
                                            {"PA05", 68, PIO_ANALOG,     0,  5},  // A1  : ADC0
                                            {"PB03", 69, PIO_ANALOG,     0, 15},  // A2  : ADC0 !NEG
                                            {"PC00", 70, PIO_ANALOG,     1, 10},  // A3  : ADC1 !NEG
                                            {"PC01", 71, PIO_ANALOG,     1, 11},  // A4  : ADC1 !NEG
                                            {"PC02", 72, PIO_ANALOG,     1,  4},  // A5  : ADC1
                                            {"PC03", 73, PIO_ANALOG,     1,  5},  // A6  : ADC1
                                            {"PB04", 74, PIO_ANALOG,     1,  6},  // A7  : ADC1
                                            {"PB05", 54, PIO_ANALOG,     1,  7},  // A8  : ADC1
                                            {"PB06", 55, PIO_ANALOG,     1,  8},  // A9  : ADC1 !NEG
                                            {"PB07", 56, PIO_ANALOG,     1,  9},  // A10 : ADC1 !NEG
                                            {"PB08", 57, PIO_ANALOG,     0,  2},  // A11 : ADC0
                                            {"PB08", 57, PIO_ANALOG,     1,  0},  // A11 : ADC1
                                            {"PB09", 58, PIO_ANALOG,     0,  3},  // A12 : ADC0
                                            {"PB09", 58, PIO_ANALOG,     1,  1},  // A12 : ADC1
                                            {"PA04", 59, PIO_ANALOG,     0,  4},  // A13 : ADC0
                                            {"PA06", 60, PIO_ANALOG,     0,  6},  // A14 : ADC0
                                            {"PA07", 61, PIO_ANALOG,     0,  7}}; // A15 : ADC0

  I20PinData adcVRefPin = {"PA03", 84, PIO_ANALOG, 0, 0};  // AREF

// ADC config values for Adafruit Grand Central M4
// These are indices into InverterAdcCfg[] above

#define I20_PIN_A0_ADC0 0
#define I20_PIN_A1_ADC0 1
#define I20_PIN_A2_ADC0 2
#define I20_PIN_A3_ADC1 3
#define I20_PIN_A4_ADC1 4
#define I20_PIN_A5_ADC1 5
#define I20_PIN_A6_ADC1 6
#define I20_PIN_A7_ADC1 7
#define I20_PIN_A8_ADC1 8
#define I20_PIN_A9_ADC1 9
#define I20_PIN_A10_ADC1 10
#define I20_PIN_A11_ADC0 11
#define I20_PIN_A11_ADC1 12
#define I20_PIN_A12_ADC0 13
#define I20_PIN_A12_ADC1 14
#define I20_PIN_A13_ADC0 15
#define I20_PIN_A14_ADC0 16
#define I20_PIN_A15_ADC0 17
#define I20_PIN_GND     100
#define I20_PIN_NONE    101
  
  /*
  static const int numGclkConfigs = 16;
  const I20PinData MetGcGclkioCfg[numGclkConfigs] = 
                                    {{"PA14", 28, PIO_NOT_A_PIN, 0, 0, 0},
                                     {"PA15", 23, PIO_NOT_A_PIN, 0, 0, 1},
                                     {"PA16", 37, PIO_NOT_A_PIN, 0, 0, 2},
                                     {"PA17", 36, PIO_NOT_A_PIN, 0, 0, 3},
                                     {"PB12", 18, PIO_NOT_A_PIN, 0, 0, 6},
                                     {"PB13", 19, PIO_NOT_A_PIN, 0, 0, 7},
                                     {"PB14", 39, PIO_NOT_A_PIN, 0, 0, 0},
                                     {"PB15", 38, PIO_NOT_A_PIN, 0, 0, 1},
                                     {"PB16", 14, PIO_NOT_A_PIN, 0, 0, 2},
                                     {"PB17", 15, PIO_NOT_A_PIN, 0, 0, 3},
                                     {"PB18",  8, PIO_NOT_A_PIN, 0, 0, 4},
                                     {"PB19", 29, PIO_NOT_A_PIN, 0, 0, 5},
                                     {"PB20", 20, PIO_NOT_A_PIN, 0, 0, 6},
                                     {"PB21", 21, PIO_NOT_A_PIN, 0, 0, 7},
                                     {"PB22", 10, PIO_NOT_A_PIN, 0, 0, 0},
                                     {"PB23", 11, PIO_NOT_A_PIN, 0, 0, 1}};
  */
  
#endif // _VARIANT_GRAND_CENTRAL_M4_

