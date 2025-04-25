/*
 * This file contains definitions for running the IcosaLogic inverter on the Adafruit Metro M4
 * board.
 * 
 * Incomplete -- see ADC TODO below
 */
 
#ifdef _VARIANT_METRO_M4_
#define INVERTER_TIMER 2
  // Uses a 64-pin ATSAMD51J19A
  // Note conflict between TCC0_B and TCC1.  No conflicts between TCC0_A and TCC1
  // These have not been verified
  const InverterTimerCfg MetroTcc1Cfg   = {
    TCC1, 1, TCC1_CC_NUM, true, 2, 4, 8,  // TCC1 IOSET1  Preferred TCC
                                          {{{"PA16", 13, PIO_TIMER_ALT},     // WO[0] L1 Q1
                                            {"PA21",  8, PIO_TIMER_ALT},     // WO[5] L1 Q2
                                            {"PA20",  9, PIO_TIMER_ALT},     // WO[4] L1 Q3
                                            {"PA17", 12, PIO_TIMER_ALT}},    // WO[1] L1 Q4  SCK
                                           {{"PA18", 10, PIO_TIMER_ALT},     // WO[2] L2 Q1
                                            {"PA23",  0, PIO_TIMER_ALT},     // WO[7] L2 Q2  LED!!!
                                            {"PA22",  1, PIO_TIMER_ALT},     // WO[6] L2 Q3
                                            {"PA19", 11, PIO_TIMER_ALT}}}};  // WO[3] L2 Q4

  const InverterTimerCfg* timerCfg[5] = {NULL, NULL, NULL, &MetroTcc1Cfg, NULL};
					 
  static const int numAdcConfigs = 8;
  I20PinData InverterAdcCfg[numAdcConfigs] = {
                                            {"PA02", 14, PIO_ANALOG,     0,  0},  // A0 : ADC0
                                            {"PA05", 15, PIO_ANALOG,     0,  5},  // A1 : ADC0
                                            {"PA06", 16, PIO_ANALOG,     0,  6},  // A2 : ADC0
                                            {"PA04", 17, PIO_ANALOG,     0,  4},  // A3 : ADC0
                                            {"PB08", 18, PIO_ANALOG,     0,  2},  // A4 : ADC0
                                            {"PB08", 18, PIO_ANALOG,     1,  0},  // A4 : ADC1
                                            {"PB09", 19, PIO_ANALOG,     0,  3},  // A5 : ADC0
                                            {"PB09", 19, PIO_ANALOG,     1,  1}}; // A5 : ADC1

// ADC config values for Adafruit Metro M4
// These are indices into InverterAdcCfg[] above

#define I20_PIN_A0_ADC0 0
#define I20_PIN_A1_ADC0 1
#define I20_PIN_A2_ADC0 2
#define I20_PIN_A3_ADC0 3
#define I20_PIN_A4_ADC0 4
#define I20_PIN_A4_ADC1 5
#define I20_PIN_A5_ADC0 6
#define I20_PIN_A5_ADC1 7
#define I20_PIN_GND     100
#define I20_PIN_NONE    101
    
  
  static const int numGclkConfigs = 10;
  const I20PinData MetroM4GclkioCfg[numGclkConfigs] = 
                                      {{"PA14", 24, PIO_NOT_A_PIN, 0, 0, 0},   // MISO
                                       {"PA16", 13, PIO_NOT_A_PIN, 0, 0, 2},
                                       {"PA17", 12, PIO_NOT_A_PIN, 0, 0, 3},
                                       {"PB12",  7, PIO_NOT_A_PIN, 0, 0, 6},
                                       {"PB13",  4, PIO_NOT_A_PIN, 0, 0, 7},
                                       {"PB14",  5, PIO_NOT_A_PIN, 0, 0, 0},
                                       {"PB15",  6, PIO_NOT_A_PIN, 0, 0, 1},
                                       {"PB16",  3, PIO_NOT_A_PIN, 0, 0, 2},
                                       {"PB17",  2, PIO_NOT_A_PIN, 0, 0, 3},
                                       {"PB22", 40, PIO_NOT_A_PIN, 0, 0, 0}};  // NeoPixel
  
 #endif // _VARIANT_METRO_M4_
