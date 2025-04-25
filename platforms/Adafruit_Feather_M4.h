/*
 * This file contains definitions for running the IcosaLogic inverter on the Adafruit Feather M4
 * board.
 */
 
#ifdef _VARIANT_FEATHER_M4_
#define INVERTER_TIMER 1
  // Uses a 64-pin ATSAMD51J19A
              
  const InverterTimerCfg FeatherTcc06Cfg = {
    TCC0, 0, 6, false, 0, 2, 0, 2,              // TCC0 IOSET6, WO[0..3] conflicts with TCC1 IOSET1 below
    {-1, -1, -1, -1, 1, 0, -1, -1},             //              WO[4..5] usable
    {false, false, false, false, true, true, false, false}, //  WO[6..7] undefined
    {false, false, false, false},
    {false, false, false, false, true, true},
    {false, false}};
    
  const InverterTimerCfg FeatherTcc11Cfg = {    // TCC1 IOSET1, all good
    TCC1, 1, 1, true, 2, 4, 4, 8,
    { 5, 25, 6, 9, 10, 11, 12, 13},
    {true, true, true, true, true, true, true, true},
    {true, true, true, true},
    {true, true, true, true, true, true},
    {true, true}};
    
  const InverterTimerCfg* timerCfg[5] = {NULL, &FeatherTcc06Cfg, NULL, &FeatherTcc11Cfg, NULL};
  
  static const int numAdcConfigs = 8;
  I20PinData InverterAdcCfg[numAdcConfigs] = {
                                            {"PA02", 14, PIO_ANALOG,     0,  0},  // A0 : ADC0
                                            {"PA05", 15, PIO_ANALOG,     0,  5},  // A1 : ADC0
                                            {"PB08", 16, PIO_ANALOG,     0,  2},  // A2 : ADC0
                                            {"PB08", 40, PIO_ANALOG,     1,  0},  // A2 : ADC1
                                            {"PB09", 17, PIO_ANALOG,     0,  3},  // A3 : ADC0
                                            {"PB09", 41, PIO_ANALOG,     1,  1},  // A3 : ADC1
                                            {"PA04", 18, PIO_ANALOG,     0,  4},  // A4 : ADC0
                                            {"PA06", 19, PIO_ANALOG,     0,  6}}; // A5 : ADC0

// ADC config values for Adafruit Feather M4
// Must match InverterAdcCfg[] above

#define I20_PIN_A0_ADC0 0
#define I20_PIN_A1_ADC0 1
#define I20_PIN_A2_ADC0 2
#define I20_PIN_A2_ADC1 3
#define I20_PIN_A3_ADC0 4
#define I20_PIN_A3_ADC1 5
#define I20_PIN_A4_ADC0 6
#define I20_PIN_A5_ADC0 7
#define I20_PIN_GND     100
#define I20_PIN_NONE    101
  /* Not Yet
  static const int numGclkConfigs = 7;
  const I20PinData FeatherM4GclkioCfg[numGclkConfigs] = 
                                        {{"PA14",  4, PIO_NOT_A_PIN, 0, 0, 0},
                                         {"PA16",  5, PIO_NOT_A_PIN, 0, 0, 2},
                                         {"PA17", 25, PIO_NOT_A_PIN, 0, 0, 3},   // SCK
                                         {"PB16",  1, PIO_NOT_A_PIN, 0, 0, 2},
                                         {"PB17",  0, PIO_NOT_A_PIN, 0, 0, 3},
                                         {"PB22", 23, PIO_NOT_A_PIN, 0, 0, 0},   // MISO
                                         {"PB23", 24, PIO_NOT_A_PIN, 0, 0, 1}};  // MOSI
  */
  
#endif // _VARIANT_FEATHER_M4_
