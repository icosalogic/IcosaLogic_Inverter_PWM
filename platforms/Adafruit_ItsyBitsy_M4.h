/*
 * This file contains definitions for running the IcosaLogic inverter on the Adafruit ItsyBitsy M4
 * board.
 */
 
#ifdef _VARIANT_ITSYBITSY_M4_
#define INVERTER_TIMER 4
// uses a 48-pin ATSAMD51G19A
                                            
  const InverterTimerCfg ItsyBitsyTcc11Cfg = {  // TCC1 IOSET1, all good
    TCC1, 1, 1, true, 2, 4, 4, 8,
    { 0, 1, 7, 9, 10, 11, 13, 12},
    {true, true, true, true, true, true, true, true},
    {true, true, true, true},
    {true, true, true, true, true, true},
    {true, true}};

  const InverterTimerCfg* timerCfg[5] = {NULL, NULL, NULL, &ItsyBitsyTcc11Cfg, NULL};
  
  static const int numAdcConfigs = 8;
  I20PinData InverterAdcCfg[numAdcConfigs] = {
                                            {"PA02", 14, PIO_ANALOG,     0,  0},  // A0 : ADC0
                                            {"PA05", 15, PIO_ANALOG,     0,  5},  // A1 : ADC0
                                            {"PB08", 16, PIO_ANALOG,     0,  2},  // A2 : ADC0
                                            {"PB08", 16, PIO_ANALOG,     1,  0},  // A2 : ADC1
                                            {"PB09", 17, PIO_ANALOG,     0,  3},  // A3 : ADC0
                                            {"PB09", 17, PIO_ANALOG,     1,  1},  // A3 : ADC1
                                            {"PA04", 18, PIO_ANALOG,     0,  4},  // A4 : ADC0
                                            {"PA06", 19, PIO_ANALOG,     0,  6}}; // A5 : ADC0

// ADC config values for Adafruit ItsyBitsy M4
// These are indices into InverterAdcCfg[] above

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

/*inverterAdcCf
  static const int numGclkConfigs = 6;
  const I20PinData ItsyBitsyGclkioCfg[numGclkConfigs] = 
                                        {{"PA14",  4, PIO_NOT_A_PIN, 0, 0, 0},
                                         {"PA15",  5, PIO_NOT_A_PIN, 0, 0, 1},
                                         {"PA16",  0, PIO_NOT_A_PIN, 0, 0, 2},
                                         {"PA17",  1, PIO_NOT_A_PIN, 0, 0, 3},
                                         {"PB22",  3, PIO_NOT_A_PIN, 0, 0, 0},
                                         {"PB23", 23, PIO_NOT_A_PIN, 0, 0, 1}};
*/

#endif // _VARIANT_ITSYBITSY_M4_

