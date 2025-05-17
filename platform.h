/*
 * Include the board-specific files for the inverter.
 * Only the board set by the Arduino UI will actually be included.
 * 
 * External VREF constraints
 * - VREFA - PA03
 * - VREFB - PA04
 * - VREFC - PA06
 */

#include "platforms/Adafruit_Feather_M4.h"
#include "platforms/Adafruit_Grand_Central_M4.h"
#include "platforms/Adafruit_ItsyBitsy_M4.h"
#include "platforms/Adafruit_Metro_M4.h"

// See also SparkFun Thing Plus -- SAMD51

#ifndef INVERTER_TIMER
  Error: Not a supported SAMD51 platform
#endif
