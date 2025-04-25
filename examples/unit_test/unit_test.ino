/*
 * Unit test client for the IcosaLogic inverter library.
 * This test exercises most of the basic functionality of the
 * inverter, and should run on every board.
 * 
 * Board-specific tests may be found in other examples.
 */

int firstVariable = 4321;

#include <Arduino.h>
#include <IcosaLogic_Inverter_PWM.h>
#include <SAMD51_Dumpster.h>

// Basic inverter configuration values
const I20InvArch        invArch        = I20_T_TYPE;
const I20HalfWaveSignal hws            = I20_HWS_NONE;
const uint8_t           numLines       = 2;
const uint16_t          outRmsVoltage  = 120;
const uint8_t           outputFreq     = 60;
const uint32_t          pwmFreq        = 6000;
const uint16_t          deadTimeNs     = 50;
const uint8_t           adcNumBits     = 12;
const uint16_t          adcPrescale    = 4;
const uint16_t          adcSampleTicks = 24;

// Define ADC pins for different boards

#if defined(_VARIANT_FEATHER_M4_) || defined(_VARIANT_ITSYBITSY_M4_)

#define P_X0 I20_PIN_A0_ADC0
#define P_X1 I20_PIN_A1_ADC0
#define P_X2 I20_PIN_A2_ADC1
#define P_X3 I20_PIN_A3_ADC1
#define P_X4 I20_PIN_A4_ADC0
#define P_X5 I20_PIN_A5_ADC0

#elif defined(_VARIANT_GRAND_CENTRAL_M4_)

#define P_X0 I20_PIN_A0_ADC0
#define P_X1 I20_PIN_A1_ADC0
#define P_X2 I20_PIN_A3_ADC1
#define P_X3 I20_PIN_A4_ADC1
#define P_X4 I20_PIN_A11_ADC0
#define P_X5 I20_PIN_A12_ADC0

#endif

// A basic set of feedback signal definitions
I20FeedbackSignal vLine1   = {P_X4, I20_PIN_GND,  0, false, I20_LINE1_VOLTAGE,   820000,  7400, 0.0};
I20FeedbackSignal vLine2   = {P_X2, I20_PIN_GND,  0, false, I20_LINE2_VOLTAGE,   820000,  7400, 0.0};
I20FeedbackSignal aLine1   = {P_X5, I20_PIN_GND,  1, false, I20_LINE1_CURRENT,        0,     0, 0.025177};
I20FeedbackSignal aLine2   = {P_X3, I20_PIN_GND,  1, true,  I20_LINE2_CURRENT,        0,     0, 0.025177};
I20FeedbackSignal vBattTop = {P_X0, I20_PIN_GND,  2, false, I20_BATTTOP_VOLTAGE, 820000,  4460, 0.0};
I20FeedbackSignal vBattMid = {P_X1, I20_PIN_GND,  2, false, I20_BATTMID_VOLTAGE, 820000,  4460, 0.0};

I20Feedback feedback = {{&vLine1, &vLine2, &aLine1, &aLine2, &vBattTop, &vBattMid }};

I20InputParams defaultParams = {invArch,         // inverter architecture
                                hws,             // half wave signal to generate
                                outRmsVoltage,   // RMS voltage of output lines
                                numLines,        // number of output lines
                                outputFreq,      // output frequency, either 50 or 60 Hz
                                pwmFreq,         // PWM frequency
                                I20_PS_TCC1,     // Primary TCC configuration to use
                                I20_PS_TCC0,     // Secondary TCC configuration to use
                                deadTimeNs,      // dead time between MOSFET transitions
                                adcNumBits,      // number of bits in an ADC reading
                                adcPrescale,     // ADC clock prescale value
                                adcSampleTicks,  // ADC clock ticks to hold sample
                                AR_DEFAULT,      // ADC reference
                                &feedback,       // voltage and current feedback
                              };

I20InputParams inParams;

IcosaLogic_Inverter_PWM inverter;     // inverter object

SAMD51_Dumpster ilsd;

// IRQ counters
volatile uint32_t numTcc0_0_IrqRaw = 0;
volatile uint32_t numTcc1_0_IrqRaw = 0;
volatile uint32_t numAdc0_1_IrqRaw = 0;
volatile uint32_t numAdc1_1_IrqRaw = 0;

// When > 0, this limits how many sine waves the inverter will generate before stopping
uint32_t numWaveLimit = 0;

volatile bool inverterRunning = false;
volatile uint32_t startTicks = 0;
volatile uint32_t stopTicks = 0;


/*
 * Initialize the inverter.
 */
void setup() {
  setupSerial();

  dumpMemoryAddresses();
  
  Serial.println(" ");
  
  ilsd.begin(true);
  
  runUnitTests();
  
  Serial.println(" ");
}

/**
 * Set up the serial interface.  Delay units are in milliseconds.
 */
void setupSerial() {
  Serial.begin(115200);
  
  const int delayIncr = 100;
  const int maxDelay = 2000;
  for (int i = 0; i < maxDelay; i += delayIncr) {
    delay(delayIncr);
    Serial.print(".");
    Serial.flush();
  }
  
  Serial.println(" ");
  Serial.println("\n\nIcosaLogic_Inverter_PWM Unit Tests\n");
}

/*
 * Dump an address of code, memory and the stack, just to get a feel for where the
 * segments are being located.  According to the SAMD51 datasheet:
 *     code/flash:  00000000 - flash size
 *     SRAM:        20000000 - 20040000
 *     peripherals: 40000000 - 48000000
 *     system:      e0000000 - ffffffff
 */
void dumpMemoryAddresses() {
  int32_t x = 1234;
  Serial.printf("Memory map:\n    Code:  %08x\n    Data:  %08x\n    Stack: %08x\n",
                (uint32_t) &setup, (uint32_t) &firstVariable, (uint32_t) &x);
}

/*
 * Checks for errors in the inverter.  If the error count is > 0, prints the
 * error messages.  Returns the error count.
 */
unsigned int checkErrors() {
  unsigned int numErrors = inverter.getNumErrors();
  if (numErrors > 0) {
    Serial.printf("Inverter had %d errors:\n", numErrors);
    for (int i = 1; i <= numErrors; i++) {
      const char* errMsg = inverter.getErrorText(i);
      Serial.printf("  %3d %s\n", i, errMsg == NULL ? "<NULL>" : errMsg);
    }
  }
  return numErrors;
}

/*
 * Returns true if the given error is found in the list of detected errors.
 */
bool errorDetected(uint16_t errorNumber) {
  unsigned int numErrors = inverter.getNumErrors();
  if (numErrors == 0) {
    return false;
  }
  
  for (int i = 1; i <= numErrors; i++) {
    if (inverter.getErrorNum(i) == errorNumber) {
      return true;
    }
  }
  return false;
}

unsigned long nextLoopMs = 0;
const unsigned long intervalMs = 1000;

bool firstTime = true;
int dotCount = 0;
const int dotsPerLine = 60;
uint32_t lineCount = 0;

void loop() {
  if (firstTime) {
    firstTime = false;
    
    Serial.printf("in loop()\n");
    Serial.printf("%8d: ", lineCount);
    Serial.flush();
  }
  
  unsigned long curMs = millis();
  if (curMs >= nextLoopMs) {
    nextLoopMs = curMs + intervalMs;

    Serial.print(".");
    Serial.flush();
    dotCount += 1;
    if (dotCount >= dotsPerLine) {
      dotCount = 0;
      lineCount += 1;
      Serial.printf("\n%8d: ", lineCount);
    }

    // delay until the next interval to reduce CPU utilization
    /* */
    unsigned long msDelay = nextLoopMs - millis();
    if (msDelay > 1000) {
      msDelay = 1000;
    }
  }
}

/*
 * Handler for TCC0 OVF interrupts.
 */
void TCC0_0_Handler() {
  numTcc0_0_IrqRaw += 1;
  inverter.tccxHandler(0);
  
  if (numWaveLimit > 0 && inverter.getNumWaves() >= numWaveLimit) {
    inverter.stop();
    stopTicks = DWT->CYCCNT;
    inverterRunning = false;
  }
};

/*
 * Handler for TCC1 OVF interrupts.
 */
void TCC1_0_Handler() {
  numTcc1_0_IrqRaw += 1;
  inverter.tccxHandler(1);
  
  if (numWaveLimit > 0 && inverter.getNumWaves() >= numWaveLimit) {
    inverter.stop();
    stopTicks = DWT->CYCCNT;
    inverterRunning = false;
  }
};

/*
 * Handler for ADC0 RESRDY interrupt.
 */
void ADC0_1_Handler() {
  numAdc0_1_IrqRaw += 1;
  inverter.adc0Handler();
}

/*
 * Handler for ADC1 RESRDY interrupt.
 */
void ADC1_1_Handler() {
  numAdc1_1_IrqRaw += 1;
  inverter.adc1Handler();
}

void resetIrqCounts() {
  numTcc0_0_IrqRaw = 0;
  numTcc1_0_IrqRaw = 0;
  numAdc0_1_IrqRaw = 0;
  numAdc1_1_IrqRaw = 0;
}

/*
 * TODO:
 * - PWM freq adjustment
 */
 
/*
 * Unit test framework.
 * To add a new unit test:
 * 1. Write a function with the interface "bool fcn()", that returns true when it passes, and
 *    false when it fails.  See examples below.
 * 2. Add the name of the fuction to the fcnTest array below.
 */
typedef struct {
  bool     (*fcnTest)();
  char     name[64];
  bool     pass;
  int      valuesTested;
} I20UnitTestInfo;

I20UnitTestInfo testInfo[] = {
  {testInvArchValid,                                "testInvArchValid"},                    // 0
  {testHwsValid,                                    "testHwsValid"},                        // 1
  {testNumLinesInvalid,                             "testNumLinesInvalid"},
  {testNumLinesValid,                               "testNumLinesValid"},
  {testOutFreqInvalid,                              "testOutFreqInvalid"},
  {testOutFreqValid,                                "testOutFreqValid"},                    // 5
  {testAdcNumBitsInvalid,                           "testAdcNumBitsInvalid"},
  {testAdcNumBitsValid,                             "testAdcNumBitsValid"},
  {testAdcPrescaleInvalid,                          "testAdcPrescaleInvalid"},
  {testAdcPrescaleValid,                            "testAdcPrescaleValid"},
  {testAdcSampleTicksInvalid,                       "testAdcSampleTicksInvalid"},           // 10
  {testAdcSampleTicksValid,                         "testAdcSampleTicksValid"},
  {test50HzDeadTimeInvalid,                         "test50HzDeadTimeInvalid"},
  {test50HzDeadTimeValid,                           "test50HzDeadTimeValid"},
  {test60HzDeadTimeInvalid,                         "test60HzDeadTimeInvalid"},
  {test60HzDeadTimeValid,                           "test60HzDeadTimeValid"},               // 15
  {testReusedAdcInputPinError,                      "testReusedAdcInputPinError"},
  {testNoFeedback,                                  "testNoFeedback"},
  {testNoFeedback50HzOutFreqTiming,                 "testNoFeedback50HzOutFreqTiming"},
  {testNoFeedback60HzOutFreqTiming,                 "testNoFeedback60HzOutFreqTiming"},
  {testAdcSiblingDoPwmConflict,                     "testAdcSiblingDoPwmConflict"},         // 20
  {testAdcDoPwmMultiplePositions,                   "testAdcDoPwmMultiplePositions"},
  {testAdcDoPwmMultipleSchedules,                   "testAdcDoPwmMultipleSchedules"},
  {testAdcDoPwmNotSet,                              "testAdcDoPwmNotSet"},
  {testAdcSchedTimeGtrPwmCycleTime,                 "testAdcSchedTimeGtrPwmCycleTime"},
};


/*
 * Run all the unit tests in the testInfo array above.
 */
void runUnitTests() {
  int numTests = sizeof(testInfo) / sizeof(I20UnitTestInfo);
  int failCount = 0;
  Serial.printf("IcosaLogic_Inverter_PWM Unit Test: running %d tests\n", numTests);
  
  // clear out test status
  for (int i = 0; i < numTests; i++) {
    testInfo[i].pass = false;
    testInfo[i].valuesTested = 0;
  }
  
  bool testResult = true;
  for (int i = 0; i < numTests; i++) {
    Serial.println(" ");
    Serial.println(" ");
    Serial.println("##########################################################################################");
    resetIrqCounts();
    bool result = (*testInfo[i].fcnTest)();
    testInfo[i].pass = result;
    testResult &= result;
    if (!result) {
      failCount += 1;
    }
  }
  
  Serial.printf("\n\n\n");
  for (int i = 0; i < numTests; i++) {
    Serial.printf("%s  %s\n", testInfo[i].pass ? "PASSED" : "FAILED", testInfo[i].name);
  }
  Serial.printf("Results: %d passed %d failed\n", numTests - failCount, failCount);
}

// Test that valid invArch values are accepted.
bool testInvArchValid() {
  I20InvArch testValue[] = {  I20_HALF_BRIDGE, I20_T_TYPE };
  const int numTestValues = sizeof(testValue) / sizeof(I20InvArch);

  Serial.printf("    START  -- testInvArchValid: testing %d values\n", numTestValues);

  bool testResult = true;
  for (int i = 0; i < numTestValues; i++) {
    memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
    inParams.invArch = testValue[i];
    inParams.feedback = NULL;
    inverter.begin(&inParams);
    testResult &= inverter.getNumErrors() == 0;
    inverter.printErrors();
  }
  Serial.printf("    %6s -- testInvArchValid\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that valid half wave signal (HWS) values are accepted.
bool testHwsValid() {
  I20HalfWaveSignal testValue[] = { I20_HWS_NONE, I20_HWS_SINGLE, I20_HWS_PAIR };
  const int numTestValues = sizeof(testValue) / sizeof(I20HalfWaveSignal);

  Serial.printf("    START  -- testHwsValid: testing %d values\n", numTestValues);

  bool testResult = true;
  for (int i = 0; i < numTestValues; i++) {
    memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
    inParams.hws      = testValue[i];
    inParams.invArch  = I20_HALF_BRIDGE;
    inParams.numLines = 1;
    inParams.feedback = NULL;
    inverter.begin(&inParams);
    testResult &= inverter.getNumErrors() == 0;
    inverter.printErrors();
  }
  Serial.printf("    %6s -- testHwsValid\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that invalid numLines values are detected.
bool testNumLinesInvalid() {
  uint8_t testValue[] = {0, 4};
  const int numTestValues = sizeof(testValue) / sizeof(uint8_t);

  Serial.printf("    START  -- testNumLinesInvalid: testing %d values\n", numTestValues);

  bool testResult = true;
  for (int i = 0; i < numTestValues; i++) {
    memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
    inParams.numLines = testValue[i];
    inverter.begin(&inParams);
    testResult &= errorDetected(I20_ERR_INVALID_NUMLINES_VALUE);
    inverter.printErrors();
  }
  Serial.printf("    %6s -- testNumLinesInvalid\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that valid numLines values are accepted.
bool testNumLinesValid() {
  uint8_t testValue[] = {1, 2, 3};
  const int numTestValues = sizeof(testValue) / sizeof(uint8_t);

  Serial.printf("    START  -- testNumLinesValid: testing %d values\n", numTestValues);

  I20FeedbackSignal vLine1   = {P_X4, I20_PIN_GND,  0, true,  I20_LINE1_VOLTAGE,  820000,  7400, 0.0};
  I20FeedbackSignal vLine2   = {P_X2, I20_PIN_GND,  0, false, I20_LINE2_VOLTAGE,  820000,  7400, 0.0};
  I20Feedback feedback = {{&vLine1, &vLine2, NULL, NULL}};

  bool testResult = true;
  for (int i = 0; i < numTestValues; i++) {
    memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
    inParams.invArch  = I20_HALF_BRIDGE;
    inParams.numLines = testValue[i];
    inParams.feedback = &feedback;
    inverter.begin(&inParams);
    testResult &= inverter.getNumErrors() == 0;
    inverter.printErrors();
  }
  Serial.printf("    %6s -- testNumLinesValid\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that invalid output frequencies are detected.
bool testOutFreqInvalid() {
  uint8_t testValue[] = {49, 51, 59, 61};
  const int numTestValues = sizeof(testValue) / sizeof(uint8_t);

  Serial.printf("    START  -- testOutFreqInvalid: testing %d values\n", numTestValues);

  bool testResult = true;
  for (int i = 0; i < numTestValues; i++) {
    memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
    inParams.outFreq = testValue[i];
    inverter.begin(&inParams);
    testResult &= errorDetected(I20_ERR_INVALID_OUT_FREQ_VALUE);
    inverter.printErrors();
  }
  Serial.printf("    %6s -- testOutFreqInvalid\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that valid output frequencies are accepted.
bool testOutFreqValid() {
  uint8_t testValue[] = {50, 60};
  const int numTestValues = sizeof(testValue) / sizeof(uint8_t);

  Serial.printf("    START  -- testOutFreqValid: testing %d values\n", numTestValues);

  bool testResult = true;
  for (int i = 0; i < numTestValues; i++) {
    memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
    inParams.outFreq = testValue[i];
    inverter.begin(&inParams);
    testResult &= inverter.getNumErrors() == 0;
    inverter.printErrors();
  }
  Serial.printf("    %6s -- testOutFreqValid\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that invalid adcNumBits are detected.
bool testAdcNumBitsInvalid() {
  uint8_t testValue[] = {7, 9, 11, 13};
  const int numTestValues = sizeof(testValue) / sizeof(uint8_t);

  Serial.printf("    START  -- testAdcNumBitsInvalid: testing %d values\n", numTestValues);

  I20FeedbackSignal vLine1   = {P_X4, I20_PIN_GND,  0, true, I20_LINE1_VOLTAGE,  820000,  7400, 0.0};
  I20Feedback feedback = {{&vLine1, NULL, NULL, NULL, NULL, NULL}};

  bool testResult = true;
  for (int i = 0; i < numTestValues; i++) {
    memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
    inParams.feedback = &feedback;
    inParams.adcNumBits = testValue[i];
    inverter.begin(&inParams);
    testResult &= errorDetected(I20_ERR_INVALID_NUMADCBITS_VALUE);
    inverter.printErrors();
  }
  Serial.printf("    %6s -- testAdcNumBitsInvalid\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that valid adcNumBits are accepted.
bool testAdcNumBitsValid() {
  uint8_t testValue[] = {8, 10, 12};
  const int numTestValues = sizeof(testValue) / sizeof(uint8_t);

  Serial.printf("    START  -- testAdcNumBitsValid: testing %d values\n", numTestValues);

  I20FeedbackSignal vLine1   = {P_X4, I20_PIN_GND,  0, true, I20_LINE1_VOLTAGE,  820000,  7400, 0.0};
  I20Feedback feedback = {{&vLine1, NULL, NULL, NULL, NULL, NULL}};

  bool testResult = true;
  for (int i = 0; i < numTestValues; i++) {
    memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
    inParams.feedback = &feedback;
    inParams.adcNumBits = testValue[i];
    inverter.begin(&inParams);
    testResult &= inverter.getNumErrors() == 0;
    inverter.printErrors();
  }
  Serial.printf("    %6s -- testAdcNumBitsValid\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that invalid ADC prescale values are detected.
// valid values are powers of 2, where 2 <= x <= 256
bool testAdcPrescaleInvalid() {
  uint16_t testValue[] = {0, 1, 3, 5, 7, 9, 15, 17, 31, 33, 63, 65, 127, 129, 255, 257};
  const int numTestValues = sizeof(testValue) / sizeof(uint16_t);

  Serial.printf("    START  -- testAdcPrescaleInvalid: testing %d values\n", numTestValues);

  I20FeedbackSignal vLine1   = {P_X4, I20_PIN_GND,  0, true, I20_LINE1_VOLTAGE,  820000,  7400, 0.0};
  I20Feedback feedback = {{&vLine1, NULL, NULL, NULL, NULL, NULL}};

  bool testResult = true;
  for (int i = 0; i < numTestValues; i++) {
    memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
    inParams.feedback = &feedback;
    inParams.adcPrescaleVal = testValue[i];
    inverter.begin(&inParams);
    testResult &= errorDetected(I20_ERR_INVALID_ADC_PRESCALE_VALUE);
    inverter.printErrors();
  }
  Serial.printf("    %6s -- testAdcPrescaleInvalid\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that valid ADC prescale values are accepted.
bool testAdcPrescaleValid() {
  uint16_t testValue[] = {2, 4, 8, 16, 32, 64, 128, 256};
  const int numTestValues = sizeof(testValue) / sizeof(uint16_t);

  Serial.printf("    START  -- testAdcPrescaleValid: testing %d values\n", numTestValues);

  I20FeedbackSignal vLine1   = {P_X4, I20_PIN_GND,  0, true, I20_LINE1_VOLTAGE,  820000,  7400, 0.0};
  I20Feedback feedback = {{&vLine1, NULL, NULL, NULL, NULL, NULL}};

  bool testResult = true;
  for (int i = 0; i < numTestValues; i++) {
    memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
    inParams.feedback = &feedback;
    inParams.pwmFreq = 240;
    inParams.adcPrescaleVal = testValue[i];
    inverter.begin(&inParams);
    testResult &= inverter.getNumErrors() == 0;
    inverter.printErrors();
  }
  Serial.printf("    %6s -- testAdcPrescaleValid\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that invalid ADC sample tick values are detected.
bool testAdcSampleTicksInvalid() {
  uint8_t testValue[] = {0, 65};
  const int numTestValues = sizeof(testValue) / sizeof(uint8_t);

  Serial.printf("    START  -- testAdcSampleTicksInvalid: testing %d values\n", numTestValues);

  bool testResult = true;
  for (int i = 0; i < numTestValues; i++) {
    memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
    inParams.adcSampleTicks = testValue[i];
    inverter.begin(&inParams);
    testResult &= errorDetected(I20_ERR_INVALID_ADC_SAMPLE_TICKS);
    inverter.printErrors();
  }

  Serial.printf("    %6s -- testAdcSampleTicksInvalid\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that valid ADC sample tick values are accepted.
bool testAdcSampleTicksValid() {
  uint8_t testValue[] = {1, 16, 32, 64};
  const int numTestValues = sizeof(testValue) / sizeof(uint8_t);

  Serial.printf("    START  -- testAdcSampleTicksValid: testing %d values\n", numTestValues);

  bool testResult = true;
  for (int i = 0; i < numTestValues; i++) {
    memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
    inParams.adcSampleTicks = testValue[i];
    inverter.begin(&inParams);
    testResult &= inverter.getNumErrors() == 0;
    inverter.printErrors();
  }
  Serial.printf("    %6s -- testAdcSampleTicksValid\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that invalid 50 Hz deadTimeNs values are detected.
// At 50 Hz, dead time per tick is 10 ns
bool test50HzDeadTimeInvalid() {
  uint16_t testValue[] = {2551, 2552, 2560};
  const int numTestValues = sizeof(testValue) / sizeof(uint16_t);

  Serial.printf("    START  -- test50HzDeadTimeInvalid: testing %d values\n", numTestValues);

  bool testResult = true;
  for (int i = 0; i < numTestValues; i++) {
    memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
    inParams.outFreq    = 50;
    inParams.deadTimeNs = testValue[i];
    inverter.begin(&inParams);
    testResult &= errorDetected(I20_ERR_DEADTIME_MAX_EXCEEDED);
    inverter.printErrors();
  }
  Serial.printf("    %6s -- test50HzDeadTimeInvalid\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that valid 50 Hz deadTimeNs values are accepted.
// At 50 Hz, dead time per tick is 10 ns
bool test50HzDeadTimeValid() {
  uint16_t testValue[] = {0, 1, 9, 10, 11, 100, 1000, 2550};
  const int numTestValues = sizeof(testValue) / sizeof(uint16_t);

  Serial.printf("    START  -- test50HzDeadTimeValid: testing %d values\n", numTestValues);

  bool testResult = true;
  for (int i = 0; i < numTestValues; i++) {
    memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
    inParams.outFreq    = 50;
    inParams.deadTimeNs = testValue[i];
    inverter.begin(&inParams);
    testResult &= inverter.getNumErrors() == 0;
    inverter.printErrors();
  }
  Serial.printf("    %6s -- test50HzDeadTimeValid\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that invalid 60 Hz deadTimeNs values are detected.
// At 60 Hz, dead time per tick is 8.333 ns
bool test60HzDeadTimeInvalid() {
  uint16_t testValue[] = {2126, 2127};
  const int numTestValues = sizeof(testValue) / sizeof(uint16_t);

  Serial.printf("    START  -- test60HzDeadTimeInvalid: testing %d values\n", numTestValues);

  bool testResult = true;
  for (int i = 0; i < numTestValues; i++) {
    memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
    inParams.outFreq    = 60;
    inParams.deadTimeNs = testValue[i];
    inverter.begin(&inParams);
    testResult &= errorDetected(I20_ERR_DEADTIME_MAX_EXCEEDED);
    inverter.printErrors();
  }
  Serial.printf("    %6s -- test60HzDeadTimeInvalid\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that valid 60 Hz deadTimeNs values are accepted.
// At 60 Hz, dead time per tick is 8.333 ns
bool test60HzDeadTimeValid() {
  uint16_t testValue[] = {0, 1, 7, 8, 9, 10, 100, 1000, 2125};
  const int numTestValues = sizeof(testValue) / sizeof(uint16_t);

  Serial.printf("    START  -- test60HzDeadTimeValid: testing %d values\n", numTestValues);

  bool testResult = true;
  for (int i = 0; i < numTestValues; i++) {
    memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
    inParams.outFreq    = 60;
    inParams.deadTimeNs = testValue[i];
    inverter.begin(&inParams);
    testResult &= inverter.getNumErrors() == 0;
    inverter.printErrors();
  }
  Serial.printf("    %6s -- test60HzDeadTimeValid\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that reusing an ADC input pin in the feedback configuration is an error.
bool testReusedAdcInputPinError() {
  Serial.printf("    START  -- testReusedAdcInputPinError:\n");

  I20FeedbackSignal vLine1   = {P_X4, I20_PIN_GND,  0, false, I20_LINE1_VOLTAGE, 820000,  7400, 0.0};
  I20FeedbackSignal vLine2   = {P_X4, I20_PIN_GND,  0, false, I20_LINE2_VOLTAGE, 820000,  7400, 0.0};
  I20Feedback feedback = {{&vLine1, &vLine2, NULL, NULL, NULL, NULL }}; 

  bool testResult = true;
  memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
  inParams.feedback = &feedback;
  inverter.begin(&inParams);
  testResult &= errorDetected(I20_ERR_REUSED_ADC_INPUT_PIN);
  inverter.printErrors();

  Serial.printf("    %6s -- testReusedAdcInputPinError\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that no feedback initializes correctly.
bool testNoFeedback() {
  Serial.printf("    START  -- testNoFeedback:\n");

  bool testResult = true;
  memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
  inParams.feedback = NULL;
  inverter.begin(&inParams);
  testResult &= inverter.getNumErrors() == 0;
  Serial.printf("    %6s -- testNoFeedback\n", testResult ? "PASSED" : "FAILED");
  inverter.printErrors();
  
  return testResult;
}

// Test that no feedback 50 Hz output frequency timing is acceptable.
// TODO: extract logic out to inverterStartStop() function ==============================================
bool testNoFeedback50HzOutFreqTiming() {
  Serial.printf("    START  -- testNoFeedback50HzOutFreqTiming\n");

  bool testResult = false;
  
  memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
  inParams.feedback = NULL;
  inParams.numLines = 1;
  inParams.outFreq  = 50;
  inParams.pwmFreq  = 200;
  inverter.begin(&inParams);
  inverter.printErrors();
  
  if (inverter.getNumErrors() == 0) {
    ilsd.dumpTCC("after begin");
    
    numWaveLimit = 5;
    uint32_t maxExtraDelays = 500;
    uint32_t delayMs = 1000 / inParams.outFreq;
    inverter.start();
    startTicks = DWT->CYCCNT;
    delay(delayMs);
    for (int i = 0; i < maxExtraDelays && inverter.isRunning(); i++) {
      delay(2);
    }
    if (inverter.isRunning()) {
      Serial.printf("           -- inverter would not stop\n");
    } else {
      uint32_t elapsedTicks = stopTicks - startTicks;
      if (stopTicks < startTicks) {
        // the clock wrapped during the test
        elapsedTicks = (0xffffffff - startTicks) + stopTicks;
      }
      const uint32_t ticksPerMs = 120 * 1000;
      uint32_t elapsedMs = elapsedTicks / ticksPerMs;
      
      uint32_t expectedMs = 1000 * numWaveLimit / inParams.outFreq;
      uint32_t deltaPercent = 3;
      uint32_t deltaMs = expectedMs * deltaPercent / 100;
      uint32_t expectedMsMin = expectedMs - deltaMs;
      uint32_t expectedMsMax = expectedMs + deltaMs;
      
      testResult = expectedMsMin <= expectedMs && expectedMs <= expectedMsMax;
      
      Serial.printf("           -- elapsed ms min %d actual %d max %d\n",
                    expectedMsMin, elapsedMs, expectedMsMax);
    }
  }
  
  Serial.printf("    %6s -- testNoFeedback50HzOutFreqTiming\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Test that no feedback 60 Hz output frequency timing is acceptable.
bool testNoFeedback60HzOutFreqTiming() {
  Serial.printf("    START  -- testNoFeedback60HzOutFreqTiming\n");

  bool testResult = false;
  
  memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
  inParams.feedback = NULL;
  inParams.numLines = 1;
  inParams.outFreq  = 60;
  inParams.pwmFreq  = 240;
  inverter.begin(&inParams);
  inverter.printErrors();
  
  if (inverter.getNumErrors() == 0) {
    ilsd.dumpTCC("after begin");
    
    numWaveLimit = 5;
    uint32_t maxExtraDelays = 500;
    uint32_t delayMs = 1000 / inParams.outFreq;
    inverter.start();
    startTicks = DWT->CYCCNT;
    delay(delayMs);
    for (int i = 0; i < maxExtraDelays && inverter.isRunning(); i++) {
      delay(2);
    }
    if (inverter.isRunning()) {
      Serial.printf("           -- inverter would not stop\n");
    } else {
      uint32_t elapsedTicks = stopTicks - startTicks;
      if (stopTicks < startTicks) {
        // the clock wrapped during the test
        elapsedTicks = (0xffffffff - startTicks) + stopTicks;
      }
      const uint32_t ticksPerMs = 120 * 1000;
      uint32_t elapsedMs = elapsedTicks / ticksPerMs;
      
      uint32_t expectedMs = 1000 * numWaveLimit / inParams.outFreq;
      uint32_t deltaPercent = 3;
      uint32_t deltaMs = expectedMs * deltaPercent / 100;
      uint32_t expectedMsMin = expectedMs - deltaMs;
      uint32_t expectedMsMax = expectedMs + deltaMs;
      
      testResult = expectedMsMin <= expectedMs && expectedMs <= expectedMsMax;
      
      Serial.printf("           -- elapsed ms min %d actual %d max %d\n",
                    expectedMsMin, elapsedMs, expectedMsMax);
    }
  }
  
  Serial.printf("    %6s -- testNoFeedback60HzOutFreqTiming\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Verify that an error is generated when siblings do not have the doPwmUpdate value
//    I20_ERR_ADC_SIBLING_DO_PWM_CONFLICT
bool testAdcSiblingDoPwmConflict() {
  Serial.printf("    START  -- testAdcSiblingDoPwmConflict:\n");

  I20FeedbackSignal vLine1   = {P_X4, I20_PIN_GND,  0, true,  I20_LINE1_VOLTAGE, 820000,  7400, 0.0};
  I20FeedbackSignal vLine2   = {P_X1, I20_PIN_GND,  0, false, I20_LINE2_VOLTAGE, 820000,  7400, 0.0};
  I20Feedback feedback = {{&vLine1, &vLine2, NULL, NULL, NULL, NULL}};

  bool testResult = true;
  memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
  inParams.feedback = &feedback;
  inverter.begin(&inParams);
  testResult &= errorDetected(I20_ERR_ADC_SIBLING_DO_PWM_CONFLICT);
  inverter.printErrors();
  
  Serial.printf("    %6s -- testAdcSiblingDoPwmConflict\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Verify that an error is generated when doPwmUpdate is set in multiple positions in a schedule
//    I20_ERR_ADC_DO_PWM_MULTIPLE_POS,
bool testAdcDoPwmMultiplePositions() {
  Serial.printf("    START  -- testAdcDoPwmMultiplePositions:\n");

  I20FeedbackSignal vLine1   = {P_X4, I20_PIN_GND,  0, true,  I20_LINE1_VOLTAGE, 820000,  7400, 0.0};
  I20FeedbackSignal vLine2   = {P_X1, I20_PIN_GND,  1, true,  I20_LINE2_VOLTAGE, 820000,  7400, 0.0};
  I20Feedback feedback = {{&vLine1, &vLine2, NULL, NULL, NULL, NULL}};

  bool testResult = true;
  memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
  inParams.feedback = &feedback;
  inverter.begin(&inParams);
  testResult &= errorDetected(I20_ERR_ADC_DO_PWM_MULTIPLE_POS);
  inverter.printErrors();
  
  Serial.printf("    %6s -- testAdcDoPwmMultiplePositions\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Verify that an error is generated when doPwmUpdate is set in multiple schedules
//    I20_ERR_ADC_DO_PWM_ON_MULTIPLE_SCHEDULES,
bool testAdcDoPwmMultipleSchedules() {
  Serial.printf("    START  -- testAdcDoPwmMultipleSchedules:\n");

  I20FeedbackSignal vLine1   = {P_X4, I20_PIN_GND,  0, true,  I20_LINE1_VOLTAGE, 820000,  7400, 0.0};
  I20FeedbackSignal vLine2   = {P_X2, I20_PIN_GND,  0, true,  I20_LINE2_VOLTAGE, 820000,  7400, 0.0};
  I20Feedback feedback = {{&vLine1, &vLine2, NULL, NULL, NULL, NULL}};

  bool testResult = true;
  memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
  inParams.feedback = &feedback;
  inverter.begin(&inParams);
  testResult &= errorDetected(I20_ERR_ADC_DO_PWM_ON_MULTIPLE_SCHEDULES);
  inverter.printErrors();

  Serial.printf("    %6s -- testAdcDoPwmMultipleSchedules\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Detect when doPwmUpdate not set in any ADC schedule
//    I20_ERR_ADC_DO_PWM_NOT_SET
bool testAdcDoPwmNotSet() {
  Serial.printf("    START  -- testAdcDoPwmNotSet:\n");

  I20FeedbackSignal vLine1   = {P_X4, I20_PIN_GND,  0, false, I20_LINE1_VOLTAGE,  820000,  7400, 0.0};
  I20FeedbackSignal vLine2   = {P_X2, I20_PIN_GND,  0, false, I20_LINE2_VOLTAGE,  820000,  7400, 0.0};
  I20Feedback feedback = {{&vLine1, &vLine2, NULL, NULL, NULL, NULL}};

  bool testResult = true;
  memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
  inParams.feedback = &feedback;
  inverter.begin(&inParams);
  testResult &= errorDetected(I20_ERR_ADC_DO_PWM_NOT_SET);
  inverter.printErrors();

  Serial.printf("    %6s -- testAdcDoPwmNotSet\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

// Verify that we detect when an ADC schedule run time is greater than the PWM cycle time
bool testAdcSchedTimeGtrPwmCycleTime() {
  Serial.printf("    START  -- testAdcSchedTimeGtrPwmCycleTime:\n");

  bool testResult = true;
  memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
  inParams.adcPrescaleVal = 256;
  inParams.outFreq = 50;
  inParams.pwmFreq = 100000;
  inverter.begin(&inParams);
  testResult &= errorDetected(I20_ERR_ADC_SCHEDULE_TIME_LONGER_THAN_PWM_TIME);
  inverter.printErrors();
  
  Serial.printf("    %6s -- testAdcSchedTimeGtrPwmCycleTime\n", testResult ? "PASSED" : "FAILED");
  return testResult;
}

