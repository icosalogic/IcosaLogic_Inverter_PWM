/*
 * Test inverter for the IcosaLogic inverter library.
 * This test is written to run on an Adafruit ItsyBitsy M4.
 * The TCC config can use slow frequencies for debugging purposes.
 * The ADC config can be changed as desired.
 */

int firstVariable = 4321;

#include <Arduino.h>
#include <IcosaLogic_Inverter_PWM.h>
#include <SAMD51_Dumpster.h>

// When > 0, this limits how many sine waves the inverter will generate before stopping.
// when == 0, the inverter runs indefinitely.
// This makes possible to run the inverter for a short burst that is easy to analyze.
uint32_t numWaveLimit = 0;

// this should be true only for low pwmFreq, and low numWaveLimit
bool displayLog = false;            

// Basic inverter configuration values
const I20InvArch        invArch        = I20_T_TYPE;
const I20HalfWaveSignal hws            = I20_HWS_NONE;
const uint8_t           numLines       = 1;
const uint16_t          outRmsVoltage  = 120;
const uint8_t           outputFreq     = 60;
const uint32_t          pwmFreq        = 60000;
const uint16_t          deadTimeNs     = 100;
const uint16_t          adcPrescale    = 16;
const uint16_t          adcSampleTicks = 6;

// Feedback signal configurations
// ADC0 and ADC1 in parallel for 1 reading each from A4, A2
I20FeedbackSignal fbs0101   = {I20_PIN_A4_ADC0, I20_PIN_GND,  0, false, I20_LINE1_VOLTAGE, 820000,  7400, 0.0};
I20FeedbackSignal fbs0102   = {I20_PIN_A2_ADC1, I20_PIN_GND,  0, true,  I20_LINE1_CURRENT,      0,     0, 0.025177};
I20Feedback fb_2adc_1e      = {{&fbs0101, &fbs0102, NULL}};

// ADC0  1 reading A2
I20FeedbackSignal fbs0201   = {I20_PIN_A2_ADC0, I20_PIN_GND,  1, true,  I20_LINE1_VOLTAGE, 820000,  7400, 0.0};
I20Feedback fb_1adc0_1e_2   = {{&fbs0201, NULL}};

// ADC1  1 reading A2
I20FeedbackSignal fbs0301   = {I20_PIN_A2_ADC1, I20_PIN_GND,  1, true,  I20_LINE1_VOLTAGE, 820000,  7400, 0.0};
I20Feedback fb_1adc1_1e_2   = {{&fbs0301, NULL}};

// ADC0  1 reading A3
I20FeedbackSignal fbs0401   = {I20_PIN_A3_ADC0, I20_PIN_GND,  1, true,  I20_LINE1_VOLTAGE, 820000,  7400, 0.0};
I20Feedback fb_1adc0_1e_3   = {{&fbs0401, NULL}};

// ADC1  1 reading A3
I20FeedbackSignal fbs0501   = {I20_PIN_A3_ADC1, I20_PIN_GND,  1, true,  I20_LINE1_VOLTAGE, 820000,  7400, 0.0};
I20Feedback fb_1adc1_1e_3   = {{&fbs0501, NULL}};

// max config for this platform
I20FeedbackSignal fbs0601   = {I20_PIN_A4_ADC0, I20_PIN_GND,  0, false, I20_LINE1_VOLTAGE,   820000,  7400, 0.0};
I20FeedbackSignal fbs0602   = {I20_PIN_A2_ADC1, I20_PIN_GND,  0, false, I20_LINE2_VOLTAGE,   820000,  7400, 0.0};
I20FeedbackSignal fbs0603   = {I20_PIN_A5_ADC0, I20_PIN_GND,  1, false, I20_LINE1_CURRENT,        0,     0, 0.025177};
I20FeedbackSignal fbs0604   = {I20_PIN_A3_ADC1, I20_PIN_GND,  1, true,  I20_LINE2_CURRENT,        0,     0, 0.025177};
I20FeedbackSignal fbs0605   = {I20_PIN_A0_ADC0, I20_PIN_GND,  2, false, I20_BATTTOP_VOLTAGE, 820000,  4460, 0.0};
I20FeedbackSignal fbs0606   = {I20_PIN_A1_ADC0, I20_PIN_GND,  2, false, I20_BATTMID_VOLTAGE, 820000,  4460, 0.0};
I20Feedback fb_max          = {{&fbs0601, &fbs0602, &fbs0603, &fbs0604, &fbs0605, &fbs0606 }};

I20InputParams defaultParams = {invArch,         // inverter architecture
                                hws,             // half wave signal to generate
                                outRmsVoltage,   // RMS voltage of output lines
                                numLines,        // number of output lines
                                outputFreq,      // output frequency, either 50 or 60 Hz
                                pwmFreq,         // PWM frequency
                                I20_PS_TCC1,     // Primary TCC configuration to use
                                I20_PS_TCC0,     // Secondary TCC configuration to use
                                deadTimeNs,      // dead time between MOSFET transitions
                                adcPrescale,     // ADC clock prescale value
                                adcSampleTicks,  // ADC clock ticks to hold sample
                                AR_DEFAULT,      // ADC reference
                                NULL,
                              };
I20InputParams inParams;

IcosaLogic_Inverter_PWM inverter;                  // inverter object

SAMD51_Dumpster ilsd;

// IRQ counters
volatile uint32_t numTcc0_0_IrqRaw = 0;
volatile uint32_t numTcc1_0_IrqRaw = 0;
volatile uint32_t numAdc0_1_IrqRaw = 0;
volatile uint32_t numAdc1_1_IrqRaw = 0;

volatile bool inverterRunning = false;
volatile uint32_t startTicks = 0;
volatile uint32_t stopTicks = 0;


/*
 * Initialize the inverter.
 */
void setup() {
  setupSerial();
  ilsd.begin(true);

  // setupAndRunInverter();
  memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
  inverter.begin(&inParams);
  
  // ilsd.dumpGCLK("after begin()");
  // ilsd.dumpPort("after begin()");
  // ilsd.dumpADC("after begin()");
  
  Serial.println("done with setup");
}

/**
 * Set up the serial interface.  Delay units are in milliseconds.
 */
void setupSerial() {
  Serial.begin(115200);
  
  ilsd.dumpADC("After begin()");
  
  const int delayIncr = 100;
  const int maxDelay = 2000;
  for (int i = 0; i < maxDelay; i += delayIncr) {
    delay(delayIncr);
    Serial.print(".");
    Serial.flush();
  }
  
  Serial.println(" ");
  Serial.println("\n\nIcosaLogic_Inverter_PWM Test Inverter App\n\n");
}

/*
 * Set up and run the inverter for a few sine wave cycles.
 */
void setupAndRunInverter() {
  memcpy(&inParams, &defaultParams, sizeof(I20InputParams));
  inverter.begin(&inParams);
  
  if (checkErrors() > 0) {
    Serial.printf("not started due to errors\n");
  } else {
    uint32_t maxExtraDelays = 100;
    uint32_t expectedMs = 1000 * numWaveLimit / inParams.outFreq;
    
    inverter.start();
    startTicks = DWT->CYCCNT;
    
    // wait for inverter to stop
    delay(expectedMs);
    for (int i = 0; i < maxExtraDelays && inverter.isRunning(); i++) {
      delay(2);
    }
    
    if (inverter.isRunning()) {
      Serial.printf("           -- inverter would not stop\n");
    } else {
      bool result = checkIntervalInRange(expectedMs, 3);
      
      Serial.printf("    IRQ counts: TCC0 %d TCC1 %d ADC0 %d  ADC1 %d\n",
                    numTcc0_0_IrqRaw, numTcc1_0_IrqRaw, numAdc0_1_IrqRaw, numAdc1_1_IrqRaw);
      if (numTcc0_0_IrqRaw + numTcc1_0_IrqRaw == 0) {
        Serial.printf("TCC IRQ count should be > 0\n");
      }
    }
    inverter.dumpLog(false);
  }
}

/*
 * Returns true if the elapsed run time is within the allowable delta percentage.
 */
bool checkIntervalInRange(uint32_t expectedMs, uint8_t deltaPercentAllowed) {
  uint32_t elapsedTicks = stopTicks - startTicks;
  if (stopTicks < startTicks) {
    // the clock wrapped during the test
    elapsedTicks = (0xffffffff - startTicks) + stopTicks;
  }
  const uint32_t ticksPerMs = 120 * 1000;             // assumes 120 MHz clock
  uint32_t elapsedMs = elapsedTicks / ticksPerMs;
  
  uint32_t deltaMs = expectedMs * deltaPercentAllowed / 100;
  uint32_t expectedMsMin = expectedMs - deltaMs;
  uint32_t expectedMsMax = expectedMs + deltaMs;
  
  Serial.printf("           -- elapsed ms min %d actual %d max %d\n",
                expectedMsMin, elapsedMs, expectedMsMax);
                    
  bool result = expectedMsMin <= expectedMs && expectedMs <= expectedMsMax;
  
  return result;
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
    
    if (checkErrors() > 0) {
      Serial.printf("inverter not started due to errors\n");
    } else {
      inverter.start();
    }
  }
  
  if (displayLog) {
    inverter.dumpLogTail(true);
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
