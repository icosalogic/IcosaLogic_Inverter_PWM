/*
 * Test inverter for the IcosaLogic inverter library.
 * This test is written to run on an Adafruit Feather M4.
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
const uint8_t           numLines       = 2;
const uint16_t          outRmsVoltage  = 120;
const uint16_t          outCurrent     = 50;
const uint8_t           outputFreq     = 60;
const uint32_t          pwmFreq        = 48000;
const uint16_t          deadTimeNs     = 100;
const uint8_t           adcNumBits     = 12;
const uint16_t          adcPrescale    = 16;
const uint16_t          adcSampleTicks = 6;
const eAnalogReference  adcVRefNdx     = AR_DEFAULT;

/*
 * Feedback signal configurations
 * 
 * This section shows multiple feedback configurations used for testing ADC readings.
 * Users can model their feedback configuration after one of the examples below.
 */
 
// ADC0 and ADC1 in parallel for 1 reading each from A4, A2
I20FeedbackSignal fbs0101   = {I20_PIN_A4_ADC0, I20_PIN_GND,  0, false, I20_LINE1_VOLTAGE, 820000,  7400, 0.0};
I20FeedbackSignal fbs0102   = {I20_PIN_A2_ADC1, I20_PIN_GND,  0, true,  I20_LINE1_CURRENT,      0,     0, 0.025177};
I20Feedback fb_2adc_1e      = {adcNumBits, adcPrescale, adcSampleTicks, adcVRefNdx, defaultAdcVRef,
                               {&fbs0101, &fbs0102, NULL}};

// ADC0  1 reading A2
I20FeedbackSignal fbs0201   = {I20_PIN_A2_ADC0, I20_PIN_GND,  1, true,  I20_LINE1_VOLTAGE, 820000,  7400, 0.0};
I20Feedback fb_1adc0_1e_2   = {adcNumBits, adcPrescale, adcSampleTicks, adcVRefNdx, defaultAdcVRef,
                               {&fbs0201, NULL}};

// ADC1  1 reading A2
I20FeedbackSignal fbs0301   = {I20_PIN_A2_ADC1, I20_PIN_GND,  1, true,  I20_LINE1_VOLTAGE, 820000,  7400, 0.0};
I20Feedback fb_1adc1_1e_2   = {adcNumBits, adcPrescale, adcSampleTicks, adcVRefNdx, defaultAdcVRef,
                               {&fbs0301, NULL}};

// ADC0  1 reading A3
I20FeedbackSignal fbs0401   = {I20_PIN_A3_ADC0, I20_PIN_GND,  1, true,  I20_LINE1_VOLTAGE, 820000,  7400, 0.0};
I20Feedback fb_1adc0_1e_3   = {adcNumBits, adcPrescale, adcSampleTicks, adcVRefNdx, defaultAdcVRef,
                               {&fbs0401, NULL}};

// ADC1  1 reading A3
I20FeedbackSignal fbs0501   = {I20_PIN_A3_ADC1, I20_PIN_GND,  1, true,  I20_LINE1_VOLTAGE, 820000,  7400, 0.0};
I20Feedback fb_1adc1_1e_3   = {adcNumBits, adcPrescale, adcSampleTicks, adcVRefNdx, defaultAdcVRef,
                               {&fbs0501, NULL}};

// max config for this platform
I20FeedbackSignal fbs0601   = {I20_PIN_A4_ADC0, I20_PIN_GND,  0, false, I20_LINE1_VOLTAGE,   820000,  7400, 0.0};
I20FeedbackSignal fbs0602   = {I20_PIN_A2_ADC1, I20_PIN_GND,  0, false, I20_LINE2_VOLTAGE,   820000,  7400, 0.0};
I20FeedbackSignal fbs0603   = {I20_PIN_A5_ADC0, I20_PIN_GND,  1, false, I20_LINE1_CURRENT,        0,     0, 0.025177};
I20FeedbackSignal fbs0604   = {I20_PIN_A3_ADC1, I20_PIN_GND,  1, true,  I20_LINE2_CURRENT,        0,     0, 0.025177};
I20FeedbackSignal fbs0605   = {I20_PIN_A0_ADC0, I20_PIN_GND,  2, false, I20_BATTTOP_VOLTAGE, 820000,  4460, 0.0};
I20FeedbackSignal fbs0606   = {I20_PIN_A1_ADC0, I20_PIN_GND,  2, false, I20_BATTMID_VOLTAGE, 820000,  4460, 0.0};
I20Feedback fb_max          = {adcNumBits, adcPrescale, adcSampleTicks, adcVRefNdx, defaultAdcVRef,
                               {&fbs0601, &fbs0602, &fbs0603, &fbs0604, &fbs0605, &fbs0606 }};

// ADC0  2 sequential readings A2, A3
I20FeedbackSignal fbs0701   = {I20_PIN_A2_ADC0, I20_PIN_GND,  0, false, I20_LINE1_VOLTAGE, 820000,  7400, 0.0};
I20FeedbackSignal fbs0702   = {I20_PIN_A3_ADC0, I20_PIN_GND,  1, true,  I20_LINE2_VOLTAGE, 820000,  7400, 0.0};
I20Feedback fb_1adc0_2e     = {adcNumBits, adcPrescale, adcSampleTicks, adcVRefNdx, defaultAdcVRef,
                               {&fbs0701, &fbs0702, NULL}};

// ADC1  2 sequential readings A2, A3
I20FeedbackSignal fbs0801   = {I20_PIN_A2_ADC1, I20_PIN_GND,  0, false, I20_LINE1_VOLTAGE, 820000,  7400, 0.0};
I20FeedbackSignal fbs0802   = {I20_PIN_A3_ADC1, I20_PIN_GND,  1, true,  I20_LINE2_VOLTAGE, 820000,  7400, 0.0};
I20Feedback fb_1adc1_2e     = {adcNumBits, adcPrescale, adcSampleTicks, adcVRefNdx, defaultAdcVRef,
                               {&fbs0801, &fbs0802, NULL}};

// ADC0 and ADC1 in parallel 2 sequential readings each
I20FeedbackSignal fbs0901   = {I20_PIN_A4_ADC0, I20_PIN_GND,  0, false, I20_LINE1_VOLTAGE,   820000,  7400, 0.0};
I20FeedbackSignal fbs0902   = {I20_PIN_A2_ADC1, I20_PIN_GND,  0, false, I20_LINE2_VOLTAGE,   820000,  7400, 0.0};
I20FeedbackSignal fbs0903   = {I20_PIN_A5_ADC0, I20_PIN_GND,  1, false, I20_LINE1_CURRENT,        0,     0, 0.025177};
I20FeedbackSignal fbs0904   = {I20_PIN_A3_ADC1, I20_PIN_GND,  1, true,  I20_LINE2_CURRENT,        0,     0, 0.025177};
I20Feedback fb_2adc_2e      = {adcNumBits, adcPrescale, adcSampleTicks, adcVRefNdx, defaultAdcVRef,
                               {&fbs0901, &fbs0902, &fbs0903, &fbs0904, NULL }};

// ADC0 4 sibling readings position 0
I20FeedbackSignal fbs1001   = {I20_PIN_A4_ADC0, I20_PIN_GND,  0, true,  I20_LINE1_VOLTAGE,   820000,  7400, 0.0};
I20FeedbackSignal fbs1002   = {I20_PIN_A2_ADC0, I20_PIN_GND,  0, true,  I20_LINE2_VOLTAGE,   820000,  7400, 0.0};
I20FeedbackSignal fbs1003   = {I20_PIN_A5_ADC0, I20_PIN_GND,  0, true,  I20_LINE1_CURRENT,        0,     0, 0.025177};
I20FeedbackSignal fbs1004   = {I20_PIN_A3_ADC0, I20_PIN_GND,  0, true,  I20_LINE2_CURRENT,        0,     0, 0.025177};
I20Feedback fb_1adc0_4s0    = {adcNumBits, adcPrescale, adcSampleTicks, adcVRefNdx, defaultAdcVRef,
                               {&fbs1001, &fbs1002, &fbs1003, &fbs1004, NULL }};

// ADC0 4 sibling readings position 1
I20FeedbackSignal fbs1101   = {I20_PIN_A0_ADC0, I20_PIN_GND,  0, false, I20_BATTTOP_VOLTAGE, 820000,  4460, 0.0};
I20FeedbackSignal fbs1102   = {I20_PIN_A4_ADC0, I20_PIN_GND,  1, true,  I20_LINE1_VOLTAGE,   820000,  7400, 0.0};
I20FeedbackSignal fbs1103   = {I20_PIN_A2_ADC0, I20_PIN_GND,  1, true,  I20_LINE2_VOLTAGE,   820000,  7400, 0.0};
I20FeedbackSignal fbs1104   = {I20_PIN_A5_ADC0, I20_PIN_GND,  1, true,  I20_LINE1_CURRENT,        0,     0, 0.025177};
I20FeedbackSignal fbs1105   = {I20_PIN_A3_ADC0, I20_PIN_GND,  1, true,  I20_LINE2_CURRENT,        0,     0, 0.025177};
I20Feedback fb_1adc0_4s1    = {adcNumBits, adcPrescale, adcSampleTicks, adcVRefNdx, defaultAdcVRef,
                               {&fbs1101, &fbs1102, &fbs1103, &fbs1104, NULL }};

// ADC0 2 siblings in positions 0, 1, and 2
I20FeedbackSignal fbs1201   = {I20_PIN_A4_ADC0, I20_PIN_GND,  0, false, I20_LINE1_VOLTAGE,   820000,  7400, 0.0};
I20FeedbackSignal fbs1202   = {I20_PIN_A2_ADC0, I20_PIN_GND,  0, false, I20_LINE2_VOLTAGE,   820000,  7400, 0.0};
I20FeedbackSignal fbs1203   = {I20_PIN_A5_ADC0, I20_PIN_GND,  1, true,  I20_LINE1_CURRENT,        0,     0, 0.025177};
I20FeedbackSignal fbs1204   = {I20_PIN_A3_ADC0, I20_PIN_GND,  1, true,  I20_LINE2_CURRENT,        0,     0, 0.025177};
I20FeedbackSignal fbs1205   = {I20_PIN_A0_ADC0, I20_PIN_GND,  2, false, I20_BATTTOP_VOLTAGE, 820000,  4460, 0.0};
I20FeedbackSignal fbs1206   = {I20_PIN_A1_ADC0, I20_PIN_GND,  2, false, I20_BATTMID_VOLTAGE, 820000,  4460, 0.0};
I20Feedback fb_1adc0_2s0_2s1_2s2  = {adcNumBits, adcPrescale, adcSampleTicks, adcVRefNdx, defaultAdcVRef,
                                     {&fbs1201, &fbs1202, &fbs1203, &fbs1204, &fbs1205, &fbs1206 }};

// ADC0 and ADC1 in parallel 1 reading each
I20FeedbackSignal fbs1301   = {I20_PIN_A4_ADC0, I20_PIN_GND,  0, false, I20_LINE1_VOLTAGE,   820000,  7400, 0.0};
I20FeedbackSignal fbs1302   = {I20_PIN_A2_ADC1, I20_PIN_GND,  0, true,  I20_LINE2_VOLTAGE,   820000,  7400, 0.0};
I20Feedback fb_2adc_2v      = {adcNumBits, adcPrescale, adcSampleTicks, adcVRefNdx, defaultAdcVRef,
                               {&fbs1301, &fbs1302, NULL, NULL, NULL }};

I20InputParams defaultParams = {invArch,         // inverter architecture
                                outRmsVoltage,   // RMS voltage of output lines
                                outCurrent,      // output current
                                numLines,        // number of output lines
                                outputFreq,      // output frequency, either 50 or 60 Hz
                                pwmFreq,         // PWM frequency
                                I20_PS_TCC1,     // Primary TCC configuration to use
                                I20_PS_TCC0,     // Secondary TCC configuration to use
                                deadTimeNs,      // dead time between MOSFET transitions
                                &fb_2adc_2v,     // Any of the feedback configs above
                              };
I20InputParams inParams;

IcosaLogic_Inverter_PWM inverter;                  // inverter object

SAMD51_Dumpster ilsd;

volatile bool inverterRunning = false;
volatile uint32_t startTicks = 0;
volatile uint32_t stopTicks = 0;


/*
 * Initialize the inverter.
 */
void setup() {
  setupSerial();
  ilsd.begin(true);

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
  inverter.adc0Handler();
}

/*
 * Handler for ADC1 RESRDY interrupt.
 */
void ADC1_1_Handler() {
  inverter.adc1Handler();
}
