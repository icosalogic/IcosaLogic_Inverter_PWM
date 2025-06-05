/*!
 * TODO list is at the top of the .cpp file.
 * 
 * An implementation of an inverter controller based on the ATMEL SAMD51 microprocessor.
 * Generates MOSFET gate signals for either a half-bridge or T-Type solution driving 1 to
 * 3 output lines.
 * It is configurable for 50 or 60 Hz output, and for different PWM frequencies.
 * 
 * Each half-bridge output line is managed by 2 N-channel MOSFETs arranged as follows:
 * 
 *     BatteryPos  ---------- Q1 --+
 *                           D S   |
 *                                 |
 *                                 +-------- Line out -----+
 *                                 |                      Load
 *     BatteryMid  ----------------|-------- Neutral ------+
 *                                 |
 *     BatteryNeg  ---------- Q2 --+
 *                           S D
 * 
 * Each T-type output line is managed by 4 N-channel MOSFETs arranged as follows:
 * 
 *     BatteryPos  ---------- Q1 --+
 *                           D S   |
 *                                 |
 *     BatteryMid  -+-- Q3 -- Q4 --+-------- Line out -----+
 *                  |  S D   D S   |                      Load
 *                  +--------------|-------- Neutral ------+
 *                                 |
 *     BatteryNeg  ---------- Q2 --+
 *                           S D
 * 
 * On the positive half of a sine wave, Q1 and Q3 are PWM'ed as a pair, Q4 is on, Q2 is off.
 * On the negative half, Q4 and Q2 are PWM'd as a pair, Q3 is on, Q1 is off.
 *
 * The implementation details of this T-type architecture are as follows:
 *   - Q1 and Q2 share the same positions as in a half-bridge architecture
 *   - Q3 for all lines share a single supply
 *   - Q4 and Q1 for each line share a single supply, possibly bootstrapped
 *   - Q2 for all lines share a single supply.
 * 
 * We take advantage of the SAMD51 TCC implementation to provide many of the features.
 * (This code may also work on a SAMD21.  Most of the low-level hardware components
 * (TCC, ADC) are compatible.)
 */

#include "Arduino.h"

#ifndef _ICOSALOGIC_INVERTER_TT_
#define _ICOSALOGIC_INVERTER_TT_

// Errors detected by the inverter
typedef enum {
  I20_ERR_NONE,                                                        // 0
  I20_ERR_INVALID_NUMLINES_VALUE,
  I20_ERR_INVALID_TCCCFGNDX_VALUE,
  I20_ERR_TCC_BUSY,
  I20_ERR_NUMLINES_TOO_BIG_FOR_TCC,
  I20_ERR_PWM_FREQ_MAX_EXCEEDED,
  I20_ERR_INVALID_OUT_FREQ_VALUE,
  I20_ERR_UNABLE_TO_COMPUTE_TCC_PRESCALER_AND_PERIOD,
  I20_ERR_NUMSAMPLES_MAX_EXCEEDED,
  I20_ERR_NUMSAMPLES_MUST_BE_GREATER_THAN_0,
  I20_ERR_INVALID_NUMADCBITS_VALUE,                                    // 10
  I20_ERR_INVALID_ADC_PRESCALE_VALUE,
  I20_ERR_INVALID_ADC_SAMPLE_TICKS,
  I20_ERR_REQUIRED_FEEDBACK_IS_MISSING,
  I20_ERR_INVALID_ADC_NUMBER,
  I20_ERR_REUSED_ADC_INPUT_PIN,
  I20_ERR_NUM_ADC_SCHEDULE_ENTRIES_MAX_EXCEEDED,
  I20_ERR_INVALID_ADC_NEG_REF,
  I20_ERR_POS_AND_NEG_ON_DIFFERENT_ADC,
  I20_ERR_ADC_DIFF_MODE_NEG_PIN_VBAT_MID_CONFLICT,
  I20_ERR_ADC0_OVERRUN_MAX_EXCEEDED,                                   // 20
  I20_ERR_ADC1_OVERRUN_MAX_EXCEEDED,
  I20_ERR_NULL_ADC_SCHEDULE_ENTRY,
  I20_ERR_DEADTIME_MAX_EXCEEDED,
  I20_ERR_NOT_READY_TO_START,
  I20_ERR_ADC_SIBLING_DO_PWM_CONFLICT,
  I20_ERR_ADC_DO_PWM_MULTIPLE_POS,
  I20_ERR_ADC_DO_PWM_ON_MULTIPLE_SCHEDULES,
  I20_ERR_ADC_DO_PWM_NOT_SET,
  I20_ERR_ADC_SCHEDULE_TIME_LONGER_THAN_PWM_TIME,
  I20_ERR_REQUESTED_RESOURCES_EXCEEDS_AVAILABLE,
  I20_ERR_INCONSISTENT_RESISTOR_DIVIDERS,
  I20_ERR_PEAK_DIVIDED_VOLTAGE_EXCEEDS_ADC_VREF,
  I20_ERR_NEG_ADC_CHANNEL_INVALID,
  I20_ERR_PEAK_CURRENT_EXCEEDS_ADC_LIMIT,
  I20_ERR_LAST_ERROR_NUMBER,                            // keep this entry last
} I20_ERROR_NUMBER;

/*
 * Enums defining parameter values that control inverter operation.
 */
 
const int maxNumLines = 3;
const uint32_t maxPwmFrequency = 180060;            // 180k
const uint16_t maxNumSamples = 3600;                // 3600 for 180k pwmFreq
const uint16_t maxDeadtimeTicks = 255;
const int maxNumFbSignals = 16;
const float defaultAdcVRef = 3.3;

/**! Specify the architecture of the inverter for which we are generating signals.
 *   Half-bridge designs use 2 MOSFETs per output line.
 *   T-type designs use 4 MOSFETs per output line. */
enum I20InvArch {
  I20_HALF_BRIDGE,
  I20_T_TYPE,
  I20_DC              // half bridge DC-DC converter
};

/**! Specify what type, if any, half-wave signal is generated per output line.
 *   In a T-type inverter, this is useful for the high-side gate driver bootstrap circuit. */
enum I20HalfWaveSignal {
  I20_HWS_NONE,
  I20_HWS_SINGLE,
  I20_HWS_PAIR           // includes DTI between transitions
};

/**! Determines the output pins and TCC instance for the current platform */
enum I20TccCfgNdx {
  I20_PS_NONE,
  I20_PS_TCC0,
  I20_PS_TCC0_ALT,
  I20_PS_TCC1,
  I20_PS_TCC1_ALT,
};

enum I20FeedbackType {
  I20_FB_UNDEFINED,
  I20_LINE1_CURRENT,
  I20_LINE1_VOLTAGE,
  I20_LINE2_CURRENT,
  I20_LINE2_VOLTAGE,
  I20_LINE3_CURRENT,
  I20_LINE3_VOLTAGE,
  I20_BATTTOP_VOLTAGE,
  I20_BATTMID_VOLTAGE,
};

// ADC config for a feedback signal (battery voltages or output line voltage/current).
typedef struct {
  uint16_t           adcPinPos;                 // Index into inverterAdcCfg for positive; platform-specific
  uint16_t           adcPinNeg;                 // Index for negative (!= ground for diff ADC readings)
  uint16_t           pos;                       // position in the ADC schedule: 0..n
  bool               doPwmHandler;              // update PWM duty cycle after this reading is done
  I20FeedbackType    fbType;                    // type of feedback, this signal also provides lineNum
  uint32_t           vrTop;                     // voltage: divider top resistance
  uint32_t           vrBottom;                  // voltage: divider bottom resistance
  float              aLsb;                      // current: amps per LSB
} I20FeedbackSignal;
  
// Feedback mechanism for each output line, and top and midpoint of battery.
typedef struct {
  uint8_t            adcNumBits;                // ADC number of bits: 12, 10, or 8
  uint16_t           adcPrescaleVal;            // ADC clock prescale value
  uint16_t           adcSampleTicks;            // ADC clock ticks to hold sample
  eAnalogReference   adcVRefNdx;                // ADC reference to use
  float              extVRefValue;              // voltage of external VRef, if selected
  I20FeedbackSignal* signal[maxNumFbSignals];   // voltage and current feedback
} I20Feedback;

// Input parameters
typedef struct {
  I20InvArch         invArch;                   // Inverter architecture
  I20HalfWaveSignal  hws;                       // Half wave signal type
  float              outVoltage;                // target output voltage
  float              outCurrent;                // max output current
  uint8_t            numLines;                  // number of output lines
  uint8_t            outFreq;                   // output frequency, either 50 or 60 Hz
  uint32_t           pwmFreq;                   // PWM frequency
  I20TccCfgNdx       tccCfgNdx1;                // Preferred configuration to use, either TCC0 or 1
  I20TccCfgNdx       tccCfgNdx2;                // Second configuration to use, either TCC1 or 0
  uint16_t           deadTimeNs;                // dead time between MOSFET transitions
  I20Feedback*       feedback;                  // voltage and current feedback
} I20InputParams;

/**! Data used in the interrupt handlers to dynamically control scaling of the PWM duty cycle. */
typedef struct {
  Tcc*               pwmTcc;                    // PWM TCC for this line
  Tcc*               hwsTcc;                    // HWS TCC for this line
  I20FeedbackSignal* cfb;                       // current feedback
  I20FeedbackSignal* vfb;                       // voltage feedback
  uint16_t           aMaxDc;                    // Max current ADC reading for DC converter
  uint16_t           vMaxDc;                    // Max voltage ADC reading for DC converter
  uint16_t           chNum;                     // base channel number for this line
  uint16_t           hwsChNum;                  // HWS channel number
  uint16_t           sineNdx;                   // index into reference sine wave array
  uint16_t           throttle;                  // applied to reference value to get target volts
  uint16_t           load;                      // applied to reference value to get target load 
  // should these be int16_t???
  int16_t            adcVolts0;                 // prediction period n+1
  int16_t            adcVolts1;                 // reading    period n
  int16_t            adcVolts1p;                // prediction period n
  int16_t            adcVolts2;                 // reading    period n-1
  int16_t            adcAmps0;                  // prediction period n+1
  int16_t            adcAmps1;                  // reading    period n
  int16_t            adcAmps1p;                 // prediction period n
  int16_t            adcAmps2;                  // reading    period n-1
} PerLineData;
  
/**************************************************************************/
/*!
    @brief  Class that implements an inverter.
            This implementation is tailored specifically to the SAMD51 TCC.
*/
/**************************************************************************/
class IcosaLogic_Inverter_PWM {
public:
  IcosaLogic_Inverter_PWM();

  void begin(I20InputParams* inParams);
        
  void start();
  void stop();
  bool isRunning();
  
  unsigned int getNumErrors();
  const char* getErrorText(int errIndex);
  uint16_t getErrorNum(int errIndex);
  void printErrors();
  
  // Interrupt handlers
  void tccxHandler(uint8_t tccNum);
  void adc0Handler();
  void adc1Handler();
  void dumpIrqCounts(bool reset);
  
  uint32_t getNumWaves();
  void dumpScaledSineData(bool atZero);
  void dumpLog(bool verbose);
  void dumpLogTail(bool verbose);
  void dumpLogLast(int offset, int numEntries);
  void dumpLogEntry(int i, bool verbose);
  void dumpPerLineData();
  bool validateLog();

protected:

  // General Fields ################################################################################
  
  //Configuration data
  
  I20InputParams    inputParams;
  I20Feedback*      feedback;
  I20Feedback       feedbackBuf;
  bool              useFeedback;
  
  uint16_t          numSamples;
  uint16_t          numSamplesDiv2;
  int32_t           pwmSineData1[maxNumSamples];
  int32_t           perMidPoint;
  int16_t           adcSineData[maxNumSamples];          // adjusted by resistor divider
  bool              vfbSet;
  double            resistorDivider;
  double            adcThrottleRatio;

  int16_t           vBattTopRaw;
  int16_t           vBattMidRaw;

  uint8_t           iNumLines;
  
  const int32_t     defaultMaxThrottle = 1024;           // determines granularity of adjustments
  const int32_t     defaultStartThrottle = 560;          // actual output adjustment value
  int32_t           maxThrottle;
  const int16_t     defaultMaxLoad = 1000;

  PerLineData lines[maxNumLines];
  
  // Data for a physical pin
  typedef struct {
    char     pad[8];
    int      pinNum;
    EPioType peripheral;
    uint8_t  adcNum;
    uint8_t  adcChannel;
  } I20PinData;

  const char* invArchNames[3] = {"HB", "TT", "DC"};
  const char* halfWaveSignalNames[3] = {"none", "single", "pair"};

  static const int numFeedbackTypes = 9;
  const char* feedbackTypeNames[numFeedbackTypes] =
    {"Undefined     ",
     "Line1Current  ", "Line1Voltage  ",
     "Line2Current  ", "Line2Voltage  ",
     "Line3Current  ", "Line3Voltage  ",
     "BattTopVoltage", "BattMidVoltage"};

  // TCC Fields ####################################################################################
  
  Tcc*              tccs[TCC_INST_NUM] = {TCC0, TCC1};
  static bool       tccBusy[TCC_INST_NUM];
  
  volatile uint32_t numWaves;                    // number of sine waves generated
  
  const char* I20TccCfgName[5]  = {"TCC_None", "TCC0_Pri", "TCC0_Alt", "TCC1_Pri", "TCC1_Alt"};
                         
  const int tccInterruptPriority = 1;
  
  // N.B.: The valid pin configurations are constrained by the TCC IOSET definitions.
  //       See SAMD51 datasheet section 6.2.8.5.
  
  typedef struct {
    Tcc*         tcc;                 // TCC pointer
    int          tccNum;              // TCC number
    int          iosetNum;            // IOSET used
    bool         isPreferred;
    int          numUsableTTLines;    // T-type lines (2 pairs each)
    int          numUsableChs;        // channels
    int          numUsablePairs;      // 2 pins with DTI capability
    int          numUsablePins;       // single pins
    int          pinNums[8];          // board-level pin numbers
    bool         pinsUsable[8];       // usable pins
    bool         pairsUsable[4];      // usable pairs
    bool         chUsable[6];         // usable channels
    bool         ttUsable[2];         // usable T-type lines
    int          lineTT[2];           // TT lines alloced to this timer
    int          linePair[4];         // HB lines alloced to this timer
    int          ch[6];               // channels alloced to this timer
    int          pin[8];              // single pins alloced to this timer
    int          hwsPair[4];          // HWS pairs alloced to this timer
    int          hwsCh[6];            // HWS channels alloced to this timer
    int          hwsPin[8];           // HWS pins alloced to this timer
  } InverterTimerCfg;
  
#include "platform.h"

  InverterTimerCfg  timerCfg1;                     // preferred TCC; copy of cfg referenced by tccCfgNdx1
  InverterTimerCfg  timerCfg2;                     // secondary TCC; copy of cfg referenced by tccCfgNdx2
  InverterTimerCfg  timerCfgSum;                   // total of available pins and pairs
  InverterTimerCfg  timerCfgReq;                   // total pins and pairs requested
  InverterTimerCfg* timerCfgs[2] = {&timerCfg1, &timerCfg2};
  
  typedef struct {
    uint8_t  outputFreq;      // 50 or 60 Hz
    uint8_t  gclkNum;
    uint32_t gclkFreq;
    uint32_t gclkSrcDiv;
    uint8_t  tccPrescaleKey;
    uint16_t tccPrescaleDiv;
    uint32_t tccPeriod;
    uint32_t pwmFreq;
  } InverterFreqCfg;
  
  const uint16_t tccPrescalerOptions[8] = {1, 2, 4, 8, 16, 64, 256, 1024};
  
  InverterFreqCfg syntheticFreqCfg;
  
  InverterFreqCfg *curFreqCfg = &syntheticFreqCfg;
  
  // Clock and timing for TCC
  const uint32_t Freq_100MHz = 100u * 1000u * 1000u;
  const uint32_t Freq_120MHz = 120u * 1000u * 1000u;
  const uint32_t CPU_Freq = Freq_120MHz;                 // CPU is running at 120 MHz
  const uint8_t Idx_100MHz = 2;                          // GCLK Generator num for TCC
  const uint8_t Idx_120MHz = 0;
  const uint32_t maxTccPeriod = 32768;                   // is this arbitrary?  
  float tccNsPerTick = 0.0;
  uint32_t tccPsPerTick = 0;

  // TODO: verify that all these are used.
  
  volatile uint32_t tccIrq0NumTotal = 0;                    // number of interrupts
  volatile uint32_t tccIrq0NumSvc = 0;                      // number of interrupts
  volatile uint32_t tccIrq0Flags = 0;                       // INTFLAGS at last interrupt
  
  // ADC Fields ####################################################################################
  
  // Data for configuring the size of the ADC readings.
  typedef struct {
    uint8_t  numBits;
    uint8_t  cfgValue;
    uint16_t numValues;
    uint16_t midPoint;
  } AdcResultSizeEntry;
  
  static const int numAdcResultSizeEntries = 3;
  AdcResultSizeEntry adcResultSize[numAdcResultSizeEntries] = {
    {12, ADC_CTRLB_RESSEL_12BIT_Val, 4096, 2048},
    {10, ADC_CTRLB_RESSEL_10BIT_Val, 1024,  512},
    { 8, ADC_CTRLB_RESSEL_8BIT_Val,   256,  128}
  };
  
  AdcResultSizeEntry* curAdcResultSizeEntry = NULL;
  
  typedef struct {
    eAnalogReference refNdx;
    float            refVoltage;
  } AdcRefEntry;
  
  double adcVRef = defaultAdcVRef;
  
  static const int numAdcReferenceEntries = 12;
  AdcRefEntry adcReference[numAdcReferenceEntries] = {
    {AR_DEFAULT,      defaultAdcVRef},
    {AR_INTERNAL1V0,  1.00},
    {AR_INTERNAL1V1,  1.10},
    {AR_INTERNAL1V2,  1.20},
    {AR_INTERNAL1V25, 1.25},
    {AR_INTERNAL2V0,  2.00},
    {AR_INTERNAL2V2,  2.20},
    {AR_INTERNAL2V23, 2.23},
    {AR_INTERNAL2V4,  2.40},
    {AR_INTERNAL2V5,  2.50},
    {AR_INTERNAL1V65, 1.65},
    {AR_EXTERNAL,     defaultAdcVRef},
  };
  
  const IRQn_Type adcIrqs[ADC_INST_NUM] = {ADC0_1_IRQn, ADC1_1_IRQn};
  const int adcInterruptPriority = 4;      // make this higher priority (lower value)?

  const int fbTypeToLineNum[numFeedbackTypes] = {0, 1, 1, 2, 2, 3, 3, 0, 0};
  
  // An AdcSchedule is a sequence of readings to be performed by a single ADC.
  // - Each ADC runs independently, doing it's own schedule.
  // - First ADC reading in a schedule is triggered by TCC OVF IRQ handler
  // - The ADC RESRDY IRQ handler is enabled
  // - The ADC IRQ handler reads the results of the current schedule entry, then starts the next
  // - The ADC IRQ handler runs the PWM duty cycle update after the marked entry is done
  // - Schedules are structured to finish within a single PWM cycle
  typedef struct _AdcScheduleEntry {
    struct _AdcScheduleEntry* next;
    struct _AdcScheduleEntry* sibling;
    I20FeedbackSignal*        fbs;
    I20PinData*               ipd;
    Adc*                      adc;
    int16_t*                  pResult;
    bool                      doPwmHandler;  // in fbs
    uint16_t                  inputCtrl;
    uint32_t                  tsStart;
    uint32_t                  tsDone;
  } AdcScheduleEntry;
  
  static const int maxNumSchedEntries = maxNumFbSignals;
  static const uint32_t maxOverrunErrCount = 250000;

  // schedule data for a single ADC.
  typedef struct {
    AdcScheduleEntry            entry[maxNumSchedEntries];
    AdcScheduleEntry*           firstEntry;                   // first entry to run in sched
    AdcScheduleEntry*           curEntry;                     // current entry to run in sched
    AdcScheduleEntry*           lastEntry;                    // last entry in sched
    Adc*                        adc;
    uint32_t                    scheduleTimeNs;               // min time to run schedule
    uint32_t                    resRdyErrCount;
    uint32_t                    overrunErrCount;              // took too long
    uint16_t                    numEntries;                   // number of entries in this schedule
    uint16_t                    adcNum;
    bool                        isRunning;
    bool                        doPwmDetected;                // doPwmHandler detected
    uint8_t                     doPwmPos;                     // position of doPwmHandler entry
  } AdcSchedule;
  
  AdcSchedule adc0Schedule;
  AdcSchedule adc1Schedule;
  AdcSchedule* adcSchedules[2] = {&adc0Schedule, &adc1Schedule};
  
	Adc*           adcs[ADC_INST_NUM] = {ADC0, ADC1};
  static bool    adcBusy[ADC_INST_NUM];
  
  const uint16_t AdcClockPrescaleOptions[8] = {2, 4, 8, 16, 32, 64, 128, 256};
  uint32_t AdcClockPrescale = 4;                                // Divide GCLK by this value
  uint8_t  AdcClockPrescaleCfg = ADC_CTRLA_PRESCALER_DIV4_Val;  // Config value
  
  const uint32_t Freq_48MHz = 48 * 1000u * 1000u;         // GCLK frequency used by ADC
  const uint8_t Idx_48MHz = 1;                            // GCLK Generator num for ADC
  uint32_t adcConversionNs = 0;                           // time to do 1 ADC reading, in ns
  
  static const int maxNumAdcPins = 128;
  static bool adcPinBusy[maxNumAdcPins];
  
  static const int maxNumBoardPins = 256;
  static bool boardPinBusy[maxNumBoardPins];              // TODO: finish implementing this
  
  // Schedule battery voltage readings at the min point of line 0, only by line 0,
  // since the battery is a resource shared by all the lines.
  
  // Protected Methods #############################################################################

  void setupErrors();
  void setError(uint16_t errNum);

  void setupInput(I20InputParams* inParams);
  void setupBasics();
  void setupOutput();
  void setupOutputPins();
  void sumTimerResources();
  void setupTTLine(int lineNum);
  void setupPinPair(int lineNum, char prefix, int ndx1, int ndx2);
  bool setupPinPair(InverterTimerCfg* itc, int lineNum, char prefix, int ndx1, int ndx2);
  void setupPin(int lineNum, char prefix, int pinNum);
  void setupPin(InverterTimerCfg* itc, int lineNum, char prefix, int pinNum);
  void setupOutputPin(InverterTimerCfg* itc, int pinNum, int lineNum, char prefix, int qNum);
  void setupSineWaveIndices();
  void checkNumSamples();
  void genSineWaveData();
  void setupFreqCfg();
  void setupDc();
  
  void setupTccs();
  void setupTcc(int tccNum);
  void setupTccChannels();
  void startTcc(int tccNum);
  void stopTcc(int tccNum);
  
  // ADC setup methods
  void setupAdc();
  void setupAdcBitSize();
  void setupAdcInitCfg();
  void setupAdcSchedule();
  void setupAdcScheduleEntry(I20FeedbackSignal* fbs);
  void setupAdcInputPin(uint16_t adcPinNum, uint16_t schedPos);
  void setupAdcVfb(I20FeedbackSignal* fbs);
  void setupAdcInputCtrl(AdcScheduleEntry* entry);
  void setupAdcEnable();
  void getNextEntry(AdcSchedule* sched);
  void validateAdcSchedule(AdcSchedule* sched);
  
  // ADC run methods
  void adcScheduleStart(AdcSchedule* sched);
  void adcScheduleNext(AdcSchedule* sched);
  
  void getDestAddrResult(AdcScheduleEntry* entry);
  void displayAdcSchedule(AdcSchedule* pSched);
  void displayAdcScheduleEntry(AdcScheduleEntry* pEntry, int i);
  
  inline bool adcResultsAvailable();
  inline void pwmHandler();

  // void dumpLogEntry(LogEntry* entry, bool verbose);
  void dumpPerLineDataEntry(uint8_t i, PerLineData* pld);

  // Error info     ################################################################################
  
  static const int numErrDefns = I20_ERR_LAST_ERROR_NUMBER;
  const char* errText[numErrDefns] = {
    "no error",                                                                               //   0
    "invalid numLines value",
    "Invalid tccCfgNdx value",
    "TCC is already busy",
    "selected TCC does not support number of lines",
    "PWM frequency exceeds maximum",
    "output frequency must be either 50 or 60 Hz",
    "could not determine TCC prescaler and period",
    "numSamples exceeds maximum",
    "numSamples must be greater than 0",
    "invalid numAdcBits value (valid values are 8, 10, 12)",                                  //  10
    "invalid ADC prescale value (must be power of 2, 2 <= x <= 256)",
    "invalid ADC sample ticks value (must be 1 <= x <= 64)",
    "required feedback is missing",
    "invalid ADC number",
    "reused ADC input pin",
    "too many AdcScheduleEntries",
    "invalid ADC negative reference (I20_PIN_NONE)",
    "pos and neg are on different ADCs",
    "ADC negative pin must be vBattMid (differential mode only)",
    "too many ADC 0 overruns",                                                                //  20
    "too many ADC 1 overruns",
    "ADC schedule entry is null",
    "deadTimeNs exceeds maximum",
    "inverter not ready to start",
    "all siblings must have the same doPwmHandler value",
    "an ADC schedule can set doPwmUpdate in at most one position",
    "only one ADC schedule can set doPwmUpdate",
    "doPwmUpdate is not set in any ADC schedule",
    "ADC schedule time longer than PWM cycle time",
    "Requested resources exceeds available resources",
    "Resistor dividers must be consistent for all lines",
    "Peak voltage through resistor divider exceeds ADC voltage reference",
    "Negative pin for differential ADC reading must have a channel number <= 7",
    "Peak current divided by aLsb exceeds ADC maximum",
  };

  static const int maxNumErrors = 100;
  int numErrors = 0;
  uint16_t errNums[maxNumErrors];

};
#endif // _ICOSALOGIC_INVERTER_TT_
