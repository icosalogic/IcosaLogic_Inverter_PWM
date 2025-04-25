/*!
 * DESCRIPTION:
 * 
 * Inverter application utilizing minutia of SAMD51 peripherals.
 * Implements either a half-bridge or T-Type inverter architecture.
 * 
 * This inverter has the following features:
 * - 1, 2, or 3 output lines.
 * - 50 or 60 Hz output frequency
 * - configurable PWM frequency from 400 Hz to 120 kHz
 * - configurable dead time
 * - flexible scheduling of ADC readings during each PWM cycle
 * 
 * This implementation uses the following SAMD51 hardware peripherals:
 * - TCC (Timer/Counter for Control applications)
 * - ADC (Analog to Digital Conversion)
 * - NVIC (Nested Vector Interrupt Controller)
 * - DWT (Debug Watchdata and Trace) (not to be confused with WDT (Watch Dog Timer))
 * 
 * Once configured and started, no loop() code is required.
 * 
 * 
 * TODO:
 * - Adjust throttle according to output
 * - Save multiple feedback sets, use predictor to set throttle
 * - Desired output value
 * - Adjustable mid point
 * - Adjustable phase correction
 * - Implement enable input pin
 * - Investigate stop/fault settings to turn off all outputs.
 * - More precise control of outFreq down to cHz:
 *   + change outFreq from uint8_t to uint16_t; switch units from Hz to cHz
 *   + integrate with Si5351 clock chip to generate GCLK input source (not possible on small boards)
 *   + switch the TCCs to use the GCLK driven by the Si5351
 * 
 * 
 * 
 * Theory of Operation:
 * 
 * The user selects an output frequency (either 50 or 60 Hz) and a PWM frequency (matched to the
 * hardware components used in the physical construction of the inverter.
 * 
 * The inverter setup code configures a TCC instance (only TCC0 or TCC1 have all the features we
 * require) to drive the PWM gate signals for the 4 MOSFETs employed for each inverter output line.
 * 
 * One of the primary goals during setup is to configure a clock mechanism to drive the TCC at the
 * desired frequency.  There are two approaches for this:
 * 
 * 1. Use one of the established SAMD51 clocks.  This approach often doesn't yeild a predictable
 *    output frequency; it's often off by a few percent.  If that is acceptable, this is the
 *    least effort solution.
 * 2. TODO: Use an external Si5351 clock chip.  This is the most stable and accurate solution.
 * 
 * Once a clock source has been identified, a GCLK instance, peripheral clock, the TCC prescaler
 * and the period count are configured to obtain the requested PWM frequency.
 * 
 * Another constraint on which TCC to use involves the IOSET pin mappings available in the board
 * you are using.  Basically, any given TCC can drive some number of output pins.  For each TCC,
 * there are up to 6 specific sets of pins that can be used.  The board manufacturer selects one
 * of these configurations when they wire the CPU leads to the board pins.  You must use pins from a
 * single IOSET, you can't mix and match.  These are given by the pad number on the silicon die, and
 * different chip packages will map different subsets of the pads to physical leads on the chip.
 * Board manufacturers will then map different chip leads to physical pins.  See the SAMD51
 * datasheet section titled "I/O Multiplexing and Considerations" for more detail about the TCC
 * IOSETs.  See your board manufacturer documentation to determine which board pins are mapped to
 * which chip leads.
 * 
 * Another consideration is the mapping of pins to ADC instances.  The feedback configuration
 * passed in begin() should try to balance output line voltage and current readings equally on ADC0
 * and ADC1, to minimize the time required to obtain these values during a PWM cycle.  Again, this
 * depends on which of the available board pins are mapped to which ADC.
 * 
 * Note that the Arduino port for some boards did not provide access to ADC1.  This project includes
 * updated variant.h and variant.cpp files to resolve this issue.
 * 
 * Once the period count is known, the reference sine wave data is generated.  At run time, each
 * line is individually scaled (throttled) at each PWM cycle to compensate for varying loads and
 * battery levels.
 * 
 * The 50 and 60 Hz PWM frequency configurations use the same frequency configurations, we just use
 * the 120 MHz clock for 60 Hz, and the 100 MHz clock for 50 Hz.  Everything else is the same.
 * This simplifies supporting a large number of PWM frequency options.
 * 
 * We configure two TCC CC channels for each output line, and one additional CC channel to precisely
 * start ADC operations and duty cycle updates during each PWM cycle.  Since we are using the DTI
 * feature in the TCC, it automatically maps two WO[x] pins for each of the first 4 TC channels.
 * 
 * We take advantage of the TCC DTI (Dead Time Insertion) feature to provide the dead time between
 * MOSFET gate signal transitions.  Use of DTI restricts us to TCC0 and TCC1 on the SAMD51, since
 * none of the other TCC instances support DTI.
 * 
 * Details of each PWM period:
 * a. TCC OVF triggered when TCC counter matches PERIOD.  TCC counter is reset, and CCBUF values are
 *    copied to CC for each CC channel driving an output line (e.g., duty cycle is updated)
 * b. TCC OVF event triggers conditional battery ADC readings, if set.
 * f. The TCC MC interrupt handler updates the PWM duty cycle for each line.  The voltage and
 *    current readings are used to update the throttle setting, then the throttle is used to scale
 *    the reference sine wave value to obtain the duty cycle for the next PWM cycle, which is
 *    written to the CCBUF field of each output line CC channel.
 * g. The timestamps are checked to ensure the interrupt handler completes before the end of the
 *    PWM cycle.  The CCBUF value for CC3 is adjusted as necessary.  This should rarely happen
 *    after the first few cycles.
 * h. Depending on our position in the entire sine wave, enable or disable the ADC conditional
 *    readings for the battery voltages.
 * 
 * It is not known if this code could be easily ported to one of the other SAMD chips, e.g.,
 * the SAMD21.
 */

#include "IcosaLogic_Inverter_PWM.h"
#include <wiring_private.h>
#include <math.h>
#include "tcc_ioset.h"
#include "logging.h"


extern "C" {


}

bool IcosaLogic_Inverter_PWM::tccBusy[TCC_INST_NUM];
bool IcosaLogic_Inverter_PWM::adcBusy[ADC_INST_NUM];
bool IcosaLogic_Inverter_PWM::adcPinBusy[maxNumAdcPins];
bool IcosaLogic_Inverter_PWM::boardPinBusy[maxNumBoardPins];


IcosaLogic_Inverter_PWM::IcosaLogic_Inverter_PWM() {
}

/**************************************************************************/
/*!
    @brief  Configure an Inverter class.
    @param inParams Pointer to an I20IinputParams structure containing all the parameters.
*/
/**************************************************************************/
void IcosaLogic_Inverter_PWM::begin(I20InputParams* inParams) {
  Serial.printf("\n");
  Serial.printf("IcosaLogic_Inverter_PWM::begin: enter ------------------------------");
  Serial.flush();
  Serial.printf("-------------------------------------------------\n");
  
  setupErrors();
  setupLog();
  setupInput(inParams);
  
  setupBasics();
  if (numErrors > 0) {
    return;
  }

  // set up the output pins
  setupOutput();
  if (numErrors > 0) {
    return;
  }
  
  setupSineWaveIndices();
  
  // dumpPerLineData();
  
  setupTccs();
  if (numErrors > 0) {
    return;
  }
  
  setupAdc();
}

/*
 * Copy the provided params to internal variables so they can be adjusted if necessary.
 */
void IcosaLogic_Inverter_PWM::setupInput(I20InputParams* inParams) {
  // make a copy of all input parameters, then print them out
  memcpy(&inputParams, inParams, sizeof(I20InputParams));
  
  Serial.printf("    arch %d [%s]  hws %d [%s]  ",
                inParams->invArch, invArchNames[inParams->invArch], 
                inParams->hws, halfWaveSignalNames[inParams->hws]);
  Serial.flush();
  Serial.printf("vOut %d  numLines %d  outFreq %d  pwmFreq %d\n",
                inParams->outRmsVoltage, inParams->numLines, inParams->outFreq, inParams->pwmFreq);
  Serial.flush();
  Serial.printf("    tccCfgNdx1 %d [%s]  tccCfgNdx2 %d [%s]  deadTimeNs %d  ",
                inParams->tccCfgNdx1, I20TccCfgName[inParams->tccCfgNdx1],
                inParams->tccCfgNdx2, I20TccCfgName[inParams->tccCfgNdx2], inParams->deadTimeNs);
  Serial.flush();
  Serial.printf("adcPrescale %d  adcSampleTicks %d\n",
                inParams->adcPrescaleVal, inParams->adcSampleTicks);
  
  uint32_t pwmCpuTicks = CPU_Freq / inParams->pwmFreq;
  double   pwmUs = 1e6 / (double) inParams->pwmFreq;
  Serial.printf("    PWM  %u Hz  %u CPU ticks  ", inParams->pwmFreq, pwmCpuTicks);
  Serial.print(pwmUs, 3);
  Serial.printf(" us\n");
  
  feedback = inParams->feedback;
  useFeedback = inParams->feedback != NULL;
  if (useFeedback) {
    memcpy(&feedbackBuf, inParams->feedback, sizeof(I20Feedback));
    feedback = &feedbackBuf;
    inputParams.feedback = feedback;
  }
  maxThrottle   = defaultMaxThrottle;
}

/**
 * Validate the timer config and initialize basic member variables.
 */
void IcosaLogic_Inverter_PWM::setupBasics() {
  // Serial.printf("setupBasics: enter\n");
  
  for (int i = 0; i < TCC_INST_NUM; i++) {                // Mark TCCs as not busy
    tccBusy[i] = false;
  }
  for (int i = 0; i < ADC_INST_NUM; i++) {                // Mark ADCs as not busy
    adcBusy[i] = false;
  }
  for (int i = 0; i < maxNumAdcPins; i++) {               // Mark ADC pins as not busy
    adcPinBusy[i] = false;
  }
  for (int i = 0; i < maxNumBoardPins; i++) {             // Mark board pins as not busy
    boardPinBusy[i] = false;
  }
  
  if (inputParams.numLines < 1 || inputParams.numLines > maxNumLines) {
    setError(I20_ERR_INVALID_NUMLINES_VALUE);
    return;
  }
  
  iNumLines = (int) inputParams.numLines;
  for (int i = 0; i < iNumLines; i++) {
    lines[i].throttle = defaultStartThrottle;
  }
  
  setupFreqCfg();
  
  genSineWaveData();
}


/**
 * Set up the hardware output pins.
 * 
 * There are 2 types of output signals:
 *     1. MOSFET gate signals
 *     2. Half wave signals (HWS) (high for the 1st half of the sine wave, low for the 2nd half).
 * 
 * For each line, we set up the gate signal output pins according to the architecture type:
 *     - half-bridge: 1 pair [Q1,Q2] with configurable dead time (DTI) between transitions
 *     - T-type: 2 pairs [Q1,Q3],[Q4,Q2] with DTI between transitions in each pair.
 *               See comment at the top of IcosaLogic_Inverter.h for arrangement of MOSTFETs.
 * 
 * For the SAMD51, we use both TCC0 and TCC1, which have the following configurations:
 * 
 *           Pins  Channels  Pairs    PeriodBits    DTI_Supported
 *     TCC0    8       6       4          24            true
 *     TCC1    8       4       4          24            true
 * 
 * TCC0 and TCC1 are connected together as primary/secondary, so the counter on TCC0 controls
 * all the channels on both TCCs.
 * 
 * The TCC maps the channels and output pins as follows:
 * 
 *     Channel     SinglePin  PinPair
 *        0            0       [0,4]
 *        1            1       [1,5]
 *        2            2       [2,6]
 *        3            3       [3,7]
 *        4            4
 *        5            5
 *        0            6
 *        1            7
 * 
 * So, the maximum configuration has 2 TCCs, 10 channels, 16 pins, and 8 pairs.  Note that
 * many boards do not support a maximum configuration because of inadequate connections from
 * the TCC to the pad on the die to the pin on the chip package to the pin on the board.
 * One board that easily supports a maximum configuration is the Adafruit Grand Central M4
 * Express.
 * 
 * Because of this mapping, we have a very strong preference to use TCC1 for allocating the
 * first 4 pairs, because it consumes exactly 4 channels for 4 pairs.  The next preference
 * is to allocate TCC0 pairs [2,6] and [3,7] because it uses 2 channels for 2 pairs.
 * The least preferred is allocating TCC0 pairs [0,4] and [1,5], since each pair consumes
 * 2 channels.  I.e., channels 4 and 5 can't be used to generate output when channels 0 and 1
 * are used to generate output on pairs [0,4] and [1,5].
 * 
 * We optionally generate a half wave signal for each line.  This may be either a single pin
 * or a pair with DTI.
 * 
 * Common configurations:
 *     Half Bridge
 *         1 Line  -- L1 TCC1 [0,4]
 *         2 Lines -- L1 TCC1 [0,4]  L2 TCC1 [1,5]
 *         3 Lines -- L1 TCC1 [0,4]  L2 TCC1 [1,5]  L3 TCC1 [2,6]
 *     T-Type
 *         1 Line  -- L1 TCC1 [0,4],[1,5]                                            HWS TCC0 0 or [0,4]
 *         2 Lines -- L1 TCC1 [0,4],[1,5]  L2 TCC1 [2,6],[3,7]                       HWS TCC0 0 or [0,4]
 *         3 Lines -- L1 TCC1 [0,4],[1,5]  L2 TCC1 [2,6],[3,7]  L3 TCC0 [2,6],[3,7]  HWS TCC0 0, 1, 4
 * 
 * Note that setting up the analog input pins for control is done in setupAdc().
 */
void IcosaLogic_Inverter_PWM::setupOutput() {
  // Serial.printf("\nsetupOutput: enter\n");
  
  // The primary TCC config is not optional
  if (inputParams.tccCfgNdx1 == I20_PS_NONE || timerCfg[inputParams.tccCfgNdx1] == NULL) {
    setError(I20_ERR_INVALID_TCCCFGNDX_VALUE);
    return;
  }
  memcpy(&timerCfg1, timerCfg[inputParams.tccCfgNdx1], sizeof(InverterTimerCfg));
  tccBusy[timerCfg1.tccNum] = true;
  
  if (timerCfg[inputParams.tccCfgNdx2] == NULL) {
    memset(&timerCfg2, 0, sizeof(InverterTimerCfg));
  } else {
    memcpy(&timerCfg2, timerCfg[inputParams.tccCfgNdx2], sizeof(InverterTimerCfg));
    // Verify that the second TCC is not already active
    if (tccBusy[timerCfg2.tccNum]) {
      setError(I20_ERR_TCC_BUSY);
      return;
    }
    tccBusy[timerCfg2.tccNum] = true;
  }
  
  sumTimerResources();
  if (numErrors > 0) {
    return;
  }

  setupOutputPins();
}

/**
 * Sum up the available pin and pin pair counts from the 2 timer configs in to a single
 * config.
 */
void IcosaLogic_Inverter_PWM::sumTimerResources() {
  // Serial.printf("sumTimerResources: enter\n");
  
  // Sum the available resources from the timerCfgs given in input params
  timerCfgSum.numUsableTTLines = timerCfg1.numUsableTTLines + timerCfg2.numUsableTTLines;
  timerCfgSum.numUsableChs     = timerCfg1.numUsableChs     + timerCfg2.numUsableChs;
  timerCfgSum.numUsablePairs   = timerCfg1.numUsablePairs   + timerCfg2.numUsablePairs;
  timerCfgSum.numUsablePins    = timerCfg1.numUsablePins    + timerCfg2.numUsablePins;
  
  // Compute the number of pins and pin pairs requested for this config, first for output pins
  memset(&timerCfgReq, 0, sizeof(InverterTimerCfg));
  if (inputParams.invArch == I20_HALF_BRIDGE) {
    // half bridge architecture
    timerCfgReq.numUsableChs   += iNumLines;
    timerCfgReq.numUsablePairs += iNumLines;
    timerCfgReq.numUsablePins  += iNumLines * 2;
  } else {
    // T-type architecture
    timerCfgReq.numUsableTTLines += iNumLines;
    timerCfgReq.numUsableChs     += iNumLines * 2;
    timerCfgReq.numUsablePairs   += iNumLines * 2;
    timerCfgReq.numUsablePins    += iNumLines * 4;
  }
  
  // compute pins and pairs requested for optional half wave signal
  if (inputParams.hws == I20_HWS_SINGLE) {
    timerCfgReq.numUsablePins += iNumLines;
  } else if (inputParams.hws == I20_HWS_PAIR) {
    if (iNumLines <= 2) {
      // Special case: need only 1 pair for either 1 or 2 lines
      timerCfgReq.numUsableChs   += 1;
      timerCfgReq.numUsablePairs += 1;
      timerCfgReq.numUsablePins  += 2;
    } else {
      // use 1 pair each for 3 lines
      timerCfgReq.numUsableChs   += iNumLines;
      timerCfgReq.numUsablePairs += iNumLines;
      timerCfgReq.numUsablePins  += iNumLines * 2;
    }
  }
  
  Serial.printf("    output pin resources requested:  TT %d  CH %2d  pairs %2d  pins %2d\n",
                timerCfgReq.numUsableTTLines, timerCfgReq.numUsableChs,
                timerCfgReq.numUsablePairs, timerCfgReq.numUsablePins);
  Serial.printf("                         available:  TT %d  CH %2d  pairs %2d  pins %2d\n",
                timerCfgSum.numUsableTTLines, timerCfgSum.numUsableChs,
                timerCfgSum.numUsablePairs, timerCfgSum.numUsablePins);

  // verify that the resources requested do not exceed what is available
  if (timerCfgReq.numUsableTTLines > timerCfgSum.numUsableTTLines ||
      timerCfgReq.numUsablePairs   > timerCfgSum.numUsablePairs   ||
      timerCfgReq.numUsablePins    > timerCfgSum.numUsablePins) {
    setError(I20_ERR_REQUESTED_RESOURCES_EXCEEDS_AVAILABLE);
    return;
  }
}

/**
 * Setup the output pins for line output, and half wave signals.
 */
void IcosaLogic_Inverter_PWM::setupOutputPins() {
  // Serial.printf("setupOutputPins: enter\n");
  
  // Allocate output pins
  for (int lineNum = 1; lineNum <= iNumLines; lineNum++) {
    if (inputParams.invArch == I20_HALF_BRIDGE) {
      setupPinPair(lineNum, 'Q', 1, 2);
    } else {
      setupTTLine(lineNum);
    }
  }
  
  // Allocate half wave signal (HWS) pins
  // Serial.printf("\n    half wave signals\n");
  if (inputParams.hws == I20_HWS_SINGLE) {
    // alloc a single pin for each line
    for (int lineNum = 1; lineNum <= iNumLines; lineNum++) {
      setupPin(lineNum, 'H', 1);
    }
  } else if (inputParams.hws == I20_HWS_PAIR) {
    if (iNumLines < 3) {
      // special case, only 1 pair allocated for either 1 or 2 lines
      setupPinPair(1, 'H', 1, 2);
    } else {
      // alloc a pair for each line
      for (int lineNum = 1; lineNum <= iNumLines; lineNum++) {
        setupPinPair(lineNum, 'H', 1, 2);
      }
    }
  }
}

/**
 * Setup a T-type inverter line.  This sets up 2 pairs, 4 pins total.
 */
void IcosaLogic_Inverter_PWM::setupTTLine(int lineNum) {
  // Serial.printf("setupTTLine: enter\n");
  
  for (int tccNdx = 0; tccNdx < 2; tccNdx++) {
    InverterTimerCfg* itc = timerCfgs[tccNdx];
    for (int ttlNdx = 0; ttlNdx < 2; ttlNdx++) {
      if (itc->ttUsable[ttlNdx] && itc->lineTT[ttlNdx] == 0) {
        // found an unallocated TT line, mark TT and pairs as allocated to this lineNum
        itc->lineTT[ttlNdx] = lineNum;
        setupPinPair(itc, lineNum, 'Q', 1, 3);
        setupPinPair(itc, lineNum, 'Q', 4, 2);
        PerLineData* pld = &lines[lineNum - 1];
        pld->chNum -= 1;    // use the 1st channel allocated as the chNum
        return;
      }
    }
  }
}

/**
 * Set up a pair of output pins.
 */
void IcosaLogic_Inverter_PWM::setupPinPair(int lineNum, char prefix, int qNumA, int qNumB) {
  // Serial.printf("setupPinPair: enter\n");
  
  bool done = false;
  for (int tccNdx = 0; !done && tccNdx < 2; tccNdx++) {
    InverterTimerCfg* itc = timerCfgs[tccNdx];
    done = setupPinPair(itc, lineNum, prefix, qNumA, qNumB);
  }
}
  
/**
 * Set up a pair of output pins.
 */
bool IcosaLogic_Inverter_PWM::setupPinPair(InverterTimerCfg* itc, int lineNum, char prefix, int qNumA, int qNumB) {
  // Serial.printf("setupPinPair: 2 enter tccNum %d lineNum %d\n", itc->tccNum, lineNum);
  
  for (int pairNdx = 0; pairNdx < 4; pairNdx++) {
    if (itc->pairsUsable[pairNdx] && itc->linePair[pairNdx] == 0) {
      // found an unallocated pair, mark it as allocated to this lineNum
      itc->linePair[pairNdx] = lineNum;
      itc->ch[pairNdx]       = lineNum;
      PerLineData* pld = &lines[lineNum - 1];
  /*  if (pld->pwmTcc == NULL) {
        setError(I20_ERR_INVALID_TCCCFGNDX_VALUE);
        return false;
      } */
      if (prefix == 'Q') {
        // gate signal
        pld->pwmTcc = tccs[itc->tccNum];
        pld->chNum = pairNdx;
      } else if (prefix == 'H') {
        // half wave signal
        pld->hwsTcc = tccs[itc->tccNum];
        pld->hwsChNum = pairNdx;
      }
      setupOutputPin(itc, lineNum, pairNdx    , prefix, qNumA);
      setupOutputPin(itc, lineNum, pairNdx + 4, prefix, qNumB);
      return true;
    }
  }
  return false;
}
  
/**
 * Set up a single output pins.  Only used for HWS output.
 * Need to find unallocated channel, not pin!
 */
void IcosaLogic_Inverter_PWM::setupPin(int lineNum, char prefix, int qNum) {
  // Serial.printf("setupPin: enter\n");
  
  for (int tccNdx = 0; tccNdx < 2; tccNdx++) {
    InverterTimerCfg* itc = timerCfgs[tccNdx];
    for (int chNdx = 0; chNdx < itc->numUsableChs; chNdx++) {
      if (itc->chUsable[chNdx] && itc->ch[chNdx] == 0 && itc->pin[chNdx] == 0) {
        // found an unallocated TCC channel and pin
        itc->ch[chNdx]  = lineNum;
        itc->pin[chNdx] = lineNum;
        setupOutputPin(itc, lineNum, chNdx, prefix, qNum);
        PerLineData* pld = &lines[lineNum - 1];
        pld->hwsTcc = tccs[itc->tccNum];
        pld->hwsChNum = chNdx;
        return;
      }
    }
  }
}

/**
 * Set up a single output pin.  This code is extracted from analogWrite().
 */
void IcosaLogic_Inverter_PWM::setupOutputPin(InverterTimerCfg* itc, int lineNum, int pinNdx, char prefix, int qNum) {
  // Serial.printf("setupOutputPin: enter tcc %d ioset %d\n", itc->tccNum, itc->iosetNum);
  TccIoset* ioset = tccIosets[itc->tccNum]->ioset[itc->iosetNum - 1];
  // char* pad = ioset->pads[pinNdx];
  int pinNum = itc->pinNums[pinNdx];
  
  Serial.printf("    L%d %c%d output pin %2d: pad %s pioType 0x%x\n",
                lineNum, prefix, qNum, pinNum, ioset->pads[pinNdx].pad, PIO_TIMER_ALT);
  
  itc->pin[pinNdx] = lineNum;                          // mark this pin as busy
  pinPeripheral(pinNum, PIO_TIMER_ALT);

}

/**************************************************************************/
/*!
    @brief  Initialize the starting index into the sine wave array for each
    line.
*/
/**************************************************************************/
void IcosaLogic_Inverter_PWM::setupSineWaveIndices() {
  // Serial.printf("setupSineWaveIndices: enter\n");

  // line 0 always starts at 0
  lines[0].sineNdx = 0;
  lines[0].altNdx  = 0;
  
  if (iNumLines == 2) {
    // 2 lines, the second wave starts half way through the array
    lines[1].sineNdx = numSamples / 2;
    lines[1].altNdx  = lines[1].sineNdx;
  } else if (iNumLines == 3) {
    // 3 lines, the second wave starts at 2/3 the way, this will make it the 
    // second wave to cross the x-axis from negative to positive.
    lines[1].sineNdx = numSamples * 2 / 3;
    lines[1].altNdx = lines[1].sineNdx;
    // The third wave starts 1/3 of the way into the array
    lines[2].sineNdx = numSamples / 3;
    lines[2].altNdx = lines[2].sineNdx;
  }
  
  // reset number of sine waves generated
  numWaves = 0;
}


/*
 * Generate the InverterFreqCfg struct from outFreq and pwmFreq.
 * We point curFreqCfg to the result.
 */
void IcosaLogic_Inverter_PWM::setupFreqCfg() {
  // Serial.printf("setupFreqCfg: enter\n");
  
  InverterFreqCfg* sfc = &syntheticFreqCfg;
  
  // Adjust the PWM frequency to make it a multiple of the output frequency * number of lines
  uint32_t  pwmFreqReq = inputParams.pwmFreq;
  uint32_t pad = inputParams.outFreq % iNumLines == 0 ? inputParams.outFreq : inputParams.outFreq * iNumLines;
  inputParams.pwmFreq = ((pwmFreqReq + pad - 1) / pad) * pad;
  if (pwmFreqReq != inputParams.pwmFreq) {
    Serial.printf("    PWM frequency adjusted from %d to %d ", pwmFreqReq, inputParams.pwmFreq);
    Serial.flush();
    Serial.printf("to be a multiple of output freq and # lines\n");
  }
  
  if (inputParams.pwmFreq > maxPwmFrequency) {    // this max is a guess
    setError(I20_ERR_PWM_FREQ_MAX_EXCEEDED);
    return;
  }
  
  // Set the GCLK data based on output frequency
  sfc->pwmFreq = inputParams.pwmFreq;
  if (inputParams.outFreq == 50) {
    sfc->gclkNum = Idx_100MHz;
    sfc->gclkFreq = Freq_100MHz;
  } else if (inputParams.outFreq == 60) {
    sfc->gclkNum = Idx_120MHz;
    sfc->gclkFreq = Freq_120MHz;
  } else {
    setError(I20_ERR_INVALID_OUT_FREQ_VALUE);
    return;
  }
  sfc->outputFreq = inputParams.outFreq;
  sfc->gclkSrcDiv = 1;
  
  Serial.printf("    gclkFreq %d  outputFreq %d  gclkSrcDiv %d  maxTccPeriod %d\n",
                sfc->gclkFreq, sfc->outputFreq, sfc->gclkSrcDiv, maxTccPeriod);
  
  // Determine the TCC prescaler value to keep periods smaller than the max period value
  sfc->tccPeriod = 0;
  for (uint8_t prescaleKey = 0; prescaleKey < 8; prescaleKey++) {
    uint32_t tccPeriod = sfc->gclkFreq / (sfc->pwmFreq * tccPrescalerOptions[prescaleKey]);
    // Serial.printf("  trying key %d  div %d  tccPeriod %d\n",
    //               prescaleKey, tccPrescalerOptions[prescaleKey], tccPeriod);
    if (tccPeriod < maxTccPeriod) {
      sfc->tccPrescaleKey = prescaleKey;
      sfc->tccPrescaleDiv = tccPrescalerOptions[prescaleKey];
      sfc->tccPeriod = tccPeriod;
      break;
    }
  }
  
  if (sfc->tccPeriod == 0) {
    setError(I20_ERR_UNABLE_TO_COMPUTE_TCC_PRESCALER_AND_PERIOD);
    return;
  }
  perMidPoint = sfc->tccPeriod / 2;
  
  Serial.printf("    TCC clock div %d period %d\n", sfc->tccPrescaleDiv, sfc->tccPeriod);
  
  curFreqCfg = sfc;
}

/*
 * Generate the sine wave table based on the values in curFreqCfg.
 * We generate 2 sine wave tables:
 * 1. The ADC readings we expect to see in ADC feedback, adjusted by the resistor divider
 * 2. PWM duty cycle.  This will usually be adjusted down at runtime by the throttle.
 */
void IcosaLogic_Inverter_PWM::genSineWaveData() {
  // Serial.printf("genSineWaveData: enter\n");
  
  uint32_t startTicks = DWT->CYCCNT;
  
  numSamples = curFreqCfg->pwmFreq / curFreqCfg->outputFreq;
  numSamplesDiv2 = numSamples / 2;
  
  Serial.printf("    Generating sine wave data: pwmFreq=%d outFreq=%d numSamples=%d  ",
                curFreqCfg->pwmFreq, curFreqCfg->outputFreq, numSamples);
  Serial.flush();
    
  if (numSamples > maxNumSamples) {
    setError(I20_ERR_NUMSAMPLES_MAX_EXCEEDED);
    Serial.println(" ");
    return;
  }
  if (numSamples == 0) {
    setError(I20_ERR_NUMSAMPLES_MUST_BE_GREATER_THAN_0);
    Serial.println(" ");
    return;
  }
  
  /*
   * Positive half:              Negative half:
   * q1/q3: 0..amplitude         q1/q3: 0
   * q4/q2: tccPeriod            q4/q2: (0..-amplitude) [+ amplitude at runtime]
   * 
   * Examples:                        1, 2, 4, 8, 16, 64, 256, 1024
   *   outFreq   pwmFreq    gclkFreq  div  period   numSamples     hbRange1   hbrange2     ttRange1  ttRange2
   *      60       60000   120000000   1     2000      1000       -499..499     1..999    -999..999    0..999
   *      60      300000   120000000   1      400      5000     -2499..2499    1..4999  -4999..4999   0..4999
   */
  double amplitude = (double) (curFreqCfg->tccPeriod - 1);
  if (inputParams.invArch == I20_HALF_BRIDGE) {
    amplitude = (double) (curFreqCfg->tccPeriod / 2 - 1);
  }
  
  // Constants to save calculations in the loop
  const double pi = 3.14159265359;
  const double pi2 = 2.0 * pi;
  const double dNumSamples = numSamples;
  
  for (int i = 0; i < numSamples; i++) {
    double sineValue = sin((double) i * pi2 / dNumSamples);
    
    // generate PWM values for 100% output, it will be throttled down at runtime
    double rawSin = sineValue * amplitude;
    if (i < numSamplesDiv2) {
      pwmSineData1[i] = (int32_t)(rawSin + 0.5);
    } else {
      pwmSineData1[i] = (int32_t)(rawSin - 0.5);
    }
  }
  
  // log how long it took to generate the sine wave array
  uint32_t elapsedTicks = DWT->CYCCNT - startTicks;
  double elapsedMs = (double) elapsedTicks * 1000.0 / (double) CPU_Freq;
  Serial.printf("    Elapsed time ticks=%d  ms=", elapsedTicks);
  Serial.println(elapsedMs, 3);
}

/**
 * Configure the TCCs to drive all the output lines.
 */
void IcosaLogic_Inverter_PWM::setupTccs() {
  // Serial.printf("setupTccs: enter\n");

  // set up the GCLK for the TCC peripheral (check first to see if it is already enabled?)
  // We can only use TCC0 and TCC1, which share the same GCLK
  GCLK->PCHCTRL[TCC0_GCLK_ID].reg = curFreqCfg->gclkNum | GCLK_PCHCTRL_CHEN;
  // while (GCLK->PCHCTRL[TCC0_GCLK_ID].bit.CHEN);
  
  // Configure PWM TCC =============================================================================
  setupTcc(0);
  setupTcc(1);
  setupTccChannels();
  
  // Enable the interrupts for TCC0 only
  NVIC_SetPriority(TCC0_0_IRQn, tccInterruptPriority);
  NVIC_EnableIRQ(TCC0_0_IRQn);
  TCC0->INTENSET.bit.OVF = 1;
}

/**
 * Set up the given TCC instance.
 */
void IcosaLogic_Inverter_PWM::setupTcc(int tccNum) {
  // Serial.printf("setupTcc: enter %d\n", tccNum);

  Tcc* pwmTcc = tccs[tccNum];
  
  pwmTcc->CTRLA.bit.SWRST = 1;                           // Reset the TCC
  while (pwmTcc->SYNCBUSY.bit.SWRST);

  TCC_CTRLA_Type ctrla;
  ctrla.reg = 0;
  ctrla.bit.PRESCALER = curFreqCfg->tccPrescaleKey;      // prescale divider key
  ctrla.bit.PRESCSYNC = TCC_CTRLA_PRESCSYNC_PRESC_Val;   // sync on prescale clock
  if (tccNum == 1) {
    ctrla.bit.MSYNC = 1;                                 // sync to main TCC
  }
  pwmTcc->CTRLA.reg = ctrla.reg;

  pwmTcc->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;              // Set pwmTcc as normal PWM
  while ( pwmTcc->SYNCBUSY.bit.WAVE );

  // high and low side dead time convert from ns to clock ticks
  tccNsPerTick = 1e9 / (curFreqCfg->gclkFreq / curFreqCfg->gclkSrcDiv);
  tccPsPerTick = (uint32_t) (tccNsPerTick * 1000.0);
  // Serial.printf("    dead time is %d ps per tick, requested %d ns\n",
  //               tccPsPerTick, inputParams.deadTimeNs);
  
  if (inputParams.deadTimeNs > 0) {
    uint32_t dtTicks = (uint32_t) ((inputParams.deadTimeNs + tccNsPerTick - 1.0) / tccNsPerTick);
    if (dtTicks > maxDeadtimeTicks) {
      setError(I20_ERR_DEADTIME_MAX_EXCEEDED);
      return;
    }
    Serial.printf("    %d: dt=%d ticks  requested %d ns  actual %d ns\n",
                  tccNum, dtTicks, inputParams.deadTimeNs, (dtTicks * tccPsPerTick + 500) / 1000);
    
    TCC_WEXCTRL_Type wexctrl;
    wexctrl.reg = pwmTcc->WEXCTRL.reg;
    wexctrl.bit.DTLS = (dtTicks & 0xff);
    wexctrl.bit.DTHS = (dtTicks & 0xff);
    pwmTcc->WEXCTRL.reg = wexctrl.reg;
  }
  
  // Set PER to maximum counter value (resolution : 0xFF)
  pwmTcc->PER.reg = curFreqCfg->tccPeriod - 1u;
  while (pwmTcc->SYNCBUSY.bit.PER);
}

/**
 * Set up the initial values for the configured TCC channels.
 */
void IcosaLogic_Inverter_PWM::setupTccChannels() {
  // Serial.printf("setupTccChannels: enter\n");
  
  for (uint8_t iLine = 0; iLine < iNumLines; iLine += 1) {
    
    PerLineData *pld   = &lines[iLine];
    Tcc* tcc      = pld->pwmTcc;
    uint8_t tccCh = pld->chNum;
    // Serial.printf("    line %d TCC %08x CH %d   ", iLine + 1, tcc, pld->chNum);
    // Serial.flush();
    
    if (inputParams.deadTimeNs > 0) {
      uint32_t wexctrl = tcc->WEXCTRL.reg;
      wexctrl |= TCC_WEXCTRL_DTIEN0 << tccCh;
      if (inputParams.invArch == I20_T_TYPE) {
        wexctrl |= TCC_WEXCTRL_DTIEN0 << (tccCh + 1);
      }
      tcc->WEXCTRL.reg = wexctrl;
    }
    
    // Set the initial output value for each line
    
    int32_t throttledSineValue = (pwmSineData1[pld->sineNdx] * pld->throttle) / maxThrottle;
    
    while (tcc->SYNCBUSY.vec.CC);                     // this tests all CC busy bits
    if (inputParams.invArch == I20_HALF_BRIDGE) {
      // half bridge
      // Serial.printf("HB   initValue %d  ", throttledSineValue);
      tcc->CC[tccCh    ].reg = throttledSineValue + perMidPoint;
    } else if (pld->sineNdx < numSamplesDiv2) {
      // first half of T-type
      // Serial.printf("TT1  initValue %d  ", throttledSineValue);
      tcc->CC[tccCh    ].reg = throttledSineValue;
      tcc->CC[tccCh + 1].reg = curFreqCfg->tccPeriod;
    } else {
      // second half of T-type
      // Serial.printf("TT2  initValue %d  ", throttledSineValue + curFreqCfg->tccPeriod);
      tcc->CC[tccCh    ].reg =  0;
      tcc->CC[tccCh + 1].reg = throttledSineValue + curFreqCfg->tccPeriod;
    }
    while (tcc->SYNCBUSY.vec.CC);
    
    // Set the initial value for the optional HWS output
    
    tcc   = pld->hwsTcc;
    tccCh = pld->hwsChNum;
    if (tcc != NULL) {
      if (inputParams.deadTimeNs > 0 && inputParams.hws == I20_HWS_PAIR) {
        // enable DTI for HWS pair
        uint32_t wexctrl = tcc->WEXCTRL.reg;
        wexctrl |= TCC_WEXCTRL_DTIEN0 << tccCh;
        tcc->WEXCTRL.reg = wexctrl;
      }
    
      uint32_t hwsValue = pld->sineNdx < numSamplesDiv2 ? curFreqCfg->tccPeriod : 0;
      // Serial.printf("    HWS TCC %08x CH %d  initValue %d", tcc, tccCh, hwsValue);
      tcc->CC[tccCh].reg = hwsValue;
      while (tcc->SYNCBUSY.vec.CC);
    }
  }
  // Serial.printf("\n");
}

/*
 * Configure the ADCs to gather voltage/current feedback.
 * If there are 2 ADCs, we can run them in parallel.  If there is only 1, and we need multiple
 * readings, we will sequence them.
 * 
 * Change ADC configs from default init() in wiring.c.  Assume no prior calls to analogRead().
 * Setup prescaler, RESSEL (number of ADC bits), and SAMPCTRL (signal sample duration).
 * Clock is 48MHz.
 *         Bits  Div  ClkMHz  Sampticks  SAMPCTRL  SampUs  TotalTicks  TotalUs  Samples/Sec
 * Arduino  10    32    1.5       6          5       4.00      16       10.67      93750
 * Arduino  12    32    1.5       6          5       4.00      18       12.00      83333
 *          12    16    3.0       6          5       2.00      18        6.00     166667
 *          12     8    6.0       6          5       1.00      18        3.00     333333
 *          12     8    6.0      24         23       4.00      36        6.00     166667
 *          12     4   12.0       6          5       0.50      18        1.50     666667
 *          12     4   12.0      24         23       2.00      36        3.00     333333
 *          12     4   12.0      48         47       4.00      60        5.00     200000
 */
void IcosaLogic_Inverter_PWM::setupAdc() {
  // Serial.printf("setupAdc: enter\n");

  if (!useFeedback) {
    Serial.printf("    ADC not set up, no feedback configured\n");
    return;
  }
  
  setupAdcBitSize();
  if (numErrors > 0) {
    return;
  }
  setupAdcInitCfg();
  setupAdcSchedule();
  if (numErrors > 0) {
    return;
  }
  setupAdcEnable();  
}

/*
 * Set up and validate the bit size to be used with the ADC.
 */
void IcosaLogic_Inverter_PWM::setupAdcBitSize() {
  // Check the ADC bit size settings
  bool bitSizeValid = false;
  for (int i = 0; i < numAdcResultSizeEntries; i++) {
    AdcResultSizeEntry* p = &(adcResultSize[i]);
    if (p->numBits == inputParams.adcNumBits) {
      curAdcResultSizeEntry = p;
      bitSizeValid = true;
      break;
    }
  }
  
  if (! bitSizeValid) {
    setError(I20_ERR_INVALID_NUMADCBITS_VALUE);
    return;
  }
}

/*
 * Setup initial ADC configuration.
 */
void IcosaLogic_Inverter_PWM::setupAdcInitCfg() {
  // Serial.printf("setupAdcInitCfg: enter\n");

  // Validate prescale value
  bool prescaleValid = false;
  for (int i = 0; i < 8; i++) {
    if (inputParams.adcPrescaleVal == AdcClockPrescaleOptions[i]) {
      prescaleValid = true;
      AdcClockPrescale = inputParams.adcPrescaleVal;                   // Divide GCLK by this value
      AdcClockPrescaleCfg = i;
    }
  }
  if (!prescaleValid) {
    setError(I20_ERR_INVALID_ADC_PRESCALE_VALUE);
    return;
  }
  
  // Validate adcSampleTicks
  if (inputParams.adcSampleTicks == 0 || inputParams.adcSampleTicks > 64) {
    setError(I20_ERR_INVALID_ADC_SAMPLE_TICKS);
    return;
  }
    
  // Do initial ADC configuration
	for (int i = 0; i < ADC_INST_NUM; i++) {
    Adc* adc = adcs[i];
    
    adc->CTRLA.bit.SWRST = 1;                                // Reset the ADC first
    while (adc->SYNCBUSY.bit.SWRST);
    
		adc->CTRLA.bit.PRESCALER = AdcClockPrescaleCfg;          // unsynchronized field

		adc->CTRLB.bit.RESSEL = curAdcResultSizeEntry->cfgValue; // set ADC result bit size
		while (adc->SYNCBUSY.bit.CTRLB);                         // wait for sync

		adc->SAMPCTRL.reg = inputParams.adcSampleTicks - 1;      // sampling Time Length
		while (adc->SYNCBUSY.bit.SAMPCTRL);                      // wait for sync
    
    NVIC_SetPriority(adcIrqs[i], adcInterruptPriority);      // enable result ready interrupt
    NVIC_EnableIRQ(adcIrqs[i]);
    adc->INTENSET.bit.RESRDY = 1;
	}
  
	analogReference(inputParams.adcVRefNdx);                   // specify the AREF value
  adcVRef = adcReference[inputParams.adcVRefNdx].refVoltage; // save AREF value for calculations
}

/*
 * Enable the ADCs.
 */
void IcosaLogic_Inverter_PWM::setupAdcEnable() {
  // Enable the ADCs.  The first conversion in each schedule will be triggered by the TCC.
	for (int i = 0; i < ADC_INST_NUM; i++) {
    if (adcBusy[i]) {
      Adc* adc = adcs[i];
      while (adc->SYNCBUSY.bit.ENABLE);
      adc->CTRLA.bit.ENABLE = 1;
      while (adc->SYNCBUSY.bit.ENABLE);
      Serial.printf("  ADC %d enabling complete\n", i);
    } else {
      Serial.printf("  ADC %d no schedule, not enabled\n", i);
    }
  }
}

/*
 * Set up the different ADC schedules in the inverter.
 * TODO: If feedback is used, validate that doPwmHandler is set on at least one entry
 */
void IcosaLogic_Inverter_PWM::setupAdcSchedule() {
  // Serial.printf("setupAdcSchedule: enter\n");

  // generate target reference ADC voltage values, pre-adjusted by the resistor divider values
  // each line may have different resistor values!!??
  // Example1:  Vrms=120  Vpeak=169.7   Vrange=339.4   R1=820000  R2=7320  Vmax=372.97  vLsb=0.0910576
  // Example2:  Vrms=130  Vpeak=183.84  Vrange=367.68  R1=820000  R2=7320  Vmax=372.97  
  
  /*
  const double peakOutVolts = inputParams.outRmsVoltage * sqrt(2.0);
  const double rangeOutVolts = peakOutVolts * 2.0;
  const double resistorDivider = (double) feedback->vrBottom /
                                 (double)(feedback->vrTop + feedback->vrBottom);
  const double vMax = adcVRef / resistorDivider;
  const double vLsb = vMax / curAdcResultSizeEntry->numValues;
  
  const double peakOutVoltsDiv = peakOutVolts * resistorDivider;
  const double adcVoltage = peakOutVoltsDiv * (double)(curAdcResultSizeEntry->numValues) / adcVRef;
  // assert vMax > rangeOutVolts
  
  Serial.print("    Vrms=");
  Serial.print(inputParams.outRmsVoltage, 3);
  Serial.print("  Vpp=");
  Serial.print(peakOutVolts, 3);
  Serial.print("  rdiv=");
  Serial.print(resistorDivider, 5);
  Serial.print("  Vdiv=");
  Serial.print(dividedPeakVolts, 3);
  Serial.print("  Vp_adc=");
  Serial.println(adcVoltage, 3);
  */
    
  /*
  for (int i = 0; i < numSamples; i++) {
    double sineValue = sin((double) i * pi2 / dNumSamples);
    
    // generate PWM values for 100% output, it will be throttled down at runtime
    /*
    rawSin = sineValue * adcVoltage;
    if (rawSin > 0) {          // round up
      rawSin += 0.5;
    } else if (rawSin < 0) {   // round down
      rawSin -= 0.5;
    }
    adcSineData[i] = (int16_t)rawSin;
  }
  */
  
  // calculate time to perform one ADC conversion
  float adcFreq = Freq_48MHz / inputParams.adcPrescaleVal;
  float adcConversionSec = float(inputParams.adcSampleTicks + curAdcResultSizeEntry->numBits) / adcFreq;
  adcConversionNs = uint32_t(adcConversionSec * 1e9);
  float adcConversionCpuTicks = adcConversionNs / tccNsPerTick;
  Serial.printf("  ADC conversion time %d ns  cpuTicks ", adcConversionNs);
  Serial.println(adcConversionCpuTicks, 0);
  
  // Zero out the ADC scheduling fields
  memset(&adc0Schedule, 0, sizeof(AdcSchedule));
  memset(&adc1Schedule, 0, sizeof(AdcSchedule));
  
  adc0Schedule.adcNum = 0;
  adc0Schedule.adc    = adcs[0];
  adc1Schedule.adcNum = 1;
  adc1Schedule.adc    = adcs[1];
  
  Serial.printf("building ADC schedules\n");
  // Build the ADC schedules in pos order.  This also configures the I/O pins for ADC input.
  for (int pos = 0; pos < maxNumSchedEntries; pos++) {
    for (int i = 0; i < maxNumFbSignals; i++) {
      I20FeedbackSignal* fbs = feedback->signal[i];
      if (fbs != NULL && fbs->pos ==  pos) {
        setupAdcScheduleEntry(fbs);
      }
    }
  }
  Serial.printf("done building ADC schedules\n");
  
  if (adc0Schedule.doPwmDetected && adc1Schedule.doPwmDetected) {
    setError(I20_ERR_ADC_DO_PWM_ON_MULTIPLE_SCHEDULES);
  }
  if ((adc0Schedule.numEntries > 0 || adc1Schedule.numEntries > 0) &&
      !adc0Schedule.doPwmDetected && !adc1Schedule.doPwmDetected) {
    setError(I20_ERR_ADC_DO_PWM_NOT_SET);
  }
  
  // Verify that the ADC schedules will run within a single PWM cycle
  uint32_t pwmCycleNs = (uint32_t) (1e9 / (double) inputParams.pwmFreq);
  if (adc0Schedule.scheduleTimeNs > pwmCycleNs || adc1Schedule.scheduleTimeNs > pwmCycleNs) {
    setError(I20_ERR_ADC_SCHEDULE_TIME_LONGER_THAN_PWM_TIME);
  }

  // Display the ADC schedules, even if there were errors
  displayAdcSchedule(&adc0Schedule);
  displayAdcSchedule(&adc1Schedule);
}

/*
 * Add an entry to an ADC schedule, and setup ADC pins as inputs.
 * Normal entries run once every PWM cycle (if set up correctly!).
 * Sibling entries are set up to run once every Nth cycle for N siblings.
 */
void IcosaLogic_Inverter_PWM::setupAdcScheduleEntry(I20FeedbackSignal* fbs) {
  if (fbs == NULL) {
    return;
  }
  // Serial.printf("setupAdcScheduleEntry: adcPinPos %d\n", fbs->adcPinPos);
  
  // Get the low-level pin data for the referenced ADC pin
  I20PinData* pdPos = &(InverterAdcCfg[fbs->adcPinPos]);
  if (pdPos == NULL) {
    Serial.printf("setupAdcScheduleEntry: pdPos == NULL  adcPinPos %d\n", fbs->adcPinPos);
    setError(I20_ERR_INVALID_ADC_NUMBER);
    return;
  }
  Serial.printf("    adc %d pos %2d: input pin %2d: pad %s pioType 0x%x\n",
                pdPos->adcNum, fbs->pos, pdPos->pinNum, pdPos->pad, pdPos->peripheral);
  if (pdPos->adcNum < 0 || pdPos->adcNum >= ADC_INST_NUM) {
    setError(I20_ERR_INVALID_ADC_NUMBER);
    return;
  }

  // Do not allow getting multiple readings from the same pin
  // TODO: expand this to check board pins, too
  if (adcPinBusy[pdPos->pinNum]) {
    setError(I20_ERR_REUSED_ADC_INPUT_PIN);
    return;
  }
  adcPinBusy[pdPos->pinNum] = true;
  adcBusy[pdPos->adcNum] = true;
    
  // Set the pin for analog input
  // Serial.printf("    pos %2d: input pin %2d: pad %s pioType 0x%x\n",
  //               fbs->pos, pdPos->pinNum, pdPos->pad, pdPos->peripheral);
  pinPeripheral(pdPos->pinNum, PIO_ANALOG);
    
  // Get the schedule to update.  Build the schedule entry.
  AdcSchedule* sched = adcSchedules[pdPos->adcNum];
  AdcScheduleEntry* prevEntry = sched->curEntry;
  getNextEntry(sched);
  if (numErrors > 0) {
    return;
  }
    
  // Serial.printf("    Building AdcScheduleEntry\n");
  AdcScheduleEntry* entry = sched->curEntry;
  if (entry == NULL) {
    setError(I20_ERR_NULL_ADC_SCHEDULE_ENTRY);
    return;
  }
  entry->fbs          = fbs;
  entry->ipd          = pdPos;
  entry->adc          = adcs[pdPos->adcNum];
  entry->doPwmHandler = fbs->doPwmHandler;
  
  // Save the feedback pointer in the runtime PerLineData structure
  switch (fbs->fbType) {
    case I20_LINE1_CURRENT:
      lines[0].cfb  = fbs;
      break;
    case I20_LINE1_VOLTAGE:
      lines[0].vfb = fbs;
      break;
    case I20_LINE2_CURRENT:
      lines[1].cfb  = fbs;
      break;
    case I20_LINE2_VOLTAGE:
      lines[1].vfb = fbs;
      break;
    case I20_LINE3_CURRENT:
      lines[2].cfb  = fbs;
      break;
    case I20_LINE3_VOLTAGE:
      lines[2].vfb = fbs;
      break;
  }
  
  // Verify that doPwmHandler is specified in only one position in this schedule.
  // Note that it must be specified on all the siblings in a position.
  if (entry->doPwmHandler) {
    if (sched->doPwmDetected) {
      if (sched->doPwmPos != fbs->pos) {
        // don't return for this error
        setError(I20_ERR_ADC_DO_PWM_MULTIPLE_POS);
      }
    } else {
      sched->doPwmDetected = true;
      sched->doPwmPos = fbs->pos;
    }
  } 

  // Serial.printf("    Setting next and sibling links\n");
  // initialize next and sibling links
  entry->next = NULL;
  if (prevEntry == NULL) {
    // first entry in schedule, not a sibling
    entry->sibling = NULL;
    sched->scheduleTimeNs += adcConversionNs;
  } else if (prevEntry->fbs->pos != fbs->pos) {
    // not first entry in schedule, not yet a sibling
    entry->sibling = NULL;
    prevEntry->next = entry;
    sched->scheduleTimeNs += adcConversionNs;
    if (prevEntry->sibling != NULL) {
      // Fix all the next pointers in the previous list of siblings
      // Serial.printf("    fixing sibling next pointers\n");
      do {
        prevEntry->next = entry;
        prevEntry = prevEntry->sibling;
      } while (prevEntry->next == NULL);
    }
  } else {
    // sibling of the previous entry, build circular list of siblings
    if (prevEntry->sibling == NULL) {
      // prevEntry is 1st, entry is 2nd sibling
      prevEntry->sibling = entry;
      entry->sibling = prevEntry;
    } else {
      // nth sibling
      entry->sibling = prevEntry->sibling;
      prevEntry->sibling = entry;
    }
    if (entry->doPwmHandler != prevEntry->doPwmHandler) {
      // don't return because of this error, keep going
      setError(I20_ERR_ADC_SIBLING_DO_PWM_CONFLICT);
    }
  }
  setupAdcInputCtrl(entry);
  getDestAddrResult(entry);
}

/*
 * Return a pointer to the next available entry in the given AdcScheduleSlot, then
 * increment the number of entries.
 */
void IcosaLogic_Inverter_PWM::getNextEntry(AdcSchedule* sched) {
  // Serial.printf("getNextEntry: enter\n");
  
  if (sched->numEntries >= maxNumSchedEntries) {
    setError(I20_ERR_NUM_ADC_SCHEDULE_ENTRIES_MAX_EXCEEDED);
    return;
  }    
  AdcScheduleEntry* pEntry = &sched->entry[sched->numEntries];
  sched->numEntries += 1;
  sched->curEntry  = pEntry;
  sched->lastEntry = pEntry;
  if (sched->firstEntry == NULL) {
    sched->firstEntry = pEntry;
  }
}

/*
 * Validates an ADC schedule.  Most of the validation is done while it is being built
 */
 
/*
 * Assign the ADC inputctrl value to an AdcScheduleEntry.
 * We determine what the uint16_t value will be and save it as a raw uint16_t.
 * This method is perhaps overkill, as readings will probably all be single-ended.
 */
void IcosaLogic_Inverter_PWM::setupAdcInputCtrl(AdcScheduleEntry* entry) {
  // Serial.printf("setupAdcInputCtrl: enter\n");
  
  I20FeedbackSignal *fb = entry->fbs;
  I20PinData* pdPos = &(InverterAdcCfg[fb->adcPinPos]);
  
  ADC_INPUTCTRL_Type ic;
  ic.reg = 0;
  ic.bit.MUXPOS = pdPos->adcChannel;
   
  if (fb->adcPinNeg == I20_PIN_NONE) {
    setError(I20_ERR_INVALID_ADC_NEG_REF);
    return;
  } else if (fb->adcPinNeg == I20_PIN_GND) {
    // single ended ADC reading
    ic.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val;
  } else {
    // differential ADC reading
    I20PinData* pdNeg = &(InverterAdcCfg[fb->adcPinNeg]);
    if (pdPos->adcNum != pdNeg->adcNum) {
      setError(I20_ERR_POS_AND_NEG_ON_DIFFERENT_ADC);
      return;
    }
    // this may not be a valid check -- for differential mode, ADC negative is vBatt mid
    if (!adcPinBusy[pdNeg->pinNum]) {
      setError(I20_ERR_ADC_DIFF_MODE_NEG_PIN_VBAT_MID_CONFLICT);
      return;
    }
    ic.bit.MUXNEG = pdNeg->adcChannel;
    ic.bit.DIFFMODE = 1;
  }
  entry->inputCtrl = ic.reg;
}

/*
 * Start the schedule for the given ADC. 
 * Everything is preconfigured, just set the INPUTCTRL, and trigger the ADC to start.
 * If there is more than 1 position in the schedule, set the ADC to FREERUN, and after
 * the ADC is busy with the 1st reading, set INPUTCTRL for the second reading, and the
 * ADC will automatically start the next reading with no intervention.
 */
void IcosaLogic_Inverter_PWM::adcScheduleStart(AdcSchedule* sched) {
  logEntryStart(I20LogTypeAdcStart);
  logEntryVu8(sched->adcNum);
  
  Adc* adc = sched->adc;
  
  if (adc->STATUS.bit.ADCBUSY) {
    // previous schedule did not finish within PWM cycle, so don't start this time
    sched->overrunErrCount += 1;
    if (sched->overrunErrCount > maxOverrunErrCount) {
      setError(I20_ERR_ADC0_OVERRUN_MAX_EXCEEDED);
      LogEntry* thisEntry = curLogEntry;
      stop();
      thisEntry->stop = DWT->CYCCNT;
      return;
    }
  } else if (sched->firstEntry != NULL) {
    sched->isRunning = true;                             // start ADC schedule
    sched->curEntry = sched->firstEntry;
    AdcScheduleEntry* curEntry = sched->curEntry;
    
    adc->INPUTCTRL.reg = curEntry->inputCtrl;            // set the input
    logEntryVu16(curEntry->inputCtrl);
    while (adc->SYNCBUSY.bit.INPUTCTRL);
    
    if (curEntry->next != NULL) {
      while (adc->SYNCBUSY.bit.CTRLB);                   // set FREERUN
      adc->CTRLB.bit.FREERUN = 1;
      while (adc->SYNCBUSY.bit.CTRLB);
    }

    adc->SWTRIG.bit.START = 1;                           // Start conversion

    if (curEntry->sibling != NULL) {
      sched->firstEntry = curEntry->sibling;             // sibling is next first entry
    }
    
    while (adc->SWTRIG.bit.START);                       // wait for the ADC to start
    if (curEntry->next != NULL) {
      adc->INPUTCTRL.reg = curEntry->next->inputCtrl;    // prime for following entry
    }
  }

  logEntryStop();
}

/*
 * Continue the schedule for the given ADC. 
 * Copy out the result of the previous reading.
 * If there is an entry after the previous entry, the ADC is already processing it as
 * the current reading.
 * If there is another entry after the current one, set INPUTCTRL for the next entry
 * and the ADC will automatically update and start when the current entry is done.
 * If there is not another entry after the current one, turn off FREERUN, and the ADC
 * will stop after the current reading.
 * In all cases, the ADC generates a RESRDY interrupt when a reading is complete.
 */
void IcosaLogic_Inverter_PWM::adcScheduleNext(AdcSchedule* sched) {
  logEntryStart(I20LogTypeAdcNext);
  
  Adc* adc = sched->adc;
  AdcScheduleEntry* prevEntry = sched->curEntry;
  
  *(prevEntry->pResult) = adc->RESULT.reg;               // copy out the result
  logEntryVu16(adc->RESULT.reg);
  logEntryVu8(sched->adcNum);
  
  AdcScheduleEntry* curEntry = prevEntry->next;
  sched->curEntry = curEntry;                            // update current entry
  if (curEntry != NULL) {
    if (curEntry->next != NULL) {
      adc->INPUTCTRL.reg = curEntry->next->inputCtrl;      // if next entry, set input
    } else {
      while (adc->SYNCBUSY.bit.CTRLB);                     // if no next entry, turn off FREERUN mode
      adc->CTRLB.bit.FREERUN = 0;
      while (adc->SYNCBUSY.bit.CTRLB);
    }
    
    if (curEntry->sibling != NULL) {
      prevEntry->next = curEntry->sibling;                 // next time, run this entry's sibling
    }
  }

  LogEntry* thisEntry = curLogEntry;
  
  if (prevEntry->doPwmHandler) {                         // update pwm duty cycle
    pwmHandler();
  }
  
  // logEntryStop();
  thisEntry->stop = DWT->CYCCNT;
}

/*
 * Display an AdcSchedule.
 */
void IcosaLogic_Inverter_PWM::displayAdcSchedule(AdcSchedule* pSched) {
  Serial.printf("    ADC %d: numEntries %d  running: %d  schedTimeNs %d  resRdyErr %d  ",
                pSched->adcNum, pSched->numEntries, pSched->isRunning, pSched->scheduleTimeNs,
                pSched->resRdyErrCount);
  Serial.flush();
  Serial.printf("overrunErr %d  first: %8x  cur: %08x  last: %08x\n",
                pSched->overrunErrCount, pSched->firstEntry, pSched->curEntry, pSched->lastEntry);
  Serial.flush();
  for (int i = 0; i < pSched->numEntries; i++) {
    displayAdcScheduleEntry(&pSched->entry[i], i);
  }
}

/*
 * Display an AdcScheduleEntry.
 */
void IcosaLogic_Inverter_PWM::displayAdcScheduleEntry(AdcScheduleEntry* pEntry, int i) {
  Serial.printf("      %d: @ %08x: next %08x  sibling %08x  pin %d/%d  ",
                i, pEntry, pEntry->next, pEntry->sibling, pEntry->ipd->pinNum, pEntry->fbs->adcPinPos);
  Serial.flush();
  int fbType = pEntry->fbs->fbType;
  int lineNum = fbTypeToLineNum[fbType];
  Serial.printf("         type %d [%s]  line %d  inputctl %x  doPwm %d\n",
                fbType, feedbackTypeNames[fbType], lineNum, pEntry->inputCtrl, pEntry->doPwmHandler);
}

/*
 * Return a pointer to the result destination of the current ADC schedule entry.
 */
void IcosaLogic_Inverter_PWM::getDestAddrResult(AdcScheduleEntry* entry) {
  // Serial.printf("getDestAddrResult: enter\n");
  
  uint16_t* pResult = NULL;
  switch (entry->fbs->fbType) {
    case I20_LINE1_CURRENT:
      pResult = &(lines[0].adcAmps1);      // was lines[entry->lineNum]
      break;
    case I20_LINE1_VOLTAGE:
      pResult = &(lines[0].adcVolts1);
      break;
    case I20_LINE2_CURRENT:
      pResult = &(lines[1].adcVolts1);
      break;
    case I20_LINE2_VOLTAGE:
      pResult = &(lines[1].adcVolts1);
      break;
    case I20_LINE3_CURRENT:
      pResult = &(lines[2].adcVolts1);
      break;
    case I20_LINE3_VOLTAGE:
      pResult = &(lines[2].adcVolts1);
      break;
    case I20_BATTTOP_VOLTAGE:
      pResult = &vBattTopRaw;
      break;
    case I20_BATTMID_VOLTAGE:
      pResult = &vBattMidRaw;
      break;
    default:
      // ignore other values
      break;
  }
  entry->pResult = pResult;
}

/*
 * Handler for TCCx_0_IRQn.  This handler is used for setting the TCC CC values (duty cycle)
 * for the next PWM cycle.
 */
void IcosaLogic_Inverter_PWM::tccxHandler(uint8_t tccNum) {
  logEntryStart(I20LogTypeTcc);
  logEntryVu8(tccNum);
  // because nested handlers allocate multiple log entries, we have to manage this manually
  LogEntry* thisEntry = curLogEntry;
  tccIrq0NumTotal += 1;
  
  // This interrupt is overloaded and is generated due to multiple conditions.
  // Handle only the OVF condition.
  Tcc* pwmTcc = TCC0;
  tccIrq0Flags = pwmTcc->INTFLAG.reg;
  if (pwmTcc->INTFLAG.bit.OVF) {
    pwmTcc->INTFLAG.bit.OVF = 1;  // clear the overflow flag
  } else {
    // this method only services overflow interrupts
    logEntryStop();
    return;
  }
  tccIrq0NumSvc += 1;
  pwmTcc->CTRLBCLR.bit.LUPD = 1;
  
  // Overhead up to here, now implement the semantics of the interrupt
  if (!useFeedback) {
    // call pwmHandler directly
    pwmHandler();
  } else {
    // start schedules at the beginning, pwmHandler will be called from an adcHandler
    if (adc0Schedule.numEntries > 0) {
      adcScheduleStart(&adc0Schedule);
    }
    if (adc1Schedule.numEntries > 0) {
      adcScheduleStart(&adc1Schedule);
    }
  }
    
  // logEntryStop();
  thisEntry->stop = DWT->CYCCNT;
}

/*
 * Handler for ADC0_1_IRQn.  This handler reads the ADC result and copies it to
 * the configured target address, then optionally calls the pwmHandler, and/or
 * initiates the next ADC schedule slot.
 */
void IcosaLogic_Inverter_PWM::adc0Handler() {
  logEntryStart(I20LogTypeAdcHandler);
  logEntryVu8(0);
  logEntryVu16(adc0Schedule.curEntry->inputCtrl);
  LogEntry* thisEntry = curLogEntry;                 // save cur log entry
  adcScheduleNext(&adc0Schedule);
  thisEntry->stop = DWT->CYCCNT;
}

/*
 * Handler for ADC1_1_IRQn.  This handler reads the ADC result and copies it to
 * the configured target address, then optionally calls the pwmHandler, and/or
 * initiates the next ADC schedule slot.
 */
void IcosaLogic_Inverter_PWM::adc1Handler() {
  logEntryStart(I20LogTypeAdcHandler);
  logEntryVu8(1);
  logEntryVu16(adc1Schedule.curEntry->inputCtrl);
  LogEntry* thisEntry = curLogEntry;                 // save cur log entry
  adcScheduleNext(&adc1Schedule);
  thisEntry->stop = DWT->CYCCNT;
}

/*
 * Returns true if the ADC results are available.
 * 
 * Since we always schedule current readings before voltage readings, we only check here if
 * the voltage readings are available.
 */
inline bool IcosaLogic_Inverter_PWM::adcResultsAvailable() {
  return lines[0].adcVolts1 != 0 &&
         (iNumLines < 2 || lines[1].adcVolts1 != 0) &&
         (iNumLines < 3 || lines[2].adcVolts1 != 0);
}

/*
 * Inline method for adjusting CC values every PWM period.
 * Range of CC values is tccPeriod, which is scaled down by (throttle / maxThrottle).
 * We assume the ADC readings are complete before this handler is called.
 * 
 * First order throttle adjustment is based on battery voltage.  Assuming 144 Li ion cells:
 * Max SoC: throttle=562   Mid Soc: throttle=673   Min SoC: throttle=842
 * 
 * Second order throttle adjustment is based voltage/current feedback.
 * 
 * Current and voltage readings rely on 3 values:
 * amps2  / volts2  -- readings for previous cycle
 * amps1p / volts1p -- predictions for current cycle
 * amps1  / volts1  -- readings for current cycle
 * amps0  / volts0  -- predicted readings for next cycle
 */
inline void IcosaLogic_Inverter_PWM::pwmHandler() {
  logEntryStart(I20LogTypePwm);
  
  uint8_t tccChannel = 0;
  for (int i = 0; i < iNumLines; i++) {
    PerLineData *pld   = &lines[i];
    Tcc* tcc      = pld->pwmTcc;
    uint8_t chNum = pld->chNum;
    
    /*
    // Does adcSineData contain current readings or voltage readings?
    // Skip if adcSineData (used as divisor below) is zero
    if (adcSineData[pld->altNdx] != 0) {
      // Calculate the linearized dimensionless load in the current PWM cycle
      int16_t tmpLoad = (pld->adcAmps1 - curAdcResultSizeEntry->midPoint) * defaultMaxLoad / adcSineData[pld->sineNdx];  // altNdx???
      if (tmpLoad < 0) {
        tmpLoad = 0 - tmpLoad;
      }
      pld->load = (uint16_t) tmpLoad;
      logUint16(&curLogEntry->pld[i].load, tmpLoad);
    }
    */
    
    // target the next PWM cycle so we can set the upcoming duty cycle
    pld->sineNdx += 1;
    if (pld->sineNdx >= numSamples) {
      pld->sineNdx = 0;
      if (i == 0) {
        numWaves += 1;
      }
    }
    // logUint16(&curLogEntry->pld[i].sineNdx, pld->sineNdx);
    pld->altNdx += 1;
    if (pld->altNdx >= numSamplesDiv2) {
      pld->altNdx = 0;
    }
    // logUint16(&curLogEntry->pld[i].altNdx, pld->altNdx);
    
    /*
    // calculate next voltage based on last 2 readings
    // vDelta < 0 if predicted voltage is too low, and > 0 if prediction is too high
    // range of duty cycle: 0..period
    // range of adcVolts0 is 0..4095 (ADC range), or -2048..2047
    // update the throttle based on the current load and predicted voltage
    pld->adcVolts0 = pld->adcVolts1 + pld->adcVolts1 - pld->adcVolts2;
    logUint16(&curLogEntry->pld[i].adcVolts0, pld->adcVolts0);
    logUint16(&curLogEntry->pld[i].adcVolts1, pld->adcVolts1);
    logUint16(&curLogEntry->pld[i].adcVolts2, pld->adcVolts2);
    
    int32_t vDelta = (int32_t) pld->adcVolts0 - curAdcResultSizeEntry->midPoint - adcSineData[pld->sineNdx];
    // pld->throttle *= (curAdcResultSizeEntry->midPoint + vDelta) / curAdcResultSizeEntry->midPoint;  // no adjustment for now
    logUint16(&curLogEntry->pld[i].throttle, pld->throttle);
    */
    
    // update the duty cycle based on the throttle
    int32_t throttledSineValue = (pwmSineData1[pld->sineNdx] * pld->throttle) / maxThrottle;
    if (inputParams.invArch == I20_HALF_BRIDGE) {
      // half bridge wave
      tcc->CCBUF[chNum    ].reg = throttledSineValue + perMidPoint;
    } else if (pld->sineNdx < numSamplesDiv2) {
      // first half of T-type wave
      tcc->CCBUF[chNum    ].reg = throttledSineValue;
      tcc->CCBUF[chNum + 1].reg = curFreqCfg->tccPeriod;
    } else {
      // second half of T-type wave
      tcc->CCBUF[chNum    ].reg =  0;
      tcc->CCBUF[chNum + 1].reg = throttledSineValue + curFreqCfg->tccPeriod;
    }
    
    // Set the value for the optional HWS output
    tcc = pld->hwsTcc;
    if (tcc != NULL) {
      chNum = pld->hwsChNum;
      uint32_t hwsValue = pld->sineNdx < numSamplesDiv2 ? curFreqCfg->tccPeriod : 0;
      tcc->CCBUF[chNum    ].reg = hwsValue;
    }
    
    // prep for next cycle
    pld->adcVolts2  = pld->adcVolts1;
    pld->adcVolts1p = pld->adcVolts0;
    pld->adcVolts1  = 0;
    pld->adcAmps2   = pld->adcAmps1;
    pld->adcAmps1p  = pld->adcAmps0;
    pld->adcAmps1   = 0;
  }
  
  logEntryStop();
}

/**************************************************************************/
/*!
    @brief  Start the inverter by setting the TCC CNTRLA ENABLE bit.
*/
/**************************************************************************/
void IcosaLogic_Inverter_PWM::start() {
  if (numErrors > 0) {
    Serial.printf("IcosaLogic_Inverter_PWM::start: not started due to previous errors\n");
    Serial.printf("IcosaLogic_Inverter_PWM::start: check getNumErrors() and getError()\n");
    return;
  }
  
  // For now, start TCC1 first, then start TCC0
  if (tccBusy[1]) {
    startTcc(1);
  }
  startTcc(0);
}

/**
 * Start the TCC with the given tccNum.
 */
void IcosaLogic_Inverter_PWM::startTcc(int tccNum) {
  Serial.printf("  startTcc: starting TCC%d...", tccNum);
  Serial.flush();

  const uint32_t maxSpin = 10000;

  // Enable tcc if no error during setup
  Tcc* pwmTcc = tccs[tccNum];
  for (uint32_t i = 0; pwmTcc->SYNCBUSY.bit.ENABLE && i < maxSpin; i++);
  if (pwmTcc->SYNCBUSY.bit.ENABLE) {
    Serial.printf("  TCC%d is already enabling\n", tccNum);
    return;
  }
  pwmTcc->CTRLA.bit.ENABLE = 1;
  for (uint32_t i = 0; pwmTcc->SYNCBUSY.bit.ENABLE && i < maxSpin; i++);
  if (pwmTcc->SYNCBUSY.bit.ENABLE) {
    Serial.printf("  TCC%d won't enable\n", tccNum);
  }
  
  Serial.printf("started\n", tccNum);
}

/**************************************************************************/
/*!
    @brief  Stop the inverter by clearing the TCC CNTRLA ENABLE bit.
*/
/**************************************************************************/
void IcosaLogic_Inverter_PWM::stop() {  
  stopTcc(0);
  if (tccBusy[1]) {
    stopTcc(1);
  }
  // Serial.printf("IcosaLogic_Inverter_PWM::stop: stopped\n");
}

void IcosaLogic_Inverter_PWM::stopTcc(int tccNum) {  
  // Disable tcc
  Tcc* pwmTcc = tccs[tccNum];
  while (pwmTcc->SYNCBUSY.bit.ENABLE);
  pwmTcc->CTRLA.bit.ENABLE = 0;
  while (pwmTcc->SYNCBUSY.bit.ENABLE);
  
  Serial.printf("  stopTcc:  stopped TCC%d\n", tccNum);
}

/**************************************************************************/
/*!
    @brief  Returns true if the inverter is currently running.
*/
/**************************************************************************/
bool IcosaLogic_Inverter_PWM::isRunning() {
  Tcc* pwmTcc = TCC0;
  return pwmTcc->CTRLA.bit.ENABLE;
}

/**************************************************************************/
/*!
    @brief  Get the number of complete sine waves generated.
*/
/**************************************************************************/
uint32_t IcosaLogic_Inverter_PWM::getNumWaves() {
  return numWaves;
}

/*
 * Dump the sine wave values to be used at the selected PWM frequency.
 */
void IcosaLogic_Inverter_PWM::dumpScaledSineData(bool atZero) {
  const int maxValuesPerLine = 16;
  int valuesPerLine = 0;
  int qSamples = 1; numSamples / 4;
  
  Serial.printf("PWM Sine Values: numSamples=%d  period=%d  midPoint=%d\n",
                numSamples, curFreqCfg->tccPeriod, perMidPoint);
  for (int i = 0; i < numSamples; i += qSamples) {
    if (valuesPerLine == 0) {
      Serial.printf("%5d: ", i);
    }
    Serial.printf("%7d, ", pwmSineData1[i]);
    valuesPerLine += 1;
    if (valuesPerLine >= maxValuesPerLine) {
      Serial.printf("\n");
      valuesPerLine = 0;
    }
  }
  Serial.printf("\n\n");
  
  /*
  Serial.printf("ADC Sine Values: numSamples=%d  adcBits=%d  \n",
                numSamples, inputParams.adcNumBits);
  valuesPerLine = 0;
  for (int i = 0; i < numSamples; i += qSamples) {
    if (valuesPerLine == 0) {
      Serial.printf("%5d: ", i);
    }
    int32_t value = adcSineData[i];
    Serial.printf("%7d, ", value);
    valuesPerLine += 1;
    if (valuesPerLine >= maxValuesPerLine) {
      Serial.printf("\n");
      valuesPerLine = 0;
    }
  }
  Serial.printf("\n");
  */
}

/*
 * Dump the saved log data.
 */
void IcosaLogic_Inverter_PWM::dumpLog(bool verbose) {
  Serial.printf("Log:  numEntries %d  entrySize %d\n", numLogEntries, sizeof(LogEntry));
  
  for (int i = 0; i < numLogEntries; i++) {
    dumpLogEntry(i, verbose);
  }
}

/*
 * Dump the undumped log data.  This function can be called repeatedly, and should
 * only dump recent entries since the last call.
 */
void IcosaLogic_Inverter_PWM::dumpLogTail(bool verbose) {
  while (dmpLogEntry != nxtLogEntry) {
    dumpLogEntry(dmpLogEntry - firstLogEntry, verbose);
    dmpLogEntry += 1;
    if (dmpLogEntry == wrapLogEntry) {
      dmpLogEntry = firstLogEntry;
    }
  }
}

/*
 * Dump the log entry at the given index.
 */
void IcosaLogic_Inverter_PWM::dumpLogEntry(int i, bool verbose) {
  LogEntry* p = &(eventLog[i]);
  LogEntry* q = p == firstLogEntry ? NULL : p - 1;
  if (!verbose && p->logType == 0) {
    return;
  }
  
  const char* typeName = p->logType <= I20LogTypeExtension ? logTypeNames[p->logType] : "???";
  uint32_t elapsedTicks = p->stop - p->start;
  
  Serial.printf("    %5d: %-6s  %11u %11u ",
                i, typeName, p->start, p->stop);
  Serial.flush();
  Serial.printf("(%5d)  vu8 %3u %02x  vu16 %5u %04x",
                elapsedTicks, p->vu8, p->vu8, p->vu16, p->vu16);
  
  if (q != NULL) {
    // print the elapsed ticks from the start of the previous log entry to the start of this one
    Serial.printf("  [delay %5u]", p->start - q->start);
  }
  Serial.println(" ");
}

/*
 * Dump the given number of log entries, starting at the current log entry and going backwards.
 * Does not roll backwards from the first entry to the last.
 */
void IcosaLogic_Inverter_PWM::dumpLogLast(int offset, int numEntries) {
  LogEntry* startEntry = curLogEntry - offset;
  if (startEntry < firstLogEntry) {
    startEntry = wrapLogEntry - 1;
  }
  LogEntry* endEntry = startEntry - numEntries;
  if (endEntry < firstLogEntry) {
    endEntry = wrapLogEntry - (firstLogEntry - endEntry);
  }
  while (startEntry != endEntry) {
    dumpLogEntry(startEntry - firstLogEntry, false);
    startEntry -= 1;
    if (startEntry < firstLogEntry) {
      startEntry = wrapLogEntry - 1;
    }
  }
}

/*
 * Validate the contents of the log data.
 * - Make sure all ADC and PWM handler calls finish before the next TCC entry
 * - Make sure all ADC handler calls match the configured schedule
 */
bool IcosaLogic_Inverter_PWM::validateLog() {
  Serial.printf("Validating Log:  numEntries %d  entrySize %d\n", numLogEntries, sizeof(LogEntry));
  
  if (isRunning()) {
    Serial.printf("    stop the inverter before validating the log\n");
    return false;
  }
  
  LogEntry* tccEntry  = NULL;
  LogEntry* adc0Entry = NULL;
  LogEntry* adc1Entry = NULL;
  LogEntry* curEntry  = NULL;
  
  // The log may have wrapped, so find the first TCC entry
  // TODO: start at entry after curLogEntry, analyze up to and including curLogEntry
  int i = 0;
  for ( ; i < numLogEntries; i++) {
    curEntry = &(eventLog[i]);
    if (curEntry->logType == I20LogTypeTcc) {
      tccEntry = curEntry;
      i += 1;
      break;
    }
  }
  
  if (tccEntry == NULL) {
    Serial.printf("    could not find a TCC log entry\n");
    return false;
  }
  
  bool valid = true;
  for ( ; i < numLogEntries; i++) {
    // TODO: implement me!
    curEntry = &(eventLog[i]);
    switch(curEntry->logType) {
      case I20LogTypeUnused:
        // ignore unused entries
        break;
      case I20LogTypeTcc:
        break;
      case I20LogTypeAdcHandler:
        break;
      case I20LogTypeAdcStart:
        break;
      case I20LogTypeAdcNext:
        break;
      case I20LogTypePwm:
        break;
      case I20LogTypeExtension:
        break;
      default:
        Serial.printf("    %5d: invalid logType %d [0x%x]\n", curEntry->logType, curEntry->logType);
        valid = false;
        break;
    }
  }
  
  return valid;
}

/*
 * Dump the PerLineData entry for each line.
 */
void IcosaLogic_Inverter_PWM::dumpPerLineData() {
  Serial.println(" ");
  for (uint8_t n = 0; n < iNumLines; n++) {
    dumpPerLineDataEntry(n, &lines[n]);
  }
}

/*
 * Dump a single PerLineData entry.
 */
void IcosaLogic_Inverter_PWM::dumpPerLineDataEntry(uint8_t i, PerLineData* pld) {
  Serial.printf("Line %d: pwmTcc=%08x pwmCh=%d hwsTcc=%08x hwsCh=%d ",
                i + 1, pld->pwmTcc, pld->chNum, pld->hwsTcc, pld->hwsChNum);
  Serial.flush();
  Serial.printf("Ndx=%5d Alt=%5d thr=%4d load=%d ",
                pld->sineNdx, pld->altNdx, pld->throttle, pld->load);
  Serial.flush();
  Serial.printf("v0=%d v1=%d v1p=%d v2=%d a0=%d a1=%d a1p=%d a2=%d\n",
                pld->adcVolts0, pld->adcVolts1, pld->adcVolts1p, pld->adcVolts2,
                pld->adcAmps0,  pld->adcAmps1,  pld->adcAmps1p,  pld->adcAmps2);
}

/*
 * Dump the interrupt request handler counts.
 */
void IcosaLogic_Inverter_PWM::dumpIrqCounts(bool reset) {
  Serial.printf("IRQ: total=%d  svcd=%d  intflags=0x%08x\n",
                tccIrq0NumTotal, tccIrq0NumSvc, tccIrq0Flags);
  if (reset) {
    tccIrq0NumTotal = 0;
    tccIrq0NumSvc   = 0;
    tccIrq0Flags    = 0;
  }
}

/*
 * Setup the error array.
 */
void IcosaLogic_Inverter_PWM::setupErrors() {
  for (int i = 0; i < maxNumErrors; i++) {
    errNums[i] = I20_ERR_NONE;
  }
  numErrors = 0;
}

/*
 * Saves the error message and increments the error count.
 */
void IcosaLogic_Inverter_PWM::setError(uint16_t errNum) {
  numErrors += 1;
  if (numErrors < maxNumErrors) {
    errNums[numErrors] = errNum;
  }
}

/*
 * Returns the number of errors encountered.
 */
unsigned int IcosaLogic_Inverter_PWM::getNumErrors() {
  return numErrors;
}

/*
 * Returns the text associated with the errIndex'th error encountered.
 * No error is stored in errNums[0]
 */
const char* IcosaLogic_Inverter_PWM::getErrorText(int errIndex) {
  const char* result = NULL;
  if (errIndex > 0 && errIndex <= numErrors) {
    uint16_t errNum = errNums[errIndex];
    result = errText[errNum];
    if (result == NULL) {
      Serial.printf("**** errIndex %u  errNum %u\n", errIndex, errNum);
    }
  }
  return result;
}

/*
 * Returns the errNum associated with the errIndex'th error encountered.
 * No error is stored in errNums[0]
 */
uint16_t IcosaLogic_Inverter_PWM::getErrorNum(int errIndex) {
  uint16_t result = I20_ERR_NONE;
  if (errIndex > 0 && errIndex <= numErrors) {
    result = errNums[errIndex];
  }
  return result;
}

/*
 * Checks for errors in the inverter.  If the error count is > 0, prints the
 * error messages.  Returns the error count.
 */
void IcosaLogic_Inverter_PWM::printErrors() {
  Serial.printf("Errors detected: %d\n", numErrors);
  bool nullDetected = false;
  for (int i = 1; i <= numErrors; i++) {
    const char* errMsg = getErrorText(i);
    Serial.printf("  %3d [%u] %s\n", i, getErrorNum(i), errMsg == NULL ? "<NULL>" : errMsg);
    if (errMsg == NULL) {
      nullDetected = true;
    }
  }
  
  if (nullDetected) {
    for (int i = 0; i < (int) I20_ERR_LAST_ERROR_NUMBER; i++) {
      Serial.printf("    %d: %s\n", i, errText[i]);
    }
  }
}
