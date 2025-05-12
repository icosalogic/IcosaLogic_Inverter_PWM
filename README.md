# IcosaLogic_Inverter_PWM
This is an Arduino library containing logic for generating PWM signals for an inverter and using
feedback signals such as voltage and current readings to accurately control those PWM sigals.

The user describes the inverter configuration in an input parameters structure, and the library
manages operation of the inverter with no further interaction from the application.
The input parameters tell the library how the PWM signals should be generated,
and how feedback signals like voltage and current readings should be performed.
The library does the rest.

This library is targeted specifically for the SAMD51 processor architecture.
Much of the implementation details will be incomprehensible to users who are not familiar with
the internals of SAMD51 peripherals.
This library will run on many boards with SAMD51 processors, but smaller boards, or boards
using chips with a fewer number of pins may have configuration restrictions.

# Features
- 50 or 60 Hz output frequency
- Configurable PWM frequency
- Configurable dead time insertion
- 1, 2, or 3 output lines
- Supports half-bridge or T-type implementations
- Detailed configuration of ADC operations to balance speed and precision
- Flexible scheduling of ADC readings, using arbitrary combinations of the following:
    - Run readings in parallel on the 2 ADCs
    - Run readings in sequence on a single ADC
    - Run readings in round robin order on a single ADC

# Status
This implementation is incomplete.
- PWM signal generation is complete and seems robust
- ADC scheduling is mostly complete, with the exception of an ongoing investigation into a timing issue handling interrupts for parallel readings
- The framework for control is complete, but the last part to glue everything together is missing
- The entire library could use a ground-up refactoring
- The documentation is incomplete

# Platforms and Example Applications
Because this library is so intimately tied with the hardware details of both the SAMD51 processor
on which it runs and the board on which the processor is mounted, each board may require its own
example application.

This library has been tested on the following boards:
- Adafruit ItsyBitsy M4 Express
- Adafruit Feather M4 Express
- Adafruit Grand Central M4 Express

Boards to be tested in the future, which will probably result in additional board-specific examples:
- Sparkfun Thing Plus - SAMD51
- Adafruit Metro M4

The unit_test example is not very useful as a model on which to base an inverter implementation.
Much of it just involves calling `begin()` with bad parameters and making sure it detects an error.
A couple of the test methods may be useful as examples to developers who need to run the inverter
for a small number of sine waves, then stop it, to make it easier to debug and diagnose interrupt
behavior or other software issues.
Those methods show how to do a short controlled run which gives less output data to examine.

# Structure of this Document
The rest of this document is divided into two sections:
- The Quick Start section provides a basic overview of how to configure, start, and stop the inverter
- The Deep Dive section gives more information of the target environment and the rationale and details for the design and implementation of the library

# Quick Start

## Example Inverter Application

Following is the complete program to run an inverter with 1 output line at 60Hz, using a switching
frequency of 6kHz, and with no voltage or current feedback.
```C
/*
 * Minimal inverter for the IcosaLogic inverter library.
 */

#include <Arduino.h>
#include <IcosaLogic_Inverter_PWM.h>

// T-Type inverter, 1 output line at 60Hz, 6kHz PWM, 100ns dead time, no feedback
I20InputParams inParams =
    {I20_T_TYPE, I20_HWS_NONE, 120, 1, 60, 6000, I20_PS_TCC1, I20_PS_TCC0, 100, NULL};

IcosaLogic_Inverter_PWM inverter;

void setup() {
  Serial.begin(115200);
  delay(3000);
  
  Serial.println(" ");
  Serial.println("\n\nIcosaLogic_Inverter_PWM Test Inverter App\n\n");

  inverter.begin(&inParams);
  if (inverter.getNumErrors() > 0) {
    Serial.printf("inverter not started due to errors\n");
    inverter.printErrors();
  } else {
    inverter.start();
    Serial.printf("inverter running....\n");
  }

  Serial.println("done with setup");
}

void loop() {
}

// Handler for TCC0 OVF interrupts.
void TCC0_0_Handler() {
  inverter.tccxHandler(0);
};

// Handler for TCC1 OVF interrupts.
void TCC1_0_Handler() {
  inverter.tccxHandler(1);
};
```

## Inverter Object Definition

Here is the basic interface to the library:
```C++
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
}
```

## Configuration

### Input Parameters

The first objective for a user is to specify the input parameters.
```C++
typedef struct {
  I20InvArch         invArch;                   // Inverter architecture
  I20HalfWaveSignal  hws;                       // Half wave signal type
  uint16_t           outRmsVoltage;             // target RMS voltage of output lines
  uint8_t            numLines;                  // number of output lines
  uint8_t            outFreq;                   // output frequency, either 50 or 60 Hz
  uint32_t           pwmFreq;                   // PWM frequency
  I20TccCfgNdx       tccCfgNdx1;                // Preferred config to use, either TCC0 or 1
  I20TccCfgNdx       tccCfgNdx2;                // Second config to use, either TCC1 or 0
  uint16_t           deadTimeNs;                // dead time between MOSFET transitions
  I20Feedback*       feedback;                  // voltage and current feedback
} I20InputParams;
```

Each of these parameters is described below.

- `invArch` Inverter Architecture
    - `I20_HALF_BRIDGE` Generate PWM signals for a half bridge configuration with 2 MOSFETs per line
    - `I20_T_TYPE` Generate PWM signals for a T-Type inverter with 4 MOSFETs per line
- `hws` Half Wave Signal is an optional output signal that is high for half of the output sine wave
    - `I20_HWS_NONE` Don't generate a half-wave signal
    - `I20_HWS_SINGLE` Generate a single half wave signal
    - `I20_HWS_PAIR` Generate a pair of opposite half-wave signals, with dead time inserted between transitions
- `outRmsVoltage` Target RMS voltage of output<br>
    - This integer number is meaningful only if feedback is used
- `numLines` The number of output lines, which determines the number of PWM signals to generate<br>
    - An integer value from 1 to 3, inclusive
- `outFreq` The frequency of the output lines
    - Common Values are 50 and 60, and produce reasonable output.  Other values may not work as expected
- `pwmFreq` The frequency of the PWM signals
    - Integer value ranges from hundreds of hertz to over 100kHz.  The maximum value is a hardware-specific
function of your exact processor, clock configuration, etc.
- `tccCfgNdx1` Primary TCC configuration for the current platform, see `platform.h` for pin assignments
- `tccCfgNdx2` Secondary TCC configuration
    - `I20_PS_NONE` No configuration available
    - `I20_PS_TCC0` Use TCC0 primary configuration
    - `I20_PS_TCC0_ALT` Use TCC0 alternate configuration
    - `I20_PS_TCC1` Use TCC1 primary configuration
    - `I20_PS_TCC1_ALT` Use TCC1 alternate configuration
- `deadTimeNs` Dead time to insert into PWM signal transitions, in nanoseconds
    - Integer value from 0 (no dead time) to a maximum of `255 * ns_per_TCC_clock_tick`
- `feedback` ADC configuration and schedule
    - Pointer to `I20Feedback` structure, may be NULL, see next section for details

### Feedback

This section describes the generic configuration of the ADCs, and how ADC readings are scheduled.
Like for TCCs described above, pin assignments are specified by definitions in an include file
referenced by `platform.h`.
```C++
typedef struct {
  uint8_t            adcNumBits;                // ADC number of bits: 12, 10, or 8
  uint16_t           adcPrescaleVal;            // ADC clock prescale value
  uint16_t           adcSampleTicks;            // ADC clock ticks to hold sample
  eAnalogReference   adcVRefNdx;                // ADC reference
  I20FeedbackSignal* signal[maxNumFbSignals];   // voltage and current feedback
} I20Feedback;
```

- `adcNumBits` The number of bits in an ADC result.  Larger values are more precise, smaller values take less time
    - Valid values are 8, 10, and 12.
- `adcPrescaleVal` The source clock ticks are divided by this value to get the ADC clock stream
    - Valid values are 2, 4, 8, 16, 32, 64, 128, 256
- `adcSampleTicks` Sample hold time measured in ADC clock ticks
    - Valid values are from 1 to 64, inclusive
- `adcVRefNdx` Value to specify the ADC voltage reference against which input signals are measured
    - `AR_DEFAULT` Default is the 3.3V supply
    - `AR_INTERNAL1V0` Internal 1.0V reference
    - `AR_INTERNAL1V1` Internal 1.1V reference
    - `AR_INTERNAL1V2` Internal 1.2V reference
    - `AR_INTERNAL1V25` Internal 1.25V reference
    - `AR_INTERNAL2V0` Internal 2.0V reference
    - `AR_INTERNAL2V2` Internal 2.2V reference
    - `AR_INTERNAL2V23` Internal 2.23V reference
    - `AR_INTERNAL2V4` Internal 2.4V reference
    - `AR_INTERNAL2V5` Internal 2.5V reference
    - `AR_INTERNAL1V65` Internal 1.65V reference
    - `AR_EXTERNAL` Reference voltage from an ADC input pin
- `signal` A pointer to an array of feedback signals defining the ADC reading schedule
    - See the next section for details

### Feedback Signals

This structure describes an individual ADC reading, and its position in the schedule.
```C++
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
```

- `adcPinPos` The pin to use for the positive value, see platform.h
- `adcPinNeg` The pin to use for the negative value, differential reading if not GND
    - May be GND or a defined name like I20_PIN_Ax_ADCy that is defined in platform.h; note it also defines which ADC is used
- `pos` Position of this reading in the ADC schedule
    - 0-relative position in the schedule, duplicate positions are read in round-robin order in subsequent runs
- `doPwmHandler` The PWM state is updated after completion of the reading with this flag set to true; exactly one
reading in a schedule should have this flag set to true, it does not have to be set on the last reading in a schedule
    - Valid values are `true` and `false`
- `fbType` Specifies type of reading and the line to which it applies
    - `I20_FB_UNDEFINED` None of the values below
    - `I20_LINE1_CURRENT` Current of line 1
    - `I20_LINE1_VOLTAGE` Voltage of line 1
    - `I20_LINE2_CURRENT` Current of line 2
    - `I20_LINE2_VOLTAGE` Voltage of line 2
    - `I20_LINE3_CURRENT` Current of line 3
    - `I20_LINE3_VOLTAGE` Voltage of line 3
    - `I20_BATTTOP_VOLTAGE` Voltage of the battery top
    - `I20_BATTMID_VOLTAGE` Voltage of the battery midpoint
- `vrTop` Gives the top resistance value in a resistor divider, used to convert raw ADC value to real-world voltage
- `vrBottom` Gives the bottom resistance value in a resistor divider
    - Any valid 32-bit unsigned integer value greater than 0, may be 0 for non-voltage readings
- `aLsb` Amps per least significant bit, multiply raw ADC value by this to get a real-world current in amps
    - Any valid floating point number, may be 0 for non-current readings

## Interrupt Handlers

Interrupt handlers for the TCC and ADC peripherals are declared in the calling application.
While this may be viewed as an intrusion of minutia into the world of beginning users, it
does offer significant advantages to advanced users.

One does not need to define interrupt handlers for interrupts that will never be generated in a given application.
Specifically, applications that do not use feedback do not need to declare the ADC interrupt handlers.
On the other hand, there is no penalty for declaring an interrupt handler that will not be used,
and the application will not behave as intended if you do not declare an interrupt handler that is required.

The interrupt handlers given in the section below are adequate for most applications.
Additional features are available for advanced users; see the unit_test example or the Deep Dive section below.

```C
// Handler for TCC0 OVF interrupts.
void TCC0_0_Handler() {
  inverter.tccxHandler(0);
};

// Handler for TCC1 OVF interrupts.
void TCC1_0_Handler() {
  inverter.tccxHandler(1);
};

// Handler for ADC0 RESRDY interrupt.
void ADC0_1_Handler() {
  inverter.adc0Handler();
}

// Handler for ADC1 RESRDY interrupt.
void ADC1_1_Handler() {
  inverter.adc1Handler();
}
```

# Deep Dive
Coming soon to a README near you.
