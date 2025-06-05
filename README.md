# IcosaLogic_Inverter_PWM
This is an Arduino library containing logic for generating PWM signals for an inverter and using
feedback signals such as voltage and current readings to accurately control those PWM sigals.

The user describes the inverter configuration in an input parameters structure, and the library
manages operation of the inverter with no further interaction from the application.
The input parameters tell the library how the PWM signals should be generated,
and how feedback signals like voltage and current readings should be interpreted.
The library does the rest.

This library is targeted specifically for the SAMD51 processor architecture.
Much of the implementation details will be incomprehensible to users who are not familiar with
the internals of SAMD51 peripherals.
This library will run on many boards with SAMD51 processors, but smaller boards, or boards
using chips with a fewer number of pins may have configuration restrictions.

This library assumes it has total control over the TCCs and ADCs on the processor.
That means, for example, if you configure the inverter for feedback, start it, then do an
`analog_read()`, Arduino will reprogram one of the ADCs in a way that will probably break the inverter.
It is best to configure this library's feedback mechanism to do all your analog reads.
Calling `analog_write()` is also potentially a conflict.

# Features
- 50 or 60 Hz output frequency
- Configurable PWM frequency
- Configurable dead time insertion
- 1, 2, or 3 output lines
- Supports half-bridge or T-type inverter implementations
- Supports half-bridge DC configuration
- Detailed configuration of ADC operations to balance speed and precision
- Flexible scheduling of ADC readings, using arbitrary combinations of the following:
    - Run readings in parallel on the 2 ADCs
    - Run readings in sequence on a single ADC
    - Run readings in round robin order on a single ADC

# Status
This implementation is incomplete.
- PWM signal generation is complete and seems robust
- ADC scheduling is mostly complete, with the exception of an ongoing investigation into a timing
issue handling interrupts for parallel readings.  It works, but timing may be sub-optimal
- The framework for control is complete, with a PD controller implemented.  Testing on real hardware is in progress
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
    {I20_T_TYPE, 120, 1, 60, 6000, I20_PS_TCC1, I20_PS_TCC0, 100, NULL};

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
  uint16_t           outVoltage;                // target voltage of output lines (RMS for inverters)
  uint16_t           outCurrent;                // output current
  uint8_t            numLines;                  // number of output lines
  uint8_t            outFreq;                   // output frequency, either 50 or 60 Hz
  uint32_t           pwmFreq;                   // PWM frequency
  I20TccCfgNdx       tccCfgNdx1;                // Preferred config to use, either TCC0 or 1
  I20TccCfgNdx       tccCfgNdx2;                // Second config to use, either TCC1 or 0
  uint16_t           deadTimeNs;                // dead time between MOSFET transitions
  I20Feedback*       feedback;                  // voltage and current feedback
} I20InputParams;
```

Each parameter is described below.

- `invArch` Inverter Architecture
    - `I20_HALF_BRIDGE` Generate PWM signals for a half bridge configuration with 2 MOSFETs per line
    - `I20_T_TYPE` Generate PWM signals for a T-Type inverter with 4 MOSFETs per line
    - `I20_DC` Generate PWM signals for DC output using a half bridge configuration with 2 MOSFETs per line
- `outVoltage` Target RMS voltage of output<br>
    - This integer number is meaningful only if feedback is used
- `outCurrent` Target current of output<br>
    - This integer number is meaningful only if feedback is used, primarily for current-limited DC output
- `numLines` The number of output lines, which determines the number of PWM signals to generate<br>
    - An integer value from 1 to 3, inclusive
- `outFreq` The frequency of the output lines
    - Common Values are 50 and 60, and produce reasonable output.  Other values may not work as expected
- `pwmFreq` The frequency of the PWM signals
    - Integer value ranges from hundreds of hertz to 180kHz.  The effective maximum value is a
hardware-specific function of your exact processor, clock configuration, etc.
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
  float              extVRefValue;              // voltage of external VRef, if selected
  I20FeedbackSignal* signal[maxNumFbSignals];   // voltage and current feedback
} I20Feedback;
```

- `adcNumBits` The number of bits in an ADC result.  Larger values are more precise, smaller values take less time
    - Valid values are 8, 10, and 12.
- `adcPrescaleVal` The 48 MHz source clock is divided by this value to get the ADC clock
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
    - `AR_EXTERNAL` Reference voltage from the AREF input pin.  Using this means the DAC does not work properly
- `extVRefValue` Voltage of the external voltage reference
    - Floating point value of the reference voltage, used only when adcVRefNdx is AR_EXTERNAL
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

## Pin Assignments

Pin assignments are constrained by circuit layout internal to the die inside the processor chip,
and how the pads on the die are connected to the pins or balls on the chip package.
Further constraints may be imposed by how the chip package leads are connected to the circuit board.
This results in each board having a specific pin layout suitable for this inverter library.

The primary constraint on pin assignments for PWM generation are the result of TCC IOSET configurations.
A given TCC may support multiple IOSETs, but can only use one IOSET at a time.
A TCC supports pairs of pins; half-bridge configurations use 1 pair of pins per line, while T-type
configurations require 2 pairs per line.

For example, say you were implementing a half-bridge inverter with 3 lines using an ItsyBitsy M4 Express.
This would require 3 pairs of MOSFETs.
The library would assign the following pairs of pins:

| Line | MOSFET | Pin |
| ---- | ------ | --- |
|  L1  |   Q1   |   0 |
|  L1  |   Q2   |  10 |
|  L2  |   Q1   |   1 |
|  L2  |   Q2   |  11 |
|  L3  |   Q1   |   7 |
|  L3  |   Q2   |  13 |

A secondary constraint on pin assignments is how the Arduino analog input pins are declared for a given board.
One common issue is that some boards declare most of the analog input pins to be on ADC0, with very few on ADC1.

### Adafruit ItsyBitsy M4 Express

The only usable TCC IOSET for PWM on this board is TCC1 IOSET1.

| Pin | Pad  | Pair |
| --- | ---- | ---- |
|  0  | PA16 |   1  |
|  1  | PA17 |   2  |
|  7  | PA18 |   3  |
|  9  | PA19 |   4  |
| 10  | PA20 |   1  |
| 11  | PA21 |   2  |
| 13  | PA22 |   3  |
| 12  | PA23 |   4  |

Here are the available analog input pins.

| APin | Pin | Pad  | ADC0 | ADC1 |
| ---- | --- | ---- | ---- | ---- |
|  A0  |  14 | PA02 | Yes  |  No  |
|  A1  |  15 | PA05 | Yes  |  No  |
|  A2  |  16 | PB08 | Yes  |  Yes |
|  A3  |  17 | PB09 | Yes  |  Yes |
|  A4  |  18 | PA04 | Yes  |  No  |
|  A5  |  19 | PA06 | Yes  |  No  |

### Adafruit Feather M4 Express

The only usable TCC IOSET for PWM on this board is TCC1 IOSET1.

| Pin | Pad  | Pair |
| --- | ---- | ---- |
|  5  | PA16 |   1  |
| 25  | PA17 |   2  |
|  6  | PA18 |   3  |
|  9  | PA19 |   4  |
| 10  | PA20 |   1  |
| 11  | PA21 |   2  |
| 12  | PA22 |   3  |
| 13  | PA23 |   4  |

Here are the available analog input pins.

| APin | Pin | Pad  | ADC0 | ADC1 |
| ---- | --- | ---- | ---- | ---- |
|  A0  |  14 | PA02 | Yes  |  No  |
|  A1  |  15 | PA05 | Yes  |  No  |
|  A2  |  16 | PB08 | Yes  |  Yes |
|  A3  |  17 | PB09 | Yes  |  Yes |
|  A4  |  18 | PA04 | Yes  |  No  |
|  A5  |  19 | PA06 | Yes  |  No  |

### Adafruit Grand Central M4 Express

This is an exceptionally capable board, with a large number of available pins.
It is suitable for a maximum configuration inverter.

The primary TCC configuration on this board is TCC1 IOSET1.

| Pin | Pad  | Pair |
| --- | ---- | ---- |
| 37  | PA16 |   1  |
| 36  | PA17 |   2  |
| 35  | PA18 |   3  |
| 34  | PA19 |   4  |
| 33  | PA20 |   1  |
| 32  | PA21 |   2  |
| 31  | PA22 |   3  |
| 30  | PA23 |   4  |

A usable secondary is TCC0 IOSET2.

| Pin | Pad  | Pair |
| --- | ---- | ---- |
| 48  | PC04 |   1  |
| 51  | PD08 |   2  |
| 52  | PD09 |   3  |
| 53  | PD10 |   4  |
| 50  | PD11 |   1  |
| 22  | PD12 |   2  |
| 16  | PC22 |   3  |
| 17  | PC23 |   4  |

Here are the available analog input pins.

| APin | Pin | Pad  | ADC0 | ADC1 |
| ---- | --- | ---- | ---- | ---- |
|  A0  |  67 | PA02 | Yes  |  No  |
|  A1  |  68 | PA05 | Yes  |  No  |
|  A2  |  69 | PB03 | Yes  |  No  |
|  A3  |  70 | PC00 | No   | Yes  |
|  A4  |  71 | PC01 | No   | Yes  |
|  A5  |  72 | PC02 | No   | Yes  |
|  A6  |  73 | PC03 | No   | Yes  |
|  A7  |  74 | PB04 | No   | Yes  |
|  A8  |  54 | PB05 | No   | Yes  |
|  A9  |  55 | PB06 | No   | Yes  |
|  A10 |  56 | PB07 | No   | Yes  |
|  A11 |  57 | PB08 | Yes  | Yes  |
|  A12 |  58 | PB09 | Yes  | Yes  |
|  A13 |  59 | PA04 | Yes  |  No  |
|  A14 |  60 | PA06 | Yes  |  No  |
|  A15 |  61 | PA07 | Yes  |  No  |

# Deep Dive
Coming soon to a README near you.
