# IcosaLogic_Inverter_PWM
This is an Arduino library containing logic for generating PWM signals for an inverter and using
feedback signals such as voltage and current readings to accurately control those PWM sigals.

The user describes inverter details in an input parameters structure, and the library
manages operation of the inverter with no further interaction from the application.
The input parameters tell the library how the PWM signals should be generated,
and how feedback signals like voltage and current readings should be performed.
The library does the rest.

This library is targeted specifically for the SAMD51 processor architecture.
Much of the implementation details will be incomprehensible to users who are not familiar with
SAMD51 internals.
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
- The documentation is incomplete (no big surprise)

# Examples
Because this library is so intimately tied with the hardware details of both the SAMD51 processor
on which it runs and the board on which the processor is mounted, each board requires its own example.

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
Those methods show how to do a short controlled run, with less output data to examine.

# Structure of this Document
The rest of this document is divided into two sections:
- First is a Quick Start section giving users basic overview of how to configure, start, and stop the inverter
- Next is a Deep Dive section giving users more information of the target environment and the rationale and details for the design and implementation of the library

# Quick Start
Coming soon.

# Deep Dive
Coming soon.
