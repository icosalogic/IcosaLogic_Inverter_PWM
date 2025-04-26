# IcosaLogic_Inverter_PWM
This is an Arduino library containing logic for implementing major parts of the
software for an inverter.

This library is targeted specifically for the SAMD51 processor architecture.
It will run on many SAMD51 boards, but smaller boards, or boards using chips with
a fewer number of pins may have some configuration restrictions.

# Features
- 50 or 60 Hz output frequency
- 1, 2, or 3 output lines
- Supports half-bridge or T-type implementations
- Flexible scheduling of ADC readings
