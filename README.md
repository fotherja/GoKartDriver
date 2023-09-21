# GoKartDriver
A DC motor controller for a children's gokart

GoKart DC Motor Driver - 21st September 2023 - James Fotherby

Runs on a Arduino Pro-Mini
This program reads a throttle voltage (0.8-4.0v from a Hall effect pedal) on pin A0
This is used to generate a torque demand - ie. a motor current setpoint
A PI controller takes the measured current (from a 50A ACS758 Hall effect current sensor connected to pin A3) and the setpoint to generate an output PWM Duty cycle value

Timer 1 is configured to produce a 4kHz phase correct PWM signal which feeds the HW-039 H-Bridge driver via pins 9 and 10. Pins 7 and 12 enable to drivers.

There is a forward reverse switch conected to pin 5 - We only allow the motor to change direction when there is no current flowing and no voltage applied to the motor

We synchronise our ADC samples with our timer1 PWM and use interrupts:
- If our PWM Duty is >50% we take ADC readings in the middle of the high periods else we sample in the middle of the low periods
- This ensures we sample as far away as possible from the switching transients to get the best and most consistent current readings.
- We switch alternately between sampling the throttle and motor current ADC channels
- So we sample at the PWM Frequency of 4kHz (current measurements at 2kHz and throttle measurements at 2kHz)

The ADC Throttle values range between: 185-680
The Current sensor reports about 512. +ve values represent a forward torque at 10/Amp. Hence an ADC reading of 612 equates to +10A of motor current


