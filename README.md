Code designed for an Arduino Mega 2560 used in a Knee simulator for Dr. Anton of Tennessee Technological University.
This code initializes 6 stepper motors, 3 encoders, and 2 TOF sensors for use with an Arduino Mega 2560. When the Arduino Mega is first plugged into a laptop, the motors will run through a startup homing sequence, though the IE motor, AP Motors, and ML Motor will not and will remain at the position it started at. After the homing sequence, it waits for an input of 5 signed numbers delimited by commas in the serial terminal before moving the motors to those values (e.g. 1,2,3,4,5). The input is as follows within the quotation marks: “Flexion-Extension (degrees), Internal-External (degrees), Varus Valgus (degrees), Anterior Posterior (mm), Medial-Lateral (mm)”. An example of the input would be: 20, 10, -10, 3,0. 
Dependencies: This code requires 3 external libraries: AccelStepper by Mike McCauley, Encoder by Paul Stoffregen, and STM32duino VL53L4CD by SRA. 

## Documentation
- [Pin List](pinout.md)

23 April 2026 - Ver 1.0
