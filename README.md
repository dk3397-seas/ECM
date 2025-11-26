This document provides a concise reference for all adjustable parameters, pin assignments, and control commands used in the Z-axis ECM motion system. It is intended to help future users quickly understand how the code operates, how to tune motion and pump behavior, and how to safely interact with the system through the serial command interface.
Pins: 
PUL_PIN- Set as arduino pin connected to PUL on motor driver, controls the step pulse of motor 
DIR_PIN- Set as arduino pin connected to DIR on motor driver, controls the pulse direction of motor
ENA_PIN- Set as arduino pin connected to ENA on motor driver, turns on motor, locking it in place, cannot be moved by hand
PUMP_PIN- Set as arduino pin connected to PWM, controls voltage sent to pump
LIMIT_TOP_PIN- Set as arduino pin connected to top limit switch, gives reading if limit is reached
LIMIT_BOTTOM_PIN- Set as arduino pin connected to bottom limit switch, gives reading if limit is reached
Parameters:
STEPS_PER_JOG- Designates the number in pluses in one rotation of the z-axis motor, should be tuned to motor specification, in our motor setup correlates to 1mm of movement. 
PULSE_HIGH_US- Indicates the time between control pulse width, should be turned to your motor controllers specifications and left as it 
JOG_PULSE_LOW_UP- Tunes the speed of motor in the “Upward” direction, lower number means higher speed 
JOG_PULSE_HIGH_UP- Tunes the speed of motor in the “Downward” direction, lower number means higher speed 
STEPS_PER_FINE_JOG- Designates the number in pluses in a small fine movement of the z-axis motor, should be tuned to motor specification, in our motor setup correlates to 0.01mm of movement.
DRILL_PULSE_LOW_US- Tunes the speed of the motor in drilling mode, can be calculated by the material removal rate and used to keep gap size constant. 
DRILL_PUMP_PWM- Tunes the electrolyte flow rate from pump can be any number between 0-255 as controlled by the PWM
HOME_DIR_UP_LOGICAL- Indicated which direction the motor should move to home, denoted by true or false, in our case, true so the motor moves up. 
HOME_PULSE_LOW_US- Tunes the speed of motor in “homing” mode, lower number means higher speed
MOTOR_DIR_INVERT- Can be used to switch direction of movement called by motor if intuition seems wrong to the user, denoted by true or false. 
Controls: 
u - Calls motor to move up 1mm, can add numbers after to multiply and move that many mm
 d - Calls motor to move down 1mm, can add numbers after to multiply and move that many mm
f - Calls the motor to move up 0.01mm, can add numbers after to multiply and move that many 0.01mm
g - Calls the motor to move up 0.01mm, can add numbers after to multiply and move that many 0.01mm 
p - Runs pump at power that it will be run during drilling
[space]- Starts and stops drilling function depending on whether it is on or off
H- Begins homing procedure
k - Kills all functions, shuts down the system. 
r - Reports system state, status of motor, homing, pump, and limits.  
