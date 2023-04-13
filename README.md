# Kitronik-Pico-Simply-Robotics-MicroPython

A class and sample code to use the Kitronik Simply Robotics board for Raspberry Pi Pico. (www.kitronik.co.uk/5348)

This is the microPython version. 

To use save `SimplyRobotics.py` file onto the Pico so it can be imported

## Import the library and construct an instance:
``` python
from SimplyRobotics import KitronikSimplyRobotics
board = KitronikSimplyRobotics()
```

This code is designed to be used as a module. See: https://kitronik.co.uk/blogs/resources/modules-micro-python-and-the-raspberry-pi-pico for more information

## API:

### servos[] - array of 8 servos
```
servos[WHICH_SERVO].goToPosition(degrees): Sets a servo's position in degrees.
servos[WHICH_SERVO].goToPeriod(period): Sets a servo's position using the pulse length period.
servos[WHICH_SERVO].registerServo(): Sets a servo to be active.
servos[WHICH_SERVO].deregisterServo(): Sets a servo to be inactive.
    where:
    WHICH_SERVO - the servo to control (0 - 7)
    degrees - angle to go to (0 - 180)
    period - pulse length to output in uSec (500 - 2500)    
```

### motors[] - array of 4 motors
```
motors[WHICH_MOTOR].on(direction, speed): Turns the motor on at a speed in the direction.
motors[WHICH_MOTOR].off(): Turns the motor off.
    where:
    WHICH_MOTOR - the motor to control (0 - 3)
    direction - either forwards or reverse ("f" or "r")
    speed - how fast to turn the motor (0 - 100)
```

### steppers[] - array of 2 stepper motors
```
steppers[WHICH_STEPPER].step(direction): Turns the stepper motor a full step in the direction.
steppers[WHICH_STEPPER].halfStep(direction): Turns the stepper motor a half step in the direction.
    where:
    WHICH_STEPPER - the stepper motor to control (0 or 1)
    direction - either forwards or reverse ("f" or "r")

Note: stepper 0 should be connected to motors 0 and 1,
        stepper 1 should be connected to motors 3 and 4
```