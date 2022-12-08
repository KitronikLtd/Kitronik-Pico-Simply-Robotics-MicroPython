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
        servos[WHICH_SERVO].goToPosition(degrees): Sets a servo to a degree position. servo - which to set (0-7), degrees - angle to goto (0-180)
        servos[WHICH_SERVO].goToPeriod(): Sets a servo to a degree position. servo - which to set (0-7), degrees - angle to goto (0-180)
        servos[WHICH_SERVO].registerServo(): Sets a servo to a degree position. servo - which to set (0-7), degrees - angle to goto (0-180)
        servos[WHICH_SERVO].deregisterServo(): Sets a servo to a degree position. servo - which to set (0-7), degrees - angle to goto (0-180)
            where:
            WHICH_SERVO - the servo to control (0-7)
            degrees - angle to goto (0-180)
            period - pulse length to output in uSec (500 - 2500)
            
        
   ### motors[] - array of 4 motors
        motors[WHICH_MOTOR].on(direction, speed)
        motors[WHICH_MOTOR].off()
            where:
            WHICH_MOTOR - the motor to control (0-3)
            
   ### steppers[] - array of 2 stepper motors
        steppers[WHICH_STEPPER].step(direction)
        steppers[WHICH_STEPPER].halfStep(direction)
            where:
            WHICH_STEPPER - the stepper to control  (0 or 1) - stepper should be connected to motors 0 and 1 (stepper 0) or motors 3 and 4 (stepper 1)
            direction = ["f","r"] -> Forwards or Reverse.
