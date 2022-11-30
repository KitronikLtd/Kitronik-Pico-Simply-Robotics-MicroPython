# Kitronik-Pico-Simply-Robotics-MicroPython

A class and sample code to use the Kitronik Simply Robotics board for Raspberry Pi Pico. (www.kitronik.co.uk/5348)

This is the microPython version. 

To use save `SimplyRobotics.py` file onto the Pico so it can be imported

## Import the library and construct an instance:
``` python
import SimplyRobotics
board = SimplyRobotics.KitronikSimplyRobotics()
```

This code is designed to be used as a module. See: https://kitronik.co.uk/blogs/resources/modules-micro-python-and-the-raspberry-pi-pico for more information


## API:

   ### Servos[] - array of 8 servos
        Servos[WHICH_SERVO].goToPosition(degrees): Sets a servo to a degree position. servo - which to set (0-7), degrees - angle to goto (0-180)
        Servos[WHICH_SERVO].goToPeriod(): Sets a servo to a degree position. servo - which to set (0-7), degrees - angle to goto (0-180)
        Servos[WHICH_SERVO].registerServo(): Sets a servo to a degree position. servo - which to set (0-7), degrees - angle to goto (0-180)
        Servos[WHICH_SERVO].deregisterServo(): Sets a servo to a degree position. servo - which to set (0-7), degrees - angle to goto (0-180)
            where:
            WHICH_SERVO - the servo to control (0-7)
            degrees - angle to goto (0-180)
            period - pulse length to output in uSec (500 - 2500)
            
        
  ### Motors[] - array of 4 motors.
        Motors[WHICH_MOTOR].On(direction, speed)
        Motors[WHICH_MOTOR].Off()
            where:
            WHICH_MOTOR - the motor to control (0-3)
            
   ### Steppers[]
        Steppers[WHICH_STEPPER].Step(direction)
        Steppers[WHICH_STEPPER].HalfStep(direction)
            where:
            WHICH_STEPPER - the stepper to control  (0 or 1) - stepper should be connected to motors 0 and 1 (stepper 0) or motors 3 and 4 (stepper 1)
            direction = ["f","r"] -> Forwards or Reverse.
