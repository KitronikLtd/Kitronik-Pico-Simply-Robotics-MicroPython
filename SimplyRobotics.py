'''
microPython Library for the Kitronik Simply Robotics board for Pico.
www.kitronik.co.uk/5348

API:
    servos[] - array of 8 servos
        servos[WHICH_SERVO].goToPosition(degrees): Sets a servo's position in degrees.
        servos[WHICH_SERVO].goToRadians(radians): Sets a servo's position in radians.
        servos[WHICH_SERVO].goToPeriod(period): Sets a servo's position using the pulse length period.
        servos[WHICH_SERVO].registerServo(): Sets a servo to be active.
        servos[WHICH_SERVO].deregisterServo(): Sets a servo to be inactive.
            where:
            WHICH_SERVO - the servo to control (0 - 7)
            degrees - angle to go to (0 - 180)
            radians - radians to go to (0 - 3.1416 (Pi to four digits))
            period - pulse length to output in uSec (500 - 2500)    
        
    motors[] - array of 4 motors
        motors[WHICH_MOTOR].on(direction, speed): Turns the motor on at a speed in the direction.
        motors[WHICH_MOTOR].off(): Turns the motor off.
            where:
            WHICH_MOTOR - the motor to control (0 - 3)
            direction - either forwards or reverse ("f" or "r")
            speed - how fast to turn the motor (0 - 100)
            
    steppers[] - array of 2 stepper motors
        steppers[WHICH_STEPPER].step(direction): Turns the stepper motor a full step in the direction.
        steppers[WHICH_STEPPER].halfStep(direction): Turns the stepper motor a half step in the direction.
            where:
            WHICH_STEPPER - the stepper motor to control (0 or 1)
            direction - either forwards or reverse ("f" or "r")
        
        Note: stepper 0 should be connected to motors 0 and 1,
              stepper 1 should be connected to motors 3 and 4
'''

from machine import Pin, PWM, ADC, time_pulse_us
from rp2 import PIO, StateMachine, asm_pio
from time import sleep, sleep_ms, sleep_us, ticks_us

'''
a class which can encapsulate a stepper motor state machine
It makes no assumptions about steps per rev - that is upto the higher level code to do

This class will drive 4 wire, bipolar steppers. 
These have 2 coils which are alternately energised to make a step.
The class is passed the pairs of motors from the board as these are analogous to the coils.
'''
class StepperMotor:
    stepSequence = [["f","-"],
                    ["-","r"],
                    ["r","-"],
                    ["-","f"]]
    halfStepSequence = [["f","-"],
                        ["f","r"],
                        ["-","r"],
                        ["r","r"],
                        ["r","-"],
                        ["r","f"],
                        ["-","f"],
                        ["f","f"]]

    def __init__(self, coilA, coilB):
        self.coils = [coilA, coilB]
        self.state = 0

    # Full stepping is 4 states, each coil only energised in turn and one at once. 
    def step(self, direction = "f"):
        if direction == "f":
            self.state += 1
            
        elif direction == "r":
            self.state -= 1
            
        else:
            # Harsh, but at least you'll know
            raise Exception("INVALID DIRECTION")
            
        if self.state > 3:
            self.state = 0
            
        if self.state < 0:
            self.state = 3
            
        for i in range(2):
            self.coils[i].on(self.stepSequence[self.state][i], 100)
    
    # Half stepping is each coil energised in turn, but sometimes both at ones (holds halfway between positions)
    def halfStep(self, direction = "f"):
        if direction == "f":
            self.state += 1
            
        elif direction == "r":
            self.state -= 1
            
        else:
            # Harsh, but at least you'll know
            raise Exception("INVALID DIRECTION")
            
        if self.state > 7:
            self.state = 0
            
        if self.state < 0:
            self.state = 7
            
        for i in range(2):
            self.coils[i].on(self.halfStepSequence[self.state][i], 100)

# This class provides a simple wrapper to the micropython PWM pins to hold them in a set for each motor
class SimplePWMMotor:
    def __init__(self, forwardPin, reversePin, startfreq = 100):
        self.forwardPin = PWM(Pin(forwardPin))
        self.reversePin = PWM(Pin(reversePin))
        self.forwardPin.freq(startfreq)
        self.reversePin.freq(startfreq)
        self.off()
    
    # Directions are "f" - forwards, "r" - reverse and "-" - off. The inclusion of off makes stepper code simpler
    def on(self, direction, speed = 0):
        # Cap speed to 0-100%
        if speed < 0:
            speed = 0
            
        elif speed > 100:
            speed = 100
            
        # Do something better here for adaptive frequency vs speed.
        frequency = 100
        
        if speed < 15:
            frequency = 20
            
        elif speed < 20:
            frequency = 50
            
        self.forwardPin.freq(frequency)
        self.reversePin.freq(frequency)
  
        # Convert 0-100 to 0-65535
        pwmVal = int(speed * 655.35)
        
        if direction == "f":
            self.forwardPin.duty_u16(pwmVal)
            self.reversePin.duty_u16(0)
            
        elif direction == "r":
            self.forwardPin.duty_u16(0)
            self.reversePin.duty_u16(pwmVal)
            
        elif direction == "-":
            self.forwardPin.duty_u16(0)
            self.reversePin.duty_u16(0)
            
        else:
            # Harsh, but at least you'll know
            raise Exception("INVALID DIRECTION")
       
    def off(self):
        self.on("-", 0)

# List of which StateMachines we have used
usedSM = [False, False, False, False, False, False, False, False]

'''
Class that controls Serovs using the RP2040 PIO to generate the pulses.

ServoControl:
Servo 0 degrees -> pulse of 0.5ms, 180 degrees 2.5ms
pulse train freq 50hz - 20mS
1uS is freq of 1000000
servo pulses range from 500 to 2500usec and overall pulse train is 20000usec repeat.
'''
class PIOServo:
    maxServoPulse = 2500
    minServoPulse = 500
    pulseTrain = 20000
    degreesToUS = 2000 / 180
    piEstimate = 3.1416
    
    # This code drives a pwm on the PIO. It is running at 2Mhz, which gives the PWM a 1uS resolution. 
    @asm_pio(sideset_init = PIO.OUT_LOW)
    def _servo_pwm():
        # First we clear the pin to zero, then load the registers. Y is always 20000 - 20uS, x is the pulse 'on' length.     
        pull(noblock) .side(0)
        # Keep most recent pull data stashed in X, for recycling by noblock
        mov(x, osr)
        # ISR must be preloaded with PWM count max
        mov(y, isr)
        # This is where the looping work is done. the overall loop rate is 1Mhz (clock is 2Mhz - we have 2 instructions to do)    
        label("loop")
        # If there is 'excess' Y number leave the pin alone and jump to the 'skip' label until we get to the X value
        jmp(x_not_y, "skip")
        nop()         .side(1)
        label("skip")
        # Count down y by 1 and jump to pwmloop. When y is 0 we will go back to the 'pull' command
        jmp(y_dec, "loop")
             
    # Doesnt actually register/unregister, just stops and starts the servo PIO
    # A side effect of this is that the PIO is not available to anyone else when running this code as written.
    def registerServo(self):
        if not self.stateMachine.active():
            self.stateMachine.active(1)
            
    def deregisterServo(self):
        if self.stateMachine.active():
            self.stateMachine.active(0)
 
    # goToPosition takes a degree position for the servo to goto. 
    # 0 degrees->180 degrees is 0->2000us, plus offset of 500uS
    # 1 degree ~ 11uS.
    # This function does the sum (degrees to uS) then calls goToPeriod to actually poke the PIO 
    def goToPosition(self, degrees):
        pulseLength = int(degrees * self.degreesToUS + 500)
        self.goToPeriod(pulseLength)
    
    # Takes the servo to change and the angle in radians to move to.
    # 0 radians to 3.1416
    def goToRadians(self, radians):
        period = int((radians / self.piEstimate) * 2000) + 500
        self.goToPeriod(period)
    
    # goToPeriod takes a uS period to send to the servo.
    # It expects a range of 500 - 2500 uS
    def goToPeriod(self, period):
        if period < 500:
            period = 500
            
        if period > 2500:
            period = 2500
        
        # Check if servo SM is active, otherwise we are trying to control a thing we do not have control over
        if self.stateMachine.active():
            self.stateMachine.put(period)
            
        else:
            # Harsh, but at least you'll know
            raise Exception("TRYING TO CONTROL UNREGISTERED SERVO")
        
    def __init__(self, servoPin):
        for i in range(8): #  StateMachine range from 0 to 7
            if usedSM[i]:
                continue # Ignore this index if already used
            try:
                self.stateMachine = StateMachine(i, self._servo_pwm, freq = 2000000, sideset_base = Pin(servoPin))
                usedSM[i] = True # Set this index to used
                break # Have claimed the SM, can leave now
            except ValueError:
                pass # External resouce has SM, move on
            if i == 7:
                # Cannot find an unused SM
                raise ValueError("Could not claim a StateMachine, all in use")

        self.stateMachine.put(self.pulseTrain)
        self.stateMachine.exec("pull()")
        self.stateMachine.exec("mov(isr, osr)")

'''
A class to provide the functionality of the Kitronik 5348 Simply Robotics board.
www.kitronik.co.uk/5348

The motors are connected as
    Motor 1 GP2 + GP5 -
    Motor 2 GP4 + GP3 -
    Motor 3 GP6 + GP9 -
    Motor 4 GP8 + GP7 -
The servo pins are 15,14,13,12,19,18,17,16 for servo 0 -> servo 7
The numbers look strange but it makes the tracking on the PCB simpler and is hidden inside this lib
'''
class KitronikSimplyRobotics:  
    def __init__ (self, centreServos = True):
        self.motors = [SimplePWMMotor(2, 5, 100), SimplePWMMotor(4, 3, 100), SimplePWMMotor(6, 9, 100), SimplePWMMotor(8, 7, 100)]
        self.steppers = [StepperMotor(self.motors[0], self.motors[1]), StepperMotor(self.motors[2], self.motors[3])]
        self.servos = [PIOServo(15), PIOServo(14), PIOServo(13), PIOServo(12), PIOServo(19), PIOServo(18), PIOServo(17), PIOServo(16)]
        
        # Connect the servos by default on construction - advanced uses can disconnect them if required.
        for i in range(8):
            self.servos[i].registerServo()
            if centreServos:
                # Set the servo outputs to middle of the range.
                self.servos[i].goToPosition(90)
