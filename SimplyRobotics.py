'''
microPython Library for the Kitronik Simply Robotics board for Pico.
www.kitronik.co.uk/5348

API:
    Servos[] - array of 8 servos
        Servos[WHICH_SERVO].goToPosition(degrees): Sets a servo to a degree position. servo - which to set (0-7), degrees - angle to goto (0-180)
        Servos[WHICH_SERVO].goToPeriod(): Sets a servo to a degree position. servo - which to set (0-7), degrees - angle to goto (0-180)
        Servos[WHICH_SERVO].registerServo(): Sets a servo to a degree position. servo - which to set (0-7), degrees - angle to goto (0-180)
        Servos[WHICH_SERVO].deregisterServo(): Sets a servo to a degree position. servo - which to set (0-7), degrees - angle to goto (0-180)
            where:
            WHICH_SERVO - the servo to control (0-7)
            degrees - angle to goto (0-180)
            period - pulse length to output in uSec (500 - 2500)
            
        
    Motors[] - array of 4 motors.
        Motors[WHICH_MOTOR].On(direction, speed)
        Motors[WHICH_MOTOR].Off()
            where:
            WHICH_MOTOR - the motor to control (0-3)
            
    Steppers[]
        Steppers[WHICH_STEPPER].Step(direction)
        Steppers[WHICH_STEPPER].HalfStep(direction)
            where:
            WHICH_STEPPER - the stepper to control  (0 or 1) - stepper should be connected to motors 0 and 1 (stepper 0) or motors 3 and 4 (stepper 1)
            direction = ["f","r"] -> Forwards or Reverse.
        
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

class stepperMotor:
    
    StepSequence = [["f","-"],
                    ["-","r"],
                    ["r","-"],
                    ["-","f"]]

    HalfStepSequence = [["f","-"],
                        ["f","r"],
                        ["-","r"],
                        ["r","r"],
                        ["r","-"],
                        ["r","f"],
                        ["-","f"],
                        ["f","f"]]

    def __init__(self, CoilA, CoilB):
        self.coils =[CoilA,CoilB]
        self.state = 0


    #full stepping is 4 states, each coil only energised in turn and one at once. 
    def Step(self,direction = "f"):
        if(direction == "f"):
            self.state += 1
        elif (direction =="r"):
            self.state -= 1
        else:
            raise Exception("INVALID DIRECTION") #harsh, but at least you'll know
        if (self.state > 3):
            self.state = 0
        if (self.state < 0):
            self.state = 3
        for i in range(2):
            self.coils[i].On(self.StepSequence[self.state][i],100)
    
    #half stepping is each coil energised in turn, but sometimes both at ones (holds halfway between positions)
    def HalfStep(self, direction = "f"):
        global state
        if(direction == "f"):
            self.state += 1
        elif (direction =="r"):
            self.state -= 1
        else:
            raise Exception("INVALID DIRECTION") #harsh, but at least you'll know
        if (self.state > 7):
            self.state = 0
        if (self.state < 0):
            self.state = 7
        for i in range(2):
            self.coils[i].On(self.HalfStepSequence[self.state][i],100)

 
 
    
# This class provides a simple wrapper to the micropython PWM pins to hold them in a set for each motor
class SimplePWMMotor:

    def __init__(self,ForwardPin, ReversePin, startfreq = 100):
        self.forwardPin = PWM(Pin(ForwardPin))
        self.reversePin = PWM(Pin(ReversePin))
        self.forwardPin.freq(startfreq)
        self.reversePin.freq(startfreq)
        self.Off()
    #directions are "f" - forwards, "r" - reverse and "-" - off. The inclusion of off makes stepper code simpler
        
    def On(self,direction, speed = 0):
              #cap speed to 0-100%
        if (speed<0):
            speed = 0
        elif (speed>100):
            speed=100
        #do something better here for adaptive frequency vs speed.
        frequency = 100
        if (speed < 15):
            frequency = 20
        elif (speed < 20):
            frequency = 50
        self.forwardPin.freq(frequency)
        self.reversePin.freq(frequency)
  
        #convert 0-100 to 0-65535
        PWMVal = int(speed*655.35)
        if direction == "f":
            self.forwardPin.duty_u16(PWMVal)
            self.reversePin.duty_u16(0)
        elif direction == "r":
            self.forwardPin.duty_u16(0)
            self.reversePin.duty_u16(PWMVal)
        elif direction == "-":
            self.forwardPin.duty_u16(0)
            self.reversePin.duty_u16(0)
        else:
            raise Exception("INVALID DIRECTION") #harsh, but at least you'll know
       
    def Off(self):
        self.On("-",0)


#Class that controls Serovs using the RP2040 PIO to generate the pulses.
class PIOServo:
    #ServoControl:
    #Servo 0 degrees -> pulse of 0.5ms, 180 degrees 2.5ms
    #pulse train freq 50hz - 20mS
    #1uS is freq of 1000000
    #servo pulses range from 500 to 2500usec and overall pulse train is 20000usec repeat.
    maxServoPulse = 2500
    minServoPulse = 500
    pulseTrain = 20000
    degreesToUS = 2000/180
    
    #this code drives a pwm on the PIO. It is running at 2Mhz, which gives the PWM a 1uS resolution. 
    @asm_pio(sideset_init=PIO.OUT_LOW)
    def _servo_pwm():
    #first we clear the pin to zero, then load the registers. Y is always 20000 - 20uS, x is the pulse 'on' length.     
        pull(noblock) .side(0)
        mov(x, osr) # Keep most recent pull data stashed in X, for recycling by noblock
        mov(y, isr) # ISR must be preloaded with PWM count max
    #This is where the looping work is done. the overall loop rate is 1Mhz (clock is 2Mhz - we have 2 instructions to do)    
        label("loop")
        jmp(x_not_y, "skip") #if there is 'excess' Y number leave the pin alone and jump to the 'skip' label until we get to the X value
        nop()         .side(1)
        label("skip")
        jmp(y_dec, "loop") #count down y by 1 and jump to pwmloop. When y is 0 we will go back to the 'pull' command
             
    # doesnt actually register/unregister, just stops and starts the servo PIO
    # a side effect of this is that the PIO is not available to anyone else when running this code as written.
    
    def registerServo(self):
        if(not self.stateMachine.active()):
            self.stateMachine.active(1)
    def deregisterServo(self):
        if(self.stateMachine.active()):
            self.stateMachine.active(0)
 
    # goToPosition takes a degree position for the servo to goto. 
    # 0 degrees->180 degrees is 0->2000us, plus offset of 500uS
    # 1 degree ~ 11uS.
    # This function does the sum (degrees to uS) then calls goToPeriod to actually poke the PIO 
    def goToPosition(self, degrees):
        pulseLength = int(degrees*self.degreesToUS + 500)
        self.goToPeriod(pulseLength)
    
    #goToPeriod takes a uS period to send to the servo.
    #It expects a range of 500 - 2500 uS
    def goToPeriod(self, period):
        if(period < 500):
            period = 500
        if(period >2500):
            period =2500
        #check if servo SM is active, otherwise we are trying to control a thing we do not have control over
        if self.stateMachine.active():
            self.stateMachine.put(period)
        else:
            raise Exception("TRYING TO CONTROL UNREGISTERED SERVO") #harsh, but at least you'll know
        
    def __init__(self, Servo, ServoPin):
        self.stateMachine = StateMachine(Servo, self._servo_pwm, freq=2000000, sideset_base=Pin(ServoPin))
        self.stateMachine.put(self.pulseTrain)
        self.stateMachine.exec("pull()")
        self.stateMachine.exec("mov(isr, osr)")  


# A class to provide the functionality of the Kitronik 5348 Simply Robotics board.
# www.kitronik.co.uk/5348
class KitronikSimplyRobotics:
    '''
    The motors are connected as
        Motor 1 GP2 + GP5 -
        Motor 2 GP4 + GP3 -
        Motor 3 GP6 + GP9 -
        Motor 4 GP8 + GP7 -
    The servo pins are 15,14,13,12,19,18,17,16 for servo 0 -> servo 7
    The numbers look strange but it makes the tracking on the PCB simpler and is hidden inside this lib
    '''          
    def __init__ (self, centreServos = True):
        self.Motors = [SimplePWMMotor(2,5,100),SimplePWMMotor(4,3,100),SimplePWMMotor(6,9,100),SimplePWMMotor(8,7,100)]
        self.Steppers = [stepperMotor(self.Motors[0], self.Motors[1]),stepperMotor(self.Motors[2],self.Motors[3])]
        self.Servos = [PIOServo(0,15),PIOServo(1,14),PIOServo(2,13),PIOServo(3,12),PIOServo(4,19),PIOServo(5,18),PIOServo(6,17),PIOServo(7,16)]
        #connect the servos by default on construction - advanced uses can disconnect them if required.
        for i in range(8):
            self.Servos[i].registerServo()
            if(centreServos):
                self.Servos[i].goToPosition(90) #set the servo outputs to middle of the range.

