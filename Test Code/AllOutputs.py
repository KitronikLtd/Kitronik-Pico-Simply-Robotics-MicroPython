import SimplyRobotics
import utime
import _thread

'''
    uses a thread on the second core to drive the servos, whilst the first core is driving the motors
'''

board = SimplyRobotics.KitronikSimplyRobotics()
directions = ["f","r"]

def DriveServos():
    while True:
        print("servos")
        for degrees in range(180):
            for servo in range(8):
                board.Servos[servo].goToPosition(degrees)
            utime.sleep_ms(10) #ramp speed over 10x180ms => approx 2 seconds.
        for degrees in range(180):
            for servo in range(8):
                board.Servos[servo].goToPosition(180-degrees)
            utime.sleep_ms(10) #ramp speed over 10x180ms => approx 2 seconds.

_thread.start_new_thread(DriveServos,())

while True:
    print ("motors")
    for direction in directions:
        for speed in range(0,100):
            for motor in range(0,4):
              board.Motors[motor].On(direction,speed)
            utime.sleep_ms(100) #ramp speed over 25x100ms => approx 2.5 second.
        utime.sleep_ms(100)
        for speed in range(100,0,-1):
            for motor in range(0,4):
              board.Motors[motor].On(direction,speed)
            utime.sleep_ms(100) #ramp speed over 25x100ms => approx 2.5 second.
        utime.sleep_ms(1000)

 


'''
while True:
    for direction in directions:
      for motor in range(0,4):
           board.theMotors[motor].On(direction,100)
      utime.sleep_ms(5000) # let motor start and run
      for motor in range(0,4):
        board.theMotors[motor].Off() #then crash stop
        utime.sleep_ms(10)


for motor in range(0,4):
    board.theMotors[motor].On("r",90)
    '''