import SimplyRobotics
import utime
import _thread

'''
Uses a thread on the second core to drive the servos, whilst the first core is driving the motors
'''

board = SimplyRobotics.KitronikSimplyRobotics()
directions = ["f", "r"]

def driveServos():
    while True:
        print("servos")
        
        for degrees in range(180):
            for servo in range(8):
                board.servos[servo].goToPosition(degrees)
                
            # Ramp speed over 10x180ms => approx 2 seconds.
            utime.sleep_ms(10)
            
        for degrees in range(180):
            for servo in range(8):
                board.servos[servo].goToPosition(180 - degrees)
            
            # Ramp speed over 10x180ms => approx 2 seconds.
            utime.sleep_ms(10)

_thread.start_new_thread(driveServos, ())

while True:
    print ("motors")
    
    for direction in directions:
        for speed in range(0, 100):
            for motor in range(0, 4):
                board.motors[motor].on(direction, speed)
            
            # Ramp speed over 25x100ms => approx 2.5 second.
            utime.sleep_ms(100)
            
        utime.sleep_ms(100)
        
        for speed in range(100, 0, -1):
            for motor in range(0, 4):
                board.motors[motor].on(direction, speed)
            
            # Ramp speed over 25x100ms => approx 2.5 second.
            utime.sleep_ms(100)
        
        utime.sleep_ms(1000)
