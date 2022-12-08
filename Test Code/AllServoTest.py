# AllServoTest.py
# Test code that ramps each servo from 0-180-0

from SimplyRobotics import KitronikSimplyRobotics
import utime

board = KitronikSimplyRobotics()

while True:
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
