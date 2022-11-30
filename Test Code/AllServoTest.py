#AllServoTest.py
# test code that ramps each servo from 0-180-0 
from SimplyRobotics import KitronikSimplyRobotics
import utime


board = KitronikSimplyRobotics()


while True:
    for degrees in range(180):
        for servo in range(8):
            board.Servos[servo].goToPosition(degrees)
        utime.sleep_ms(10) #ramp speed over 10x180ms => approx 2 seconds.
    for degrees in range(180):
        for servo in range(8):
            board.Servos[servo].goToPosition(180-degrees)
        utime.sleep_ms(10) #ramp speed over 10x180ms => approx 2 seconds.
