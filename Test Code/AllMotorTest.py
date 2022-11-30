#AllMotorTest.py
# test code that ramps each motor 0-100-0 then changes direction and does it again.
#all motors run at once, but with staggered timings

from SimplyRobotics import KitronikSimplyRobotics
import utime


board = KitronikSimplyRobotics()
directions = ["f","r"]


while True:
    for direction in directions:
        for speed in range(0,25):
            board.Motors[0].On(direction, speed)
            board.Motors[1].On(direction, 25-speed)
            board.Motors[2].On(direction, 50-speed)
            board.Motors[3].On(direction, 75-speed)
            utime.sleep_ms(100) #ramp speed over 25x100ms => approx 2.5 second.
        for speed in range(0,25):
            board.Motors[0].On(direction, 25+speed)
            board.Motors[1].On(direction, speed)
            board.Motors[2].On(direction, 25-speed)
            board.Motors[3].On(direction, 50-speed)
            utime.sleep_ms(100) 
        for speed in range(0,25):
            board.Motors[0].On(direction, 50+speed)
            board.Motors[1].On(direction, 25+speed)
            board.Motors[2].On(direction, speed)
            board.Motors[3].On(direction, 25-speed)
            utime.sleep_ms(100) 
        for speed in range(0,25):
            board.Motors[0].On(direction, 75+speed)
            board.Motors[1].On(direction, 50+speed)
            board.Motors[2].On(direction, 25+speed)
            board.Motors[3].On(direction, speed)
            utime.sleep_ms(100) 
        for speed in range(0,25):
            board.Motors[0].On(direction, 100-speed)
            board.Motors[1].On(direction, 75+speed)
            board.Motors[2].On(direction, 50+speed)
            board.Motors[3].On(direction, 25+speed)
            utime.sleep_ms(100) 
        for speed in range(0,25):
            board.Motors[0].On( direction, 75-speed)
            board.Motors[1].On(direction, 100-speed)
            board.Motors[2].On(direction, 75+speed)
            board.Motors[3].On(direction, 50+speed)
            utime.sleep_ms(100)
        for speed in range(0,25):
            board.Motors[0].On(direction, 50-speed)
            board.Motors[1].On(direction, 75-speed)
            board.Motors[2].On(direction, 100-speed)
            board.Motors[3].On(direction, 75+speed)
            utime.sleep_ms(100) 
        for speed in range(0,25):
            board.Motors[0].On(direction, 25-speed)
            board.Motors[1].On(direction, 50-speed)
            board.Motors[2].On(direction, 75-speed)
            board.Motors[3].On(direction, 100-speed)
            utime.sleep_ms(100) 

  
    
