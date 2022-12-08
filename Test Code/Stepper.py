from SimplyRobotics import KitronikSimplyRobotics
import utime

board = KitronikSimplyRobotics()

while True:
    # 1 rev one way
    for i in range(200):
        board.steppers[0].step("f")
        board.steppers[1].step("f")
        utime.sleep(0.01)
        
    utime.sleep (1)
    
    # 1 rev the other way
    for i in range(200):
        board.steppers[0].step("r")
        board.steppers[1].step("r")
        utime.sleep(0.01)
        
    utime.sleep(1)
    
    # Half step 1 rev back again
    for i in range(400):
        board.steppers[0].halfStep("f")
        board.steppers[1].halfStep("f")
        utime.sleep(0.01)
        
    utime.sleep(1)
    
    # And reverse
    for i in range(400):
        board.steppers[0].halfStep("r")
        board.steppers[1].halfStep("r")
        utime.sleep(0.01)
        
    utime.sleep(1)
    
    # 1/4 rev to finish
    for i in range(50):
        board.steppers[0].step("f")
        board.steppers[1].step("f")
        utime.sleep(0.01)
        
    utime.sleep(1)
