from SimplyRobotics import KitronikSimplyRobotics
import utime

board = KitronikSimplyRobotics()

while True:
    #1 rev one way
    for i in range(200):
        board.Steppers[0].Step("f")
        board.Steppers[1].Step("f")
        utime.sleep(0.01)
    utime.sleep (1)
    #1 rev the other way
    for i in range(200):
        board.Steppers[0].Step("r")
        board.Steppers[1].Step("r")
        utime.sleep(0.01)
    #halfstep 1 rev back again
    utime.sleep(1)
    for i in range(400):
        board.Steppers[0].HalfStep("f")
        board.Steppers[1].HalfStep("f")
        utime.sleep(0.01)
    utime.sleep(1)
    #and reverse
    for i in range(400):
        board.Steppers[0].HalfStep("r")
        board.Steppers[1].HalfStep("r")
        utime.sleep(0.01)
    utime.sleep(1)    
    #1/4 rev to finish
    for i in range(50):
        board.Steppers[0].Step("f")
        board.Steppers[1].Step("f")
        utime.sleep(0.01)
    utime.sleep(1)
