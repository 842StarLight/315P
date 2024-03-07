#region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)


# wait for rotation sensor to fully initialize
wait(30, MSEC)


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

#endregion VEXcode Generated Robot Configuration
import time
import math
import random
# dfgdf
dt_left = MotorGroup(
    Motor(Ports.PORT8, GearSetting.RATIO_6_1, True),
    Motor(Ports.PORT17, GearSetting.RATIO_6_1, True),
    Motor(Ports.PORT19, GearSetting.RATIO_6_1, True)
)
dt_right = MotorGroup(
    Motor(Ports.PORT2, GearSetting.RATIO_6_1, False),
    Motor(Ports.PORT5, GearSetting.RATIO_6_1, False),
    Motor(Ports.PORT11, GearSetting.RATIO_6_1, False)
)
dt_left.set_stopping(BRAKE) 
dt_right.set_stopping(BRAKE)

intake = Motor(Ports.PORT14, GearSetting.RATIO_6_1, False)
intake.set_velocity(150, PERCENT)
endgame = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
# driver control
def driver_control():
    # cp controls
    controller_1.buttonL1.pressed(lambda: intake.spin(FORWARD))
    controller_1.buttonR1.pressed(lambda: intake.spin(REVERSE))

    controller_1.buttonL1.released(intake.stop)
    controller_1.buttonR1.released(intake.stop)

    controller_1.buttonL2.pressed(lambda: endgame.spin(FORWARD))
    controller_1.buttonR2.pressed(lambda: endgame.spin(REVERSE))

    controller_1.buttonL2.released(endgame.stop)
    controller_1.buttonR2.released(endgame.stop)
    # ready, set, drive!
    brain.timer.clear()
    while True:
        wait(10, MSEC)
        # dt controls
        straight_speed = controller_1.axis3.position()
        turn_speed = controller_1.axis1.position()
        l = straight_speed + turn_speed
        r = straight_speed - turn_speed
        dt_left.spin((FORWARD if l >= 0 else REVERSE), abs(l)*12/100, VOLT)
        dt_right.spin((FORWARD if r >= 0 else REVERSE), abs(r)*12/100, VOLT)

def autonomous():
    def drive4( dire, tim, v=100):
        dt_left.set_velocity(v, PERCENT)
        dt_right.set_velocity(v, PERCENT)
        dt_left.spin(dire)
        dt_right.spin(dire)
        wait(tim, SECONDS)
        dt_left.stop()
        dt_right.stop()
    drive4(REVERSE, 4)
    drive4(FORWARD, 0.25)
    drive4(REVERSE, 0.5)
    drive4(FORWARD, 0.25)
    drive4(REVERSE, 0.5)
competition = Competition(driver_control, autonomous)
