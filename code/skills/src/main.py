from vex import *
import urandom
import time
import math
import random
# constants for pneumatics
OPEN = True
CLOSE = False
# brain
brain = Brain()
# controller
controller_1 = Controller(PRIMARY)
# logging explanation
controller_1.screen.set_cursor(1, 1)
controller_1.screen.print("time")

controller_1.screen.set_cursor(2, 1)
controller_1.screen.print("dt temps")

controller_1.screen.set_cursor(3, 1)
controller_1.screen.print("cata temp")
# sensors
dist = Distance(Ports.PORT2)

orientation = Inertial(Ports.PORT1)
orientation.calibrate()
while orientation.is_calibrating():
    wait(10, MSEC)
# drivetrain
dt_left = MotorGroup(
    Motor(Ports.PORT11, GearSetting.RATIO_6_1, True),
    Motor(Ports.PORT12, GearSetting.RATIO_6_1, True),
    Motor(Ports.PORT13, GearSetting.RATIO_6_1, True)
)
dt_right = MotorGroup(
    Motor(Ports.PORT20, GearSetting.RATIO_6_1, False),
    Motor(Ports.PORT19, GearSetting.RATIO_6_1, False),
    Motor(Ports.PORT18, GearSetting.RATIO_6_1, False)
)
dt_left.set_stopping(BRAKE) 
dt_right.set_stopping(BRAKE)
# components
intake = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
catapult = Motor(Ports.PORT9, GearSetting.RATIO_18_1, False)
# solenoids
wings = DigitalOut(brain.three_wire_port.a)
endg = DigitalOut(brain.three_wire_port.b)

class Components:
    def __init__(self, cata_speed):
        catapult.set_velocity(cata_speed, PERCENT)
        intake.set_velocity(200, PERCENT)

        self.wing_value = False
        self.intake_value = None
    def intake(self, direction):
        if direction == None:
            intake.stop()
        else:
            intake.spin(direction)
        self.intake_value = direction
    def catapult(self, direction=FORWARD):
        if direction == None:
            catapult.stop()
        else:
            catapult.spin(direction)
    def wings(self, value=None):
        if value == None:
            self.wing_value = not self.wing_value
        else:
            self.wing_value = value
        wings.set(self.wing_value)
class Drivetrain:
    def __init__(self, gear_ratio, wheel_diameter, wheelbase):
        self.gear_ratio = gear_ratio
        self.wheel_diameter = wheel_diameter
        self.wheelbase = wheelbase
    def drive4(self, inches, speed=100, timeout=10**2):
        '''
        ### I think this is the mosted using dt auton function; it simply drives a number of inches.
        We are planning to eventually add a PID loop or something here later on.
        A custom timeout solution end the command and moves on if the robot gets stuck or takes too long.

        #### Arguments:
            inches: how far to drive, in inches forward. Negative values are accepted and interpreted as inches backwards.
            speed (>0, <100): speed of both sides, in percent
            timeout (>0): DEPRECATED - do n
        #### Returns:
            None
        #### Examples:
            # slowly drive 24 inches (one tile):
            dt.drive4(24, speed=25)
            # drives across three tiles
            dt.drive4(24*3)
        '''
        # speeds
        dt_left.set_velocity(speed, PERCENT)
        dt_right.set_velocity(speed, PERCENT)
        # core spin
        dt_left.spin_for(FORWARD, (inches/(math.pi*self.wheel_diameter))*self.gear_ratio, TURNS, wait=False)
        dt_right.spin_for(FORWARD, (inches/(math.pi*self.wheel_diameter))*self.gear_ratio, TURNS, wait=False)
        # record initial time; let the dt accelerate
        initial_time = brain.timer.time(SECONDS)
        wait(50, MSEC)
        # main timeout loop
        while True:
            is_not_moving = dt_left.is_done() or dt_left.velocity(PERCENT)*dt_right.velocity(PERCENT) == 0
            timeouted = brain.timer.time(SECONDS)-initial_time >= timeout
            print(is_not_moving, timeouted)
            if timeouted or is_not_moving:
                break
            wait(10, MSEC)
        # stop
        dt_left.stop()
        dt_right.stop()
        # logging
        print("drive4/done", inches, "in, took", brain.timer.time(SECONDS)-initial_time, 'sec')
    def turn2(self, angle_unmodded, speed=41):
        '''
        ### A turn-to-heading function which uses a feedback loop (just P for now) in tandem with the inertial to achieve very precise results.

        #### Arguments:
            angle_unmodded: angle turning to, in degrees clockwise. Negative inputs are allowed and will be interpreted as degrees counterclockwise. Reference point is set during calibration
            speed (>0, <100): absolute speeds of both sides, in percent (41 has been empirically derived to be the fastest)
        #### Returns:
            None
        #### Examples:
            # slowly turn to a heading of 180˚:
            dt.turn2(180, speed=15)
            # draws a square
            for i in range(4):
                dt.turn2(i*90)
                dt.drive4(10)
        '''
        # constants
        angle = angle_unmodded % 360
        print(angle)
        # initial heading
        h = orientation.heading(DEGREES)
        # main loop
        while abs(angle - h) % 360 > 1:
            # calculations
            h = orientation.heading(DEGREES)
            vel = abs((angle - h + 180) % 360 - 180) * speed * 2 / 180 + 3
            # action!
            dt_left.spin(FORWARD if (angle - h + 180) % 360 - 180 > 0 else REVERSE, vel, PERCENT)
            dt_right.spin(REVERSE if (angle - h + 180) % 360 - 180 > 0 else FORWARD, vel, PERCENT)
            # wait
            wait(10, MSEC)
        # stop dt
        dt_left.stop()
        dt_right.stop()
        # rerun if drift
        if abs(angle - orientation.heading(DEGREES)) % 360 > 1:
            self.turn2(angle, speed=10)
    def arc(self, rad, head, side, duration, speed=65):
        '''
        ### An arc function which allows us to skip parts in autons where we alternate between turn2's and drive4's.

        #### Arguments:
            rad (>0): Radius of arc, in inches
            head (FORWARD/REVERSE): whether the robot is moving for/backwards
            side (LEFT/RIGHT): which side the robot is arc towards
            duration (>0): how long the arc should last, in seconds
            speed (>0, <100): average speed of both sides, in percen
        #### Returns:
            None
        #### Examples:
            # slowly draw a circle with diameter of 20in
            dt.arc(10, FORWARD, RIGHT, 7, speed=25)
            # draws a rounded square
            for i in range(4):
                dt.drive4(2)
                dt.arc(5, FORWARD, RIGHT, 1.5, speed=65) # adjust duration to make this a quarter circle
        '''
        # convert sides to numbers
        aside = 1 if side == RIGHT else -1
        ahead = 1 if head == FORWARD else -1
        # targets for each side
        left = ahead*(rad+aside*self.wheelbase/2) # the parenthesized portion is the abs value
        right = ahead*(rad-aside*self.wheelbase/2) # of the circumference of the side's circle
        # velocities
        sconst = speed/(abs(left)/2+abs(right)/2) # to add the speed factor
        print('fastarc', rad, head, side, duration, speed)
        # and away she goes!
        dt_right.spin(FORWARD, right*sconst*12/100, VOLT)
        dt_left.spin(FORWARD, left*sconst*12/100, VOLT)
        # wait, then stop
        wait(duration, SECONDS)
        dt_left.stop()
        dt_right.stop()
        # in case you need it - here's a simple fn to determine duration of an arc
        '''
        # assuming arc is already set up
        # press B when arc should end
        def arc_dur(self, rad, head, side, speed=65)
            # convert sides to numbers
            aside = 1 if side == RIGHT else -1
            ahead = 1 if head == FORWARD else -1
            # targets for each side
            left = ahead*(rad+aside*self.wheelbase/2) # the parenthesized portion is the abs value
            right = ahead*(rad-aside*self.wheelbase/2) # of the circumference of the side's circle
            # velocities
            sconst = speed/(abs(left)/2+abs(right)/2) # to add the speed factor
            print('ARCTEST', rad, head, side, speed)
            # and away she goes!
            dt_right.spin(FORWARD, right*sconst*12/100, VOLT)
            dt_left.spin(FORWARD, left*sconst*12/100, VOLT)
            
            initial_time = brain.timer.time(SECONDS)
            while not controller_1.buttonB.pressing():
                wait(1, MSEC)
            dt_left.stop()
            dt_right.stop()
            print(brain.timer.time(SECONDS)-initial_time)
        '''
        
       
dt = Drivetrain(60/36, 3.25, 11)
cp = Components(80)
# helper functions
def matchload_setup(mangle=20):
    dt.drive4(12)
    dt.turn2(180-mangle)
    dt.drive4(12)
    dt.drive4(2.5, speed=25)
    print(orientation.heading(DEGREES))
    # match loading
    cp.catapult()
    initial_time = brain.timer.time(SECONDS)
    dt.turn2(180-mangle)
# driver control
def driver_control():
    # core archie setup
    def print_all(msg):
        controller_1.screen.clear_screen()
        for i in range(3):
            controller_1.screen.set_cursor(i+1,1)
            controller_1.screen.print(msg)
    log_n = 0 # a var to keep track of segregated logging
    brain.timer.clear()
    # skills mode + matchloading setup
    hang_n = 10
    skills = False
    if controller_1.buttonY.pressing() or controller_1.buttonA.pressing():
        skills = True
        controller_1.rumble('-.-.-.')
        print_all('SKILLS MODE')
        orientation.set_heading(270, DEGREES)
        matchload_setup()
    # cp controls
    controller_1.screen.clear_screen()
    controller_1.buttonL2.pressed(cp.wings)
    controller_1.buttonR2.pressed(cp.wings)

    controller_1.buttonL1.pressed(lambda: cp.intake(None if cp.intake_value == FORWARD else FORWARD))
    controller_1.buttonR1.pressed(lambda: cp.intake(None if cp.intake_value == REVERSE else REVERSE))

    controller_1.buttonX.pressed(cp.catapult)
    controller_1.buttonB.released(catapult.stop)

    controller_1.buttonUp.pressed(lambda: endg.set(True))
    controller_1.buttonDown.pressed(lambda: endg.set(False))
    # ready, set, drive!
    while True:
        wait(10, MSEC)
        # dt controls
        straight_speed = controller_1.axis3.position()
        turn_speed = 0.5*controller_1.axis1.position()
        l = straight_speed + turn_speed
        r = straight_speed - turn_speed
        dt_left.spin((FORWARD if l >= 0 else REVERSE), abs(l)*12/100, VOLT)
        dt_right.spin((FORWARD if r >= 0 else REVERSE), abs(r)*12/100, VOLT)
        # logging
        logs = [
            "time: %d:%g" % (math.floor(brain.timer.time(SECONDS)/60), int(brain.timer.time(SECONDS) % 60)),
            "dt: %d %d" % (dt_left.temperature(PERCENT), dt_right.temperature(PERCENT)),
            "cata: %d" % catapult.temperature(PERCENT),
        ]
        controller_1.screen.set_cursor(1 + log_n % 3, 1)
        controller_1.screen.print(logs[log_n % 3])
        log_n += 1
        # hang mode
        if brain.timer.time(SECONDS) > 50 and skills and hang_n > 0:
            if hang_n == 10:
                endg.set(True)
            print_all('HANG MODE')
            hang_n -= 1
            if hang_n == 0:
                controller_1.screen.clear_screen()
def autonomous():
    mangle = 20
    def matchload(start=False):
        if start:
            orientation.set_heading(270)
        initial_time = brain.timer.time(SECONDS)
        matchload_setup(mangle=mangle)
        while brain.timer.time(SECONDS)-initial_time < 27:
            wait(50, MSEC)
        controller_1.rumble('_._._._._.')
        wait(3, SECONDS)
        cp.catapult(None)
        print(orientation.heading(DEGREES))
    def traverse(start=False):
        if start:
            orientation.set_heading(180-mangle)
        # switch to offensive zone
        dt.drive4(-3)
        dt.turn2(45)
        dt.drive4(18)
        dt.turn2(180)
        dt.drive4(-33-24-3)#-6
    def pushes(start=False):
        """
        NEW AUTON SKILLS, starting after match loading (26s)
        Offshoot -> reduce matchload angle
        * go over barrier straight -> arc into elevation side push (11 sec)
        * transition phase (4 sec)
        * same center arcs - last center straight changed? (11 sec)
        """
        if start:
            orientation.set_heading(180, DEGREES)
        # arc back for right side push
        #dt.drive4(-6)
        dt.arc(20, REVERSE, RIGHT, 1.5, speed=65)
        dt.drive4(5)
        # reverse back & set up for next arc
        sstime = brain.timer.time(MSEC)
        dt.turn2(-15)
        dt.drive4(-13)
        dt.turn2(30)
        print(brain.timer.time(MSEC)-sstime)
        # this is an absolutely BEAUTIFUL arc
        cp.wings(OPEN)
        dt.arc(11, REVERSE, LEFT, 1.3, speed=65)
        dt.drive4(-10)
        cp.wings(CLOSE)
        dt.drive4(5)
        dt.turn2(180)
        # traverse to next push
        dt.drive4(24)
        dt.turn2(90)
        dt.drive4(-45)
        # side angled push
        dt.turn2(180+27)
        cp.wings(OPEN)
        dt.drive4(-30)
        dt.turn2(145)
        cp.wings(CLOSE)
        # let's do this! 
        dt.drive4(20)
        cp.wings(OPEN)
        dt.turn2(180)
        dt.drive4(-30)
        cp.wings(CLOSE)
        dt.drive4(15, speed=200)
        # INSERT FINAL PUSH HERE

    sequence = [matchload, traverse, pushes]
    test_sequence = [pushes]

    dt_left.set_stopping(HOLD)
    dt_right.set_stopping(HOLD)
    brain.timer.clear()
    first = True
    for s in sequence:
        s(first)
        first = False
    print(brain.timer.time(SECONDS))
    brain.screen.print(brain.timer.time(SECONDS))

print("\033[2J") # clear console
competition = Competition(driver_control, autonomous)