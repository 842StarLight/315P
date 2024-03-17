from vex import *
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
brain.screen.set_cursor(1, 1)
brain.screen.print("time")

brain.screen.set_cursor(2, 1)
brain.screen.print("dt temps")

brain.screen.set_cursor(3, 1)
brain.screen.print("cata temp")
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
        catapult.set_velocity(cata_speed, RPM)
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
        self.dt_time = 0
    def drive4(self, inches, speed=200, timeout=1):
        first = brain.timer.time(SECONDS)
        '''
        ### I think this is the most used dt auton function; it simply drives a number of inches.
        We are planning to eventually add a PID loop or something here later on.
        A custom timeout solution end the command and moves on if the robot gets stuck or takes too long.

        #### Arguments:
            inches: how far to drive, in inches forward. Negative values are accepted and interpreted as inches backwards.
            speed (>0, <100): speed of both sides, in percent
            timeout (>0): DEPRECATED - do not use, full stop.
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
        # record initial time; var to check if dt has accelerated
        initial_time = brain.timer.time(SECONDS)
        is_started = False
        running = True
        # main timeout loop
        while running:
            wait(10, MSEC)
            # is_started makes sure we don't stop during accel
            if dt_left.velocity(PERCENT) > 5:
                is_started = True
            # main checks
            if brain.timer.time(SECONDS)-initial_time >= timeout:
                print("drive4 timed out: dist", inches, "timeout", timeout)
                running = False
            elif is_started and dt_left.velocity(PERCENT) == 0: # UTB if
                # recalculate as a form of error filtering
                wait(40, MSEC)
                running = not (dt_left.velocity(PERCENT) == 0)
        # stop
        dt_left.stop()
        dt_right.stop()
        # logging
        #print("drive4/done", inches, "in, took", brain.timer.time(SECONDS)-initial_time, 'sec')
        self.dt_time += brain.timer.time(SECONDS)-first
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
        initial_time = brain.timer.time(SECONDS)
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
        #print('turn2/done', angle_unmodded, 'deg', 'took', brain.timer.time(SECONDS)-initial_time, 'sec')
    def arc(self, rad, head, side, duration, speed=65, finish=True):
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
        if not finish:
            print(left*sconst, right*sconst)
            return None
        # wait, then stop
        wait(duration, SECONDS)
        dt_left.stop()
        dt_right.stop()
        # in case you need it - here's a simple fn to determine duration of an arc
        '''
        def test(rad):
            dt.arc(..., finish=False)
            initial_time = brain.timer.time(SECONDS)
            while not controller_1.buttonB.pressing():
                wait(1, MSEC)
            dt_left.stop()
            dt_right.stop()
            print(brain.timer.time(SECONDS)-initial_time)
        '''
        
       
dt = Drivetrain(60/36, 3.25, 11)
# (4/5)(motor rpm) = matchloads per minutes
cp = Components(165) # 144 triballs per minute, 2.4 per sec
# helper functions
def matchload_setup(mangle=20):
    dt.drive4(12)
    dt.turn2(180-mangle)
    dt.drive4(12)
    dt.drive4(2.5, speed=25)
    # match loading
    cp.catapult()
    dt.turn2(180-mangle)
# driver control
def driver_control():
    # core archie setup
    def print_all(msg):
        brain.screen.clear_screen()
        for i in range(3):
            brain.screen.set_cursor(i+1,1)
            brain.screen.print(msg)
    log_n = 0 # a var to keep track of segregated logging
    brain.timer.clear()
    # skills mode + matchloading setup
    hang_n = 10
    skills = False
    if controller_1.buttonY.pressing() or controller_1.buttonA.pressing():
        skills = True
        print_all('SKILLS MODE')
        orientation.set_heading(270, DEGREES)
        matchload_setup()
    # cp controls
    brain.screen.clear_screen()
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
        brain.screen.set_cursor(1 + log_n % 3, 1)
        brain.screen.print(logs[log_n % 3])
        log_n += 1
        # hang mode
        if brain.timer.time(SECONDS) > 50 and skills and hang_n > 0:
            if hang_n == 10:
                endg.set(True)
            print_all('HANG MODE')
            hang_n -= 1
            if hang_n == 0:
                brain.screen.clear_screen()
def autonomous():
    mangle = 20
    def matchload(start=False):
        if start:
            orientation.set_heading(270)
        matchload_setup(mangle=mangle)
        loads = 45
        load_const = (45/37)*(5/4) # multiply desired # of matchloads by (45/37)*(5/4) - 5/4 due to gear ratio, 45/37 for error correction?
        catapult.spin_for(FORWARD, 45*load_const, TURNS, wait=False) 
        while catapult.is_spinning():
            if catapult.position(TURNS) > load_const*(loads-3):
                intake.spin(FORWARD)
            wait(50, MSEC)
    def traverse(start=False):
        intake.stop()
        if start:
            orientation.set_heading(180-mangle)
        # switch to offensive zone
        dt.drive4(-3)
        dt.turn2(45)
        dt.drive4(21)
        dt.turn2(180)
        dt.drive4(-33-24-3, timeout=3)
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
            dt.drive4(-3)
        # arc back for right side push
        """
        AUTON SKILLS, ICC official run: stops at the beginning of this fn
        Possible options:
            Not timeouting??? -> add back timeout clause in dt.drive4
            Sequence is wrong??? -> fix it
        """
        dt.arc(20, REVERSE, RIGHT, 1.2, speed=65)
        dt.drive4(-10)
        dt.drive4(5)
        dt.drive4(-5)
        dt.drive4(5)
        # dive into the heart of the enemy - i mean offensive zone
        dt.turn2(0)
        dt.drive4(-10)
        dt.turn2(15)
        dt.drive4(-32)
        # angle push 1
        dt.turn2(155)
        dt.drive4(-24)
        # traverse to next push
        dt.drive4(7)
        dt.turn2(180)
        dt.drive4(12)
        dt.turn2(-90)
        dt.drive4(46, timeout=3)
        # angle push 2
        dt.turn2(33)
        dt.drive4(30)
        dt.turn2(33)
        # center push 1
        dt.arc(9.5, REVERSE, RIGHT, 1.5)
        dt.turn2(180)
        cp.wings(OPEN)
        dt.drive4(-20)
        dt.drive4(10)
        dt.drive4(-18, speed=200)
        cp.wings(CLOSE)
        # center push 2 at a slightly diff angle
        dt.drive4(18, speed=200)
        dt.turn2(160)
        cp.wings(OPEN)
        dt.drive4(-26)
        dt.drive4(7, speed=200)
        cp.wings(CLOSE)
        # side push 2
        dt.turn2(93)
        dt.drive4(-12)
        cp.wings(OPEN)
        dt.arc(9, REVERSE, LEFT, 1.5, speed=65) # reuse arc from first side push
        cp.wings(CLOSE)
        dt.drive4(-5, speed=200)

    sequence = [matchload, traverse, pushes]
    # test_sequence = [pushes]

    dt_left.set_stopping(HOLD)
    dt_right.set_stopping(HOLD)
    brain.timer.clear()
    first = True
    for s in sequence:
        s(first)
        first = False
    # print(brain.timer.time(SECONDS))
    brain.screen.print(brain.timer.time(SECONDS))

print("\033[2J") # clear console
competition = Competition(driver_control, autonomous)