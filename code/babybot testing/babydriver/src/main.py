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
# sensors - MAIN PART HERE
# orientation = Inertial(Ports.PORT1)
# orientation.calibrate()
# while orientation.is_calibrating():
#     wait(10, MSEC)

# drivetrain
dt_left = MotorGroup(
    Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
)
dt_right = MotorGroup(
    Motor(Ports.PORT20, GearSetting.RATIO_18_1, False)
)
dt_left.set_stopping(BRAKE) 
dt_right.set_stopping(BRAKE)

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
        
       
dt = Drivetrain(1, 4, 11)
# driver control
def driver_control():
    # EXTREMELY dumbed-down
    while True:
        wait(10, MSEC)
        # dt controls
        straight_speed = controller_1.axis3.position()
        turn_speed = 0.5*controller_1.axis1.position()
        l = straight_speed + turn_speed
        r = straight_speed - turn_speed
        dt_left.spin((FORWARD if l >= 0 else REVERSE), abs(l)*12/100, VOLT)
        dt_right.spin((FORWARD if r >= 0 else REVERSE), abs(r)*12/100, VOLT)
def autonomous():
    pass
competition = Competition(driver_control, autonomous)