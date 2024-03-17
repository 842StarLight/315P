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
# sensors - MAIN PART HERE
orientation = Inertial(Ports.PORT10)
orientation.calibrate()
while orientation.is_calibrating():
    wait(10, MSEC)

# drivetrain
dt_right = MotorGroup(Motor(Ports.PORT1, GearSetting.RATIO_18_1, True))
dt_left = MotorGroup(Motor(Ports.PORT4, GearSetting.RATIO_18_1, False))
drive = SmartDrive(dt_left, dt_right, orientation)
dt_left.set_stopping(BRAKE) 
dt_right.set_stopping(BRAKE)
class PID:
    def __init__(self, sample_rate, initial_error, consts): # sample_rate in seconds
        self.sample_rate = sample_rate
        self.kP = consts[0]
        self.kI = consts[1]
        self.kD = consts[2]

        self.integral = 0.0  # Initialize integral for error accumulation
        self.prev_error = initial_error  # Initialize previous error for derivative calculation

    def update(self, error):
        # Proportional term
        proportional = self.kP * error

        # Integral term
        self.integral += error*self.sample_rate  # Accumulate error over time
        integral = self.kI * self.integral

        # Derivative term
        derivative = self.kD * (error - self.prev_error) / self.sample_rate
        self.prev_error = error  # Update previous error for next calculation

        # Calculate intended velocity
        velocity = proportional + integral + derivative

        return velocity

class Drivetrain:
    def __init__(self, gear_ratio, wheel_diameter, wheelbase):
        self.gear_ratio = gear_ratio
        self.wheel_diameter = wheel_diameter
        self.wheelbase = wheelbase
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
        # consts
        dt_const = self.gear_ratio/(math.pi*self.wheel_diameter)
        #12.5in per revolution
        turns = dt_const * inches
        # core spin
        dt_left.spin_for(FORWARD, dt_const*inches, TURNS, wait=False)
        dt_right.spin_for(FORWARD, dt_const*inches, TURNS, wait=False)
        # record initial time; var to check if dt has accelerated
        initial_time = brain.timer.time(SECONDS)
        running = True
        # wait until started
        while dt_left.velocity(PERCENT) <= 5:
            wait(10, MSEC)
        # main loop
        while running:
            wait(10, MSEC)
            if brain.timer.time(SECONDS) - initial_time > timeout:
                running = False
            elif dt_left.velocity(PERCENT) == 0:
                # recalculate as a form of error filtering
                wait(40, MSEC)
                running = not (dt_left.velocity(PERCENT) == 0)
        # stop
        drive.stop()
        # logging
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
        dt_right.spin(FORWARD, right*sconst*12/100, VoltageUnits.VOLT) # type: ignore
        dt_left.spin(FORWARD, left*sconst*12/100, VoltageUnits.VOLT) # type: ignore
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

# voltage utility
def clamp_volt(volt):
    if volt > 0:
        volt = min(12, volt)
        volt = max(3, volt)
    elif volt < 0:
        volt = max(-12, volt)
        volt = min(-3, volt)
    return volt
# driver control
def driver_control(time=10**10):
    # EXTREMELY dumbed-down
    initial_time = brain.timer.time(SECONDS)
    while brain.timer.time(SECONDS) - initial_time <= time:
        wait(2, MSEC)
        # dt controls
        straight_speed = controller_1.axis3.position()
        turn_speed = 0.75*controller_1.axis1.position()
        l = straight_speed + turn_speed
        r = straight_speed - turn_speed
        dt_left.spin((FORWARD if l >= 0 else REVERSE), abs(l)*12/100, VoltageUnits.VOLT) # type: ignore
        dt_right.spin((FORWARD if r >= 0 else REVERSE), abs(r)*12/100, VoltageUnits.VOLT) # type: ignore
    dt_left.stop()
    dt_right.stop()
# pid turn function
def pid_turn(angle_unmodded, c=(2, 0, 0), logs=False):
    # error calc fn
    angle = angle_unmodded % 360
    def error_calc(d):
        abs_err = (angle-d)%360 # 0 to 360
        norm_err = ((-abs_err % -180)+(180 if abs_err>=180 else 0)) # -180 to 180
        return norm_err/180 # -1 to 1
    # basic vars & controller setup
    wait_time = 0.01
    err = error_calc(orientation.heading(DEGREES))
    control = PID(wait_time, err, c)
    # logs stuff
    vels = []
    itime = brain.timer.time(SECONDS)
    while (abs(err) >= 0.01 or abs(dt_left.velocity(PERCENT)) >= 5) and brain.timer.time(SECONDS)-itime <= 5:
        # speed calcs
        err = error_calc(orientation.heading(DEGREES))
        target = control.update(err)*100
        volt = clamp_volt(target*12/100)
        # spin ahoy!
        dt_left.spin(REVERSE, volt, VoltageUnits.VOLT) # type: ignore
        dt_right.spin(FORWARD, volt, VoltageUnits.VOLT) # type: ignore
        # vels
        vels.append((target, -dt_left.velocity(PERCENT) ))
        # 10 msec wait
        wait(wait_time*1000, MSEC)
    dt_left.stop()
    dt_right.stop()
    if logs:
        print('\n'.join([' '.join([str(j) for j in i]) for i in vels]))
class TurnTest: # comprehensive class in which we can write code to test a variety of turn functions
    def __init__(self, fn_wrapper):
        # fn_wrapper - a function instance which takes in an angle then turns to it
        self.fn = fn_wrapper
    def comprehensive(self, to=180):
        times = []
        for degrees in range(10, to+1, 10):
            wait(2, SECONDS)

            orientation.set_heading(0, DEGREES)

            brain.timer.clear()
            self.fn(degrees)
            times.append(brain.timer.time(SECONDS)/2)
            brain.timer.clear()
            self.fn(0)
            times[-1] += brain.timer.time(SECONDS)/2

        print(', '.join([str(round(i,3)) for i in times]))
    def single(self, a=90):
        orientation.set_heading(0, DEGREES)
        brain.timer.clear()
        self.fn(a)
        print(brain.timer.time(MSEC))
        wait(0.5, SECONDS)
        print(orientation.heading(DEGREES))
def turn_wrap(*consts, log=False):
    def wrapper(a):
        pid_turn(a, c=consts, logs=log)
    return TurnTest(wrapper)
# pid drive
def pid_drive(inches, c=(2, 0, 0), speed=100, timeout=5, log=False):
    # reset
    dt_left.reset_position()
    dt_right.reset_position()
    initial_time = brain.timer.time(SECONDS)
    # error calc
    dt_const = dt.gear_ratio/(math.pi*dt.wheel_diameter)
    def error_calc(encoder_val):
        # convert rotations to inches
        turns = inches*dt_const
        return (turns-encoder_val)/abs(turns)
    # set up error & controllers
    wait_time = 0.01
    left_err = error_calc(dt_left.position(TURNS)/0.7233334)
    right_err = error_calc(dt_right.position(TURNS))
    left_control = PID(wait_time, left_err, c)
    right_control = PID(wait_time, right_err, c)
    buffer = ""
    while (
        (abs(left_err) >= 0.01) or (abs(right_err) >= 0.01) 
        #or
        #(dt_left.velocity(PERCENT) > 5) or (dt_right.velocity(PERCENT) > 5)
    ) and brain.timer.time(SECONDS)-initial_time <= timeout:
        left_err = error_calc(dt_left.position(TURNS)/0.7233334)
        right_err = error_calc(dt_right.position(TURNS))
        # calculate target velocities
        """left_target = clamp_volt(
            left_control.update(
                left_err
            ) * speed * 12/100
        )
        right_target = clamp_volt(
            right_control.update(
                right_err
            )  * speed * 12/100
        )"""
        left_target = left_control.update(left_err) * speed
        right_target = right_control.update(right_err)  * speed
        buffer += str(left_target) + " " + str(left_err) + " " + str(dt_left.velocity(PERCENT)) + "\n"
        # spin ahoy!
        #dt_left.spin(FORWARD if left_target >= 0 else REVERSE, left_target, VoltageUnits.VOLT) # type: ignore
        #dt_right.spin(FORWARD if right_target >= 0 else REVERSE, right_target, VoltageUnits.VOLT) # type: ignore
        dt_left.spin(FORWARD, left_target, PERCENT) # type: ignore
        dt_right.spin(FORWARD, right_target, PERCENT) # type: ignore
        # 10 msec wait
        wait(wait_time*1000, MSEC)
    dt_left.stop()
    dt_right.stop()
    if log:
        print(buffer)
class DriveTest: # comprehensive class in which we can write code to test a variety of drive functions
    def __init__(self, fn_wrapper):
        # fn_wrapper - a function instance which takes in an distance then drives to it
        self.fn = fn_wrapper
    # def comprehensive(self): - how would we implement this?
    def single(self, dist=24):
        orientation.set_heading(0, DEGREES)
        brain.timer.clear()
        self.fn(dist)
        print(brain.timer.time(MSEC))
def drive_wrap(*consts, log=False, speed=100):
    def wrapper(dist):
        pid_drive(dist, c=consts, speed=speed, log=log)
    return DriveTest(wrapper)

"""
EXPERIMENTS:
drive4: switch to PID
 - go-to: drive_wrap(2, .5, .5, log=True).single()
 - figure it the hell out!
FINAL turn2: switch to PD
 - constants kP = 3, kI = 0, kD = 0.2 go-to: turn_wrap(2, 0.5, 0.5).single()
 - test results below
 - integrate back into drivetrain class and put testing code into separate file
"""
"""
Test results (angles from 10 to 180, incrementing by 10; average of two turns to get time per angle) (pid constants 3, 0, 0.2)
turn_pid: 0.28, 0.46, 0.47, 0.5, 0.59, 0.58, 0.54, 0.515, 0.565, 0.565, 0.57, 0.64, 0.61, 0.67, 1.221, 0.73, 0.95, 0.805
dt.turn2: 0.545, 0.56, 0.6, 0.695, 0.74, 0.985, 1.065, 1.205, 1.165, 1.235, 1.23, 1.271, 1.335, 1.375, 1.39, 1.415, 1.465, 1.66
"""