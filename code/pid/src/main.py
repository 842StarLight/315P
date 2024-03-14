from vex import *
import urandom
import time
import math
import random
# brain
brain = Brain()
# controller
controller_1 = Controller(PRIMARY)

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

class Drivetrain:
    def __init__(self, gear_ratio, wheel_diameter, wheelbase):
        self.gear_ratio = gear_ratio
        self.wheel_diameter = wheel_diameter
        self.wheelbase = wheelbase
        self.dt_time = 0
    def old_turn2(self, angle_unmodded, speed=41):

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
            self.old_turn2(angle, speed=10)
        #print('turn2/done', angle_unmodded, 'deg', 'took', brain.timer.time(SECONDS)-initial_time, 'sec')
        
       
dt = Drivetrain(60/36, 3.25, 11)

orientation = Inertial(Ports.PORT1)
orientation.calibrate()
while orientation.is_calibrating():
    wait(10, MSEC)

def test_turn(fn):
    times = []
    for degrees in range(10, 190, 10):
        orientation.set_heading(0, DEGREES)\

        brain.timer.clear()
        fn(degrees)
        times.append(brain.timer.time(SECONDS)/2)

        brain.timer.clear()
        fn(0)
        times[-1] += brain.timer.time(SECONDS)/2
    print(', '.join([str(round(i,3)) for i in times]))

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

def pid_turn(angle_unmodded, c=(2, 0.5, 0.2)):
    # error calc fn
    angle = angle_unmodded % 360
    def error_calc(d):
        abs_err = (angle-d)%360 # 0 to 360
        norm_err = ((-abs_err % -180)+(180 if abs_err>180 else 0)) # -180 to 180
        return norm_err/180 # -1 to 1
    # basic vars
    wait_time = 0.01
    err = error_calc(orientation.heading(DEGREES))
    # set up controller
    control = PID(wait_time, err, c)

    while abs(err) >= 0.01 or abs(dt_left.velocity(PERCENT)) >= 5:
        # error
        err = error_calc(orientation.heading(DEGREES))
        vel = control.update(err)
        # spin ahoy!
        dt_left.spin(REVERSE, vel*50, PERCENT)
        dt_right.spin(FORWARD, vel*50, PERCENT)
        # 10 msec wait
        wait(wait_time*1000, MSEC)
    dt_left.stop()
    dt_right.stop()

def ttest(consts):
    orientation.set_heading(0, DEGREES)
    brain.timer.clear()
    pid_turn(90, c=consts)
    print(brain.timer.time(MSEC), orientation.heading(DEGREES))
#test_turn(dt.old_turn2)
"""
Set I and D to 0. Increase P until you get oscillation off the system. Then cut P in half (p = 1.8 -> 0.9)

increase I until you are returning to the set-point quick enough for your application. Too much integral gain can cause a lot of overshoot.

increase D until you’ve started to dampen your response as you approach the set point.
""" 
"""
old turn2: 0.695, 0.625, 0.83, 0.84, 0.75, 0.84, 0.89, 0.705, 0.875, 0.89, 1.0, 0.98, 1.045, 1.125, 1.045, 1.13, 0.71, 0.756
pid turn2:
"""