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
def brain_setup():
    brain.screen.set_font(FontType.PROP60)

    brain.screen.set_cursor(1, 1)
    brain.screen.print("time")

    brain.screen.set_cursor(2, 1)
    brain.screen.print("dt temps")

    brain.screen.set_cursor(3, 1)
    brain.screen.print("cata temp")
brain_setup()
# sensors
distance = Distance(Ports.PORT2)

orientation = Inertial(Ports.PORT1)
orientation.calibrate()
while orientation.is_calibrating():
    wait(10, MSEC)
# drivetrain
dt_right = MotorGroup(
    Motor(Ports.PORT11, GearSetting.RATIO_6_1, False),
    Motor(Ports.PORT12, GearSetting.RATIO_6_1, False),
    Motor(Ports.PORT13, GearSetting.RATIO_6_1, False)
)
dt_left = MotorGroup(
    Motor(Ports.PORT20, GearSetting.RATIO_6_1, True),
    Motor(Ports.PORT19, GearSetting.RATIO_6_1, True),
    Motor(Ports.PORT18, GearSetting.RATIO_6_1, True)
)
dt_left.set_stopping(BRAKE) 
dt_right.set_stopping(BRAKE)
# components
intake = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
catapult = Motor(Ports.PORT9, GearSetting.RATIO_18_1, False)
# solenoids
wings = DigitalOut(brain.three_wire_port.a)
endg = DigitalOut(brain.three_wire_port.b)
# voltage utility 
def clamp_volt(volt):
    if volt > 0:
        volt = min(12, volt)
        volt = max(3, volt)
    elif volt < 0:
        volt = max(-12, volt)
        volt = min(-3, volt)
    return volt
# the core feedback loop; a reusable controller that we use in two of our crucial architectural functions
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
# the drivetrain class is completely custom, designed for simplicity, speed, & accuracy
class Drivetrain:
    def __init__(self, gear_ratio, wheel_diameter, wheelbase):
        self.gear_ratio = gear_ratio
        self.wheel_diameter = wheel_diameter
        self.wheelbase = wheelbase
    def drive4(self, inches, speed=100, timeout=1.5):
        first = brain.timer.time(SECONDS)
        '''
        ### I think this is the most used dt auton function; it simply drives a number of inches.
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
            elif brain.timer.time(SECONDS)-initial_time >= timeout:
                running = False
            elif is_started and dt_left.velocity(PERCENT) == 0:
                # recalculate as a form of error filtering
                wait(40, MSEC)
                running = not (dt_left.velocity(PERCENT) == 0)

        # stop
        dt_left.stop()
        dt_right.stop()
        # logging
    def _drive4(self, inches:float, speed:int=200, timeout:float=1):
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
        # ~12.5in per revolution
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
                print('TIMEOUT')
                running = False
            elif dt_right.velocity(PERCENT) == 0:
                print('YAS VELOCITY')
                # recalculate as a form of error filtering
                wait(40, MSEC)
                running = not (dt_left.velocity(PERCENT) == 0)
                print(running)
        # stop  
        dt_left.stop()
        dt_right.stop()
        # logging
    def turn2(self, angle_unmodded, speed=30, tolerance = 1):
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
        while abs(angle - h) % 360 > tolerance:
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
        if abs(angle - orientation.heading(DEGREES)) % 360 > tolerance:
            self.turn2(angle, speed=10)
        #print('turn2/done', angle_unmodded, 'deg', 'took', brain.timer.time(SECONDS)-initial_time, 'sec')
    def arc(self, rad, head, side, duration, speed=40, finish=True):
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
    def old_arc(self, l, r, duration):
        dt_left.spin(FORWARD, l, PERCENT)
        dt_right.spin(FORWARD, r, PERCENT)
        wait(duration, SECONDS)
        dt_left.stop()
        dt_right.stop()  
dt = Drivetrain(48/36, 3.25, 11) 
# components class that wraps all of the non-drivetrain functions into one simple namespace
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
cp = Components(165) # 144 triballs per minute, 2.4 per sec #  (4/5)(motor rpm) = matchloads per minutes 
# advanced, high-level class for things that aren't directly related to components or drivetrain
class Advanced:
    def __init__(self, distance: Distance):
        self.distance_sensor = distance
    def find_triball(self, color: str, angle, tol_angle = 5, timeout = 2):
        dt.turn2(angle-(tol_angle-1))
        print('dsgsdgkjfdhglkj')
        # next
        dt_left.spin(FORWARD, 3, PERCENT)
        dt_right.spin(REVERSE, 3, PERCENT)
        # loop
        running = True
        initial_time = brain.timer.time(SECONDS)
        while running:
            if not ( (angle-tol_angle) % 360 < orientation.heading(DEGREES) < (angle + tol_angle) % 360 ):
                print('angled out')
                running = False
            if brain.timer.time(SECONDS) - initial_time > timeout:
                print('timed out')
                running = False
            if self.distance_sensor.object_size() == ObjectSizeType.MEDIUM:
                print('checked out')
                running = False
         # stop all
        dt_left.stop()
        dt_right.stop()
    # helper func
    def matchload_setup(self, mangle=20):
        dt.drive4(12)
        dt.turn2(180-mangle)
        dt.drive4(12)
        dt.drive4(2.5, speed=25)
        # match loading
        cp.catapult()
        dt.turn2(180-mangle)
advanced = Advanced(distance = Distance(Ports.PORT2))
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
        advanced.matchload_setup()
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
        dt_left.spin((FORWARD if l >= 0 else REVERSE), abs(l)*12/100, VOLT) # type: ignore
        dt_right.spin((FORWARD if r >= 0 else REVERSE), abs(r)*12/100, VOLT) # type: ignore # type: ignore
        # logging
        logs = [
            "time: %d:%g" % (math.floor(brain.timer.time(SECONDS)/60), int(brain.timer.time(SECONDS) % 60)),
            "dt: %d %d" % (dt_left.temperature(PERCENT), dt_right.temperature(PERCENT)), # type: ignore
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
    orientation.set_heading(180, DEGREES)
    initial_time = brain.timer.time(SECONDS)
    # elevation
    cp.intake(FORWARD)
    wait(300, MSEC)
    # get to matchload
    dt.drive4(-22)
    # arc, matchload
    dt.old_arc(-50, -30, 1)
    cp.wings(OPEN)
    dt.old_arc(-100,-60,1)

    # dt.arc(28, REVERSE, RIGHT, 0.7)
    # cp.wings()
    # # score!
    # dt.arc(28, REVERSE, RIGHT, 0.7)
    dt.drive4(-4)
    cp.wings(CLOSE)
    dt.drive4(4)
    # score elevation
    dt.turn2(-90)
    cp.intake(REVERSE)
    dt.drive4(8)
    # make sure the elevation is scored
    dt.drive4(-3)
    dt.drive4(3)
    # head to & intake less impossible
    dt.turn2(-90)
    dt.drive4(-10)

    dt.turn2(203)
    cp.intake(FORWARD)
    dt.drive4(40)
    advanced.find_triball('GREEN', 203, tol_angle=10)
    dt.drive4(7)

    # pseudo-arc smash
    dt.turn2(120)
    cp.wings()
    dt.drive4(-10)

    dt.turn2(180)
    dt.drive4(-20)

    # score less impossible
    dt.drive4(4)
    cp.wings()
    dt.turn2(0)

    cp.intake(REVERSE)
    dt.drive4(12)
    dt.drive4(-10)
    #
    print(brain.timer.time(SECONDS)-initial_time)
#autonomous()
#competition = Competition(driver_control, autonomous) 
dt.turn2(180)
#cp.intake(FORWARD)
#dt.drive4(40)
advanced.find_triball('GREEN', -135, tol_angle=10)
#dt.drive4(7)