from vex import *
import time
import math
import random
# brain & controller
brain = Brain()
controller_1 = Controller(PRIMARY)
# logging on the brain
# TODO: add auton selector
class BrainScreen:
    def __init__(self):
        brain.screen.set_font(FontType.PROP60)

        brain.screen.set_cursor(1, 1)
        brain.screen.print("time")

        brain.screen.set_cursor(2, 1)
        brain.screen.print("dt temps")

        brain.screen.set_cursor(3, 1)
        brain.screen.print("cata temp")
        self.log_n = 0
    def driver_log(self, logs):
        brain.screen.set_cursor(1 + self.log_n % 3, 1)
        brain.screen.print(logs[self.log_n % 3])
        self.log_n += 1
    def print_all(self, msg):
        brain.screen.clear_screen()
        for i in range(3):
            brain.screen.set_cursor(i+1,1)
            brain.screen.print(msg)
    def autonomous_selector(self, options=list[str]):
        raise NameError('Autonomous selector not yet implemented')
screen = BrainScreen()
# a core feedback loop; a reusable controller that we use in two of our crucial architectural functions
class PID:
    def __init__(self, sample_rate, initial_error, consts): # sample_rate in seconds
        self.sample_rate = sample_rate
        self.kP = consts[0]
        self.kI = consts[1]
        self.kD = consts[2]

        self.integral = 0.0  # Initialize integral for error accumulation
        self.prev_error = initial_error  # Initialize previous error for derivative calculation
    # update method
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
    
    # voltage utility
    @staticmethod
    def clamp_volt(volt):
        if volt > 0:
            volt = min(12, volt)
            volt = max(3, volt)
        elif volt < 0:
            volt = max(-12, volt)
            volt = min(-3, volt)
        return volt
# the drivetrain class is completely custom, designed for simplicity, speed, & accuracy
class DrivetrainConfig:
    def __init__(
            self,
            left_ports:list[int],
            left_bool:bool,
            right_ports:list[int],
            right_bool:bool,
            cartridge:GearSetting.GearSetting,
            gear_ratio:float,
            wheel_diameter:float,
            wheelbase:float
    ):
        self.useful_ratio = 1/gear_ratio
        self.gear_ratio = gear_ratio
        self.left_group = MotorGroup(*[Motor(port, cartridge, left_bool) for port in left_ports])
        self.right_group  = MotorGroup(*[Motor(port, cartridge, right_bool) for port in right_ports])
        self.cartridge = cartridge
        self.wheel_diameter = wheel_diameter
        self.wheelbase = wheelbase
class Drivetrain(SmartDrive):
    def __init__(self, config: DrivetrainConfig, inertial_port: int):
        self.config = config
        self.orientation = Inertial(inertial_port)
        self.left = config.left_group
        self.right = config.right_group
        
        super().__init__(
            config.left_group,
            config.right_group,
            self.orientation,
            wheelTravel=config.wheel_diameter*math.pi,
            wheelBase=config.wheelbase,
            units=INCHES,
            externalGearRatio=config.gear_ratio
        )

        self.set_stopping(BRAKE)

        self.orientation.calibrate()
        while self.orientation.is_calibrating():
            wait(10, MSEC)
    def drive4(self, inches:float, speed:int=200, timeout:float=1):
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
        self.drive_for(FORWARD if inches > 0 else REVERSE, abs(inches), INCHES, speed, PERCENT, wait=False)
        # record initial time; var to check if dt has accelerated
        initial_time = brain.timer.time(SECONDS)
        running = True
        # wait until started
        while self.left.velocity(PERCENT) <= 5:
            wait(10, MSEC)
        # main loop
        while running:
            wait(10, MSEC)
            if brain.timer.time(SECONDS) - initial_time > timeout:
                running = False
            elif self.left.velocity(PERCENT) == 0:
                # recalculate as a form of error filtering
                wait(40, MSEC)
                running = not (self.left.velocity(PERCENT) == 0)
                print(running)
        # stop  
        self.stop()
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
        # set up
        angle = angle_unmodded % 360
        initial_time = brain.timer.time(SECONDS)
        h = self.orientation.heading(DEGREES)
        # main loop
        while abs(angle - h) % 360 > tolerance:
            # calculations
            h = self.orientation.heading(DEGREES)
            vel = abs((angle - h + 180) % 360 - 180) * speed * 2 / 180 + 3
            # action!
            self.turn(RIGHT if (angle - h + 180) % 360 - 180 > 0 else LEFT, vel, PERCENT)
            # wait
            wait(10, MSEC)
        # stop dt
        self.stop()
        # rerun if drift; logging
        if abs(angle - self.orientation.heading(DEGREES)) % 360 > tolerance:
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
        left = ahead*(rad+aside*self.config.wheelbase/2) # the parenthesized portion is the abs value
        right = ahead*(rad-aside*self.config.wheelbase/2) # of the circumference of the side's circle
        # velocities
        sconst = speed/(abs(left)/2+abs(right)/2) # to add the speed factor
        # and away she goes!
        self.left.spin(FORWARD, left*sconst*12/100, VoltageUnits.VOLT) # type: ignore
        self.right.spin(FORWARD, right*sconst*12/100, VoltageUnits.VOLT) # type: ignore
        if not finish:
            return None
        # wait, then stop
        wait(duration, SECONDS)
        self.stop()
        # logging
        #print('fastarc', rad, head, side, duration, speed)
        # in case you need it - here's a simple fn to determine duration of an arc
        '''
        def test(rad):
            dt.arc(..., finish=False)
            initial_time = brain.timer.time(SECONDS)
            while not controller_1.buttonB.pressing():
                wait(1, MSEC)
            dt.stop()
            print(brain.timer.time(SECONDS)-initial_time)
        '''  
config = DrivetrainConfig(
    [Ports.PORT20, Ports.PORT19, Ports.PORT18], True,
    [Ports.PORT11, Ports.PORT12, Ports.PORT13], False,
    cartridge=GearSetting.RATIO_6_1, gear_ratio=36/48, wheel_diameter=3.25, wheelbase=11
)
dt = Drivetrain(
    config,
    1
)
# pneumatics
back_wings = Pneumatics(brain.three_wire_port.a)
endgame = Pneumatics(brain.three_wire_port.b)
front_wings = Pneumatics(brain.three_wire_port.c)

# intake
intake_motor = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
def intake(direction):
        if intake_motor.direction == DirectionType.UNDEFINED:
            intake_motor.stop()
        else:
            intake_motor.spin(direction)
intake_motor.set_velocity(200, PERCENT)
# cata - in an iffy position right now, since we are planning to reimplement using limit switch
catapult = Motor(Ports.PORT9, GearSetting.RATIO_18_1, False)
catapult.set_velocity(165, RPM) # 144 triballs per minute, 2.4 per sec #  (4/5)(motor rpm) = matchloads per minutes

# advanced, high-level class for things that aren't directly related to components or drivetrain
class Advanced:
    def __init__(self, distance_port:int):
        self.distance_sensor = Distance(distance_port)
    def find_triball(self, color: str, angle, tol_angle = 5, timeout = 2):
        dt.turn2(angle-(tol_angle-1))
        # next
        dt.left.spin(FORWARD, 3, PERCENT)
        dt.right.spin(REVERSE, 3, PERCENT)
        # loop
        running = True
        initial_time = brain.timer.time(SECONDS)
        while running:
            if not ( (angle-tol_angle) % 360 < dt.orientation.heading(DEGREES) < (angle + tol_angle) % 360 ):
                running = False
            if brain.timer.time(SECONDS) - initial_time > timeout:
                running = False
            if self.distance_sensor.object_size() == ObjectSizeType.MEDIUM:
                running = False
         # stop all
        dt.left.stop()
        dt.right.stop()
    # helper func
    def matchload_setup(self, mangle=20):
        dt.drive4(12)
        dt.turn2(180-mangle)
        dt.drive4(12)
        dt.drive4(2.5, speed=25)
        # match loading
        catapult.spin(FORWARD)
        dt.turn2(180-mangle)
advanced = Advanced(distance_port = Ports.PORT2)
# driver control
def driver_control():
    # clear timer
    brain.timer.clear()
    # skills mode + matchloading setup
    hang_n = 10
    skills = False
    if controller_1.buttonY.pressing() or controller_1.buttonA.pressing():
        skills = True
        screen.print_all('SKILLS MODE')
        dt.orientation.set_heading(270, DEGREES)
        advanced.matchload_setup()
    # cp controls
    brain.screen.clear_screen()
    controller_1.buttonL2.pressed(lambda: front_wings.close() if front_wings.value else front_wings.open())
    controller_1.buttonL2.pressed(lambda: back_wings.close() if back_wings.value else back_wings.open())

    controller_1.buttonL1.pressed(lambda: intake(None if intake_motor.direction == FORWARD else FORWARD))
    controller_1.buttonR1.pressed(lambda: intake(None if intake_motor.direction == REVERSE else REVERSE))

    controller_1.buttonX.pressed(lambda: catapult.spin(FORWARD)) # will eventually phase out these controls
    controller_1.buttonB.released(catapult.stop) # in favor of automatic matchloading thru limit switch :)

    controller_1.buttonUp.pressed(endgame.open)
    controller_1.buttonDown.pressed(endgame.close)
    # ready, set, drive!
    while True:
        wait(10, MSEC)
        # dt controls
        straight_speed = controller_1.axis3.position()
        turn_speed = controller_1.axis1.position()
        l = straight_speed + turn_speed
        r = straight_speed - turn_speed
        
        dt.left.spin((FORWARD if l >= 0 else REVERSE), abs(l)*12/100, VOLT) # type: ignore
        dt.right.spin((FORWARD if r >= 0 else REVERSE), abs(r)*12/100, VOLT) # type: ignore
        # logging
        logs = [
            "time: %d:%g" % (math.floor(brain.timer.time(SECONDS)/60), int(brain.timer.time(SECONDS) % 60)),
            "dt: %d %d" % (dt.left.temperature(PERCENT), dt.right.temperature(PERCENT)), # type: ignore
            "cata: %d" % catapult.temperature(PERCENT),
        ]
        screen.driver_log(logs)
        # hang mode
        if brain.timer.time(SECONDS) > 50 and skills and hang_n > 0:
            if hang_n == 10:
                endgame.open()
            screen.print_all('HANG MODE')
            hang_n -= 1
            if hang_n == 0:
                brain.screen.clear_screen()
# autons
def autonomous():
    pass
# auton selector here

competition = Competition(driver_control, autonomous) 