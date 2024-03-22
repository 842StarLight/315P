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
dt_left = MotorGroup(Motor(Ports.PORT2, GearSetting.RATIO_18_1, False))
dt_right = MotorGroup(Motor(Ports.PORT1, GearSetting.RATIO_18_1, True))

dt_left.set_stopping(BRAKE) 
dt_right.set_stopping(BRAKE)
# distance
distance = Distance(Ports.PORT6)

# START NEW CODE HERE

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
# voltage utility 
def clamp_volt(volt):
    if volt > 0:
        volt = min(12, volt)
        volt = max(3, volt)
    elif volt < 0:
        volt = max(-12, volt)
        volt = min(-3, volt)
    return volt
class Drivetrain:
    def __init__(self, gear_ratio, wheel_diameter, wheelbase):
        self.gear_ratio = gear_ratio
        self.wheel_diameter = wheel_diameter
        self.wheelbase = wheelbase
    def pid_drive_distance(self, inches, c=(3, 0, 0.2), speed=100, timeout=2):
        "PD controller that utilizes the distance sensor to have accurate drives."
        # reset
        dt_left.reset_position()
        dt_right.reset_position()
        initial_time = brain.timer.time(SECONDS)
        # error calc
        initial_dist = distance.object_distance(INCHES)
        def error_calc(raw_dist):
            return (inches - (raw_dist-initial_dist))/abs(inches)
        # set up error & controllers
        wait_time = 0.01
        err = error_calc(initial_dist)
        control = PID(wait_time, err, c)
        while abs(err) >= 0.01 and brain.timer.time(SECONDS)-initial_time <= timeout:
            print(err)
            err = error_calc(distance.object_distance(INCHES))
            target = control.update(err) * speed
            # spin ahoy!
            dt_left.spin(FORWARD, target, PERCENT)
            dt_right.spin(FORWARD, target, PERCENT)
            # 10 msec wait
            wait(wait_time*1000, MSEC)
        dt_left.stop()
        dt_right.stop()
    def drive4(self, inches, speed=200, timeout=1):
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
        # factor = 1.315556
        # main loop
        while running:
            wait(10, MSEC)
            if brain.timer.time(SECONDS) - initial_time > timeout:
                running = False
            elif dt_right.velocity(PERCENT) == 0:
                # recalculate as a form of error filtering
                wait(40, MSEC)
                running = not (dt_left.velocity(PERCENT) == 0)
        # stop  
        dt_left.stop()
        dt_right.stop()
        # logging
    def turn2(self, angle_unmodded, c=(3, 0, 0.2), speed=100, timeout=2):
        '''
        ### A turn-to-heading function which uses a PD feedback loop (can easily be modified for PID) in tandem with the IMU to achieve very precise results.

        #### Arguments:
            angle_unmodded: angle turning to, in degrees clockwise. Negative inputs are allowed and will be interpreted as degrees counterclockwise. Reference point is set during calibration
            speed (>0, <100): absolute speeds of both sides, in percent (100 works, and there's no reason to change it)
            timeout (>0): DEPRECATED - do not use, full stop. At some point in the future, let's test then finalize this function with this input removed
        #### Returns:
            None
        #### Examples:
            # draws a square
            for i in range(4):
                dt.turn2(i*90)
                dt.drive4(10)
        '''
        # error calc fn
        angle = angle_unmodded % 360
        def error_calc(d):
            abs_err = (angle - d) % 360 # 0 to 360
            norm_err = (-abs_err % -180)+(180 if abs_err>=180 else 0) # -180 to 180
            return norm_err/180 # -1 to 1
        # basic vars & controller setup
        wait_time = 0.01
        err = error_calc(orientation.heading(DEGREES))
        control = PID(wait_time, err, c)
        initial_time = brain.timer.time(SECONDS)
        while (abs(err) >= 0.01 or abs(dt_left.velocity(PERCENT)) >= 5) and brain.timer.time(SECONDS)-initial_time <= timeout:
            # speed calcs
            err = error_calc(orientation.heading(DEGREES))
            target = control.update(err)*speed
            volt = clamp_volt(target*12/100)
            # spin ahoy!
            dt_left.spin(REVERSE, volt, VoltageUnits.VOLT) # type: ignore
            dt_right.spin(FORWARD, volt, VoltageUnits.VOLT) # type: ignore
            # 10 msec wait
            wait(wait_time*1000, MSEC)
        dt_left.stop()
        dt_right.stop()
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

    def old_turn2(self, angle_unmodded, speed=41):
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
# END NEW CODE HERE
       
dt = Drivetrain(1, 4, 11)

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
# declare the triball config
class FieldTriball:
    def __init__(self, color, dist, angle, signatures: dict[str, Signature]):
        self.color = color
        self.dist = dist
        self.angle = angle
        self.vision_signature = signatures[color]
    def score(self, sensor: Vision):
        # this important functions rates the "error" of detected triball
        shot = sensor.take_snapshot(self.vision_signature)
        if shot == None:
            return (1000, RIGHT)
        shot = shot[0]
        # calculations, oh my!
        raw_cam_dist = (1/math.tan(30.5*math.pi/180))*3*316*0.5/(shot.width)
        actual_ang = orientation.heading(DEGREES)
        abs_err = ((self.angle % 360) - actual_ang) % 360 # 0 to 360
        norm_err = (-abs_err % -180)+(180 if abs_err>=180 else 0) # -180 to 180
        # distance
        precise_distance = abs(distance.object_distance(INCHES)-self.dist)/10
        accurate_distance = abs(raw_cam_dist-self.dist)/10
        # center
        centered = (316/2-shot.centerX)/(316/2)
        # angle
        angle_diff = norm_err/180
        # final
        return (angle_diff + precise_distance + accurate_distance + abs(centered)), (RIGHT if centered > 0 else LEFT)
# vision stuff
class VisionPro:
    def __init__(self, config):
        def signatures_brightness_from_utility(generated_config):
            def numerify(str):
                try: 
                    return int(str)
                except:
                    return float(str)
            lines = [x for x in generated_config.split('\n') if not x == '']
            triballs = lines[:3]
            triballs = [l.split('(')[-1][:-2] for l in triballs]
            triballs = [[numerify(i) for i in l.split(',')] for l in triballs]
            signatures = [Signature(*x) for x in triballs]

            brightness = int(lines[-1].split(',')[1])
            return signatures, brightness
        sigs, brightness = signatures_brightness_from_utility(config)
        self.signatures = dict(zip(
            ['GREEN', 'RED', 'BLUE'],
            sigs
        ))
        self.colors = {
            'GREEN': Color.GREEN,
            'BLUE': Color.BLUE,
            'RED': Color.RED
        }
        self.index_lookup = {'GREEN': 1, 'RED': 2, 'BLUE': 3}
        self.sensor = Vision(Ports.PORT16, brightness, *sigs) # resolution 316 horizontal, 212 vertical
    def turn2triball(self, triball: str, timeout=1):
        # uses trig to calculate approx relative angle of triball relative to vision sensor
        # setup fns and consts
        def pixel_to_angle(pixel_diff):
            return math.atan(
                pixel_diff * math.tan(30.5 * math.pi/180) / 108
            ) * 180/math.pi
        initial_time = brain.timer.time(SECONDS)
        # wait until take snapshot
        snap = self.sensor.take_snapshot(self.signatures[triball])
        timeouted = False
        running = True
        while running:
            snap = self.sensor.take_snapshot(self.signatures[triball])
            if not snap == None:
                print('DETECTED')
                running = False
            if brain.timer.time(SECONDS) - initial_time > timeout:
                print('TIMEOUTING')
                timeouted = True
                running = False
            wait(50, MSEC)
        # math time!
        if not timeouted:
            angle = pixel_to_angle(108-snap[0].centerX)
            dt.old_turn2(orientation.heading(DEGREES) + angle)
            if distance.object_distance(INCHES) > 40:
                self.turn2triball(triball)
            else:
                dt.drive4(-distance.object_distance(INCHES)*0.7)
    def _turn2tri(self, color: str, angle, target_dist, tol_dist = 2, tol_angle = 10, timeout=2):
        dt.old_turn2(angle-9)
        print(orientation.heading(DEGREES))
        # next
        dt_left.spin(FORWARD, 3, PERCENT)
        dt_right.spin(REVERSE, 3, PERCENT)
        # loop
        running = True
        initial_time = brain.timer.time(SECONDS)
        reason = ""
        while running:
            if not ( (angle-tol_angle) % 360 < orientation.heading(DEGREES) < (angle + tol_angle) % 360 ):
                reason = "ANGLE TOLERANCE"
                running = False
            if brain.timer.time(SECONDS) - initial_time > timeout:
                reason = "TIMEOUT"
                running = False
            # if target_dist - tol_dist < distance.object_distance(INCHES) < target_dist + tol_dist:
            #     reason = "DISTANCE MET"
            #     running = False
            if distance.object_size == ObjectSizeType.MEDIUM:
                reason = 'MEDIUM'
                running = False
         # stop all
        dt_left.stop()
        dt_right.stop()
        print(reason, distance.object_distance(INCHES), orientation.heading(DEGREES))

config_str = '''vision::signature TRIBALL_GREEN (1, -5183, -2857, -4020, -6013, -3823, -4918, 3.000, 0);
vision::signature TRIBALL_RED (2, 7821, 10139, 8980, -1445, -1053, -1250, 3.000, 0);
vision::signature TRIBALL_BLUE (3, -3619, -2753, -3186, 6785, 9133, 7960, 3.000, 0);
vision::signature SIG_4 (4, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_5 (5, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 3.000, 0);
vex::vision vision1 ( vex::PORT1, 50, TRIBALL_GREEN, TRIBALL_RED, TRIBALL_BLUE, SIG_4, SIG_5, SIG_6, SIG_7 );'''
reality = VisionPro(config_str) # definitely not a joke ;)
# colors = ['GREEN', 'RED', 'BLUE']
# while True:
#     wait(10, MSEC)
#     color = None
#     if controller_1.buttonX.pressing():
#         color = 'GREEN'
#     elif controller_1.buttonA.pressing() or controller_1.buttonY.pressing():
#         color = 'RED'
#     elif controller_1.buttonB.pressing():
#         color = 'BLUE'
#     if color:
#         reality.turn2triball(color)
#     wait(5, MSEC)

#reality._turn2tri('GREEN', 45, target_dist=)