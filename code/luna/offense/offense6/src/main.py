#region VEXcode Generated Robot Configuration
from vex import *
import random

# Brain should be defined by default
brain=Brain()

# Robot configuration code


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

#endregion VEXcode Generated Robot Configuration
import time
import math
import random
# constants
OPEN = True
CLOSE = False
# controller
controller_1 = Controller(PRIMARY)
# logging explanation
controller_1.screen.set_cursor(1, 1)
controller_1.screen.print("DT Temps")

controller_1.screen.set_cursor(2, 1)
controller_1.screen.print("Cata")

controller_1.screen.set_cursor(3, 1)
controller_1.screen.print("Inertial")
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
# sensors
dist = Distance(Ports.PORT2)

orientation = Inertial(Ports.PORT1)
orientation.calibrate()
while orientation.is_calibrating():
    wait(10, MSEC)
print(brain.timer.time(MSEC), "Inertial calibrated")
brain.timer.clear()
"""
TODO:
Switch to default drivetrain class
Ditch components class
Remove drivetrain class and broaden dt functions' scope
Rewrite driver controls through arcade
"""

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
        self.dt_time = 0
        self.turn_time = 0
        self.num_turns = 0
        self.turn_times = []
    def drive4_aadish(self, inches:float, speed:int=200, timeout:float=1):
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
                running = False
            elif dt_right.velocity(PERCENT) == 0:
                # recalculate as a form of error filtering
                wait(40, MSEC)
                running = not (dt_left.velocity(PERCENT) == 0)
        # stop  
        dt_left.stop()
        dt_right.stop()
        # logging

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
        #print("drive4/done", inches, "in, took", brain.timer.time(SECONDS)-initial_time, 'sec')
        self.dt_time += brain.timer.time(SECONDS)-first

    def drive4_good_old(self, inches, speed=100, timeout=3):
        start_time = brain.timer.time(SECONDS)

        dt_left.set_timeout(15)
        dt_right.set_timeout(15)
        dt_left.set_velocity(speed, PERCENT)
        dt_right.set_velocity(speed, PERCENT)
        direction = FORWARD if inches > 0 else REVERSE
        dt_left.spin_for(direction, (abs(inches)/(math.pi*self.wheel_diameter))*self.gear_ratio, TURNS, wait=False)
        dt_right.spin_for(direction, (abs(inches)/(math.pi*self.wheel_diameter))*self.gear_ratio, TURNS, wait=False)
        
        #print("drive4/waiting \t")
        while brain.timer.time(SECONDS) - start_time < timeout:
            #print(int(dt_left.is_spinning()), int(dt_right.is_spinning()), int(dt_left.velocity(PERCENT)), end="\t")
            if dt_left.is_done():
                break
            wait(10, MSEC)
        #print("\ndrive4/done", inches, "in, took", brain.timer.time(SECONDS)-old_time, "sec")
        dt_left.stop()
        dt_right.stop()
        end_time = brain.timer.time(SECONDS)
        self.dt_time = self.dt_time + (end_time - start_time)

    def drive4_old(self, inches, speed=100, timeout=3):
        dt_left.set_timeout(15)
        dt_right.set_timeout(15)
        dt_left.set_velocity(speed, PERCENT)
        dt_right.set_velocity(speed, PERCENT)
        direction = FORWARD if inches > 0 else REVERSE
        dt_left.spin_for(direction, (abs(inches)/(math.pi*self.wheel_diameter))*self.gear_ratio, TURNS, wait=False)
        dt_right.spin_for(direction, (abs(inches)/(math.pi*self.wheel_diameter))*self.gear_ratio, TURNS, wait=False)
        
        old_time = brain.timer.time(SECONDS)
        print("drive4/waiting \t")
        while brain.timer.time(SECONDS) - old_time < timeout:
            print(int(dt_left.is_spinning()), int(dt_right.is_spinning()), int(dt_left.velocity(PERCENT)), end="\t")
            if dt_left.is_done():
                break
            wait(10, MSEC)
        print("\ndrive4/done", inches, "in, took", brain.timer.time(SECONDS)-old_time, "sec")
        dt_left.stop()
        dt_right.stop()

    def turn2(self, angle_unmodded, speed=41, tolerance = 1):
        start_time =  brain.timer.time(SECONDS)
        angle = angle_unmodded % 360
        h = orientation.heading(DEGREES)
        while abs(angle - h) % 360 > tolerance:
            h = orientation.heading(DEGREES)
            dt_left.set_velocity(abs((angle - h + 180) % 360 - 180) * speed * 2 / 180 + 3, PERCENT)
            dt_right.set_velocity(abs((angle - h + 180) % 360 - 180) * speed * 2 / 180 + 3, PERCENT)
            dt_left.spin(FORWARD if (angle - h + 180) % 360 - 180 > 0 else REVERSE)
            dt_right.spin(REVERSE if (angle - h + 180) % 360 - 180 > 0 else FORWARD)
            wait(10, MSEC)
        dt_left.stop()
        dt_right.stop()
        if abs(angle - orientation.heading(DEGREES)) % 360 > tolerance:
            self.turn2(angle, speed=10)
        end_time = brain.timer.time(SECONDS)
        time_taken = end_time - start_time
        self.turn_time = self.turn_time + time_taken
        self.num_turns = self.num_turns + 1
        self.turn_times.append((angle_unmodded, time_taken))
        return (end_time - start_time)
        
    def arc_new(self,left_speed, right_speed,duration):
        dt_right.set_velocity(right_speed, PERCENT)
        dt_left.set_velocity(left_speed, PERCENT)
        dt_right.spin(FORWARD)
        dt_left.spin(FORWARD)
        wait(duration, SECONDS)
        dt_left.stop()
        dt_right.stop()

    def arc(self, rad, ang, head=FORWARD, side=RIGHT, speed=25, timeout=5):
        """
        Inputs:
        rad (>0): Radius of arc, in inches
        ang (>0): Angle of arc, in degrees
        head (FORWARD/REVERSE): whether the robot is moving for/backwards
        side (LEFT/RIGHT): which side the robot is arc towards
        """
        # constants
        aside = 1 if side == RIGHT else -1

        dt_const = self.gear_ratio/(math.pi*self.wheel_diameter) # multiply to get # of turns
        in_const = (ang/360)*2*math.pi # multiply to get # of inches
        # targets for each side
        left = dt_const*in_const*(rad+aside*self.wheelbase/2) # the parenthesized portion is the abs value
        right = dt_const*in_const*(rad-aside*self.wheelbase/2) # of the circumference of the side's circle
        # velocities
        dt_left.set_velocity(speed*math.sqrt(left/right), PERCENT)
        dt_right.set_velocity(speed*math.sqrt(right/left), PERCENT)
        # spin!
        dt_right.spin_for(head, right, TURNS, wait=False)
        dt_left.spin_for(head, left, TURNS, wait=False)
        # timeout routine
        old_time = brain.timer.time(SECONDS)
        while brain.timer.time(SECONDS) - old_time < timeout:
            if dt_left.is_done():
                break
            wait(10, MSEC)
        dt_left.stop()
        dt_right.stop()
dt = Drivetrain(48/36, 3.25, 11)
cp = Components(80)
mangle = 20
# helper functions
def matchload_setup():
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
    brain.timer.clear()
    while True:
        wait(10, MSEC)
        # dt controls
        straight_speed = controller_1.axis3.position()
        turn_speed = 0.5*controller_1.axis1.position()
        l = straight_speed + turn_speed
        r = straight_speed - turn_speed
        dt_left.spin((FORWARD if l >= 0 else REVERSE), abs(l)*12/100, VOLT) # type: ignore
        dt_right.spin((FORWARD if r >= 0 else REVERSE), abs(r)*12/100, VOLT) # type: ignore
        # logging
        logs = [
            "time: %g" % brain.timer.time(SECONDS),
            "dt: %d %d" % (dt_left.temperature(PERCENT), dt_right.temperature(PERCENT)), # type: ignore
            "cata: %d" % catapult.temperature(PERCENT),
        ]
        # controller_1.screen.set_cursor(1 + log_n % 3, 1)
        # controller_1.screen.print(logs[log_n % 3])
        log_n += 1
        # hang mode
        if brain.timer.time(SECONDS) > 50 and skills and hang_n > 0:
            if hang_n == 10:
                controller_1.rumble('-.-.-.')
                endg.set(True)
            print_all('HANG MODE')
            hang_n -= 1
            if hang_n == 0:
                controller_1.screen.clear_screen()
# advanced, high-level class for things that aren't directly related to components or drivetrain
class Advanced:
    def __init__(self, distance: Distance):
        self.distance_sensor = distance
    def find_triball(self, min_angle, max_angle, max_distance, direction=LEFT, turn_to_start = True):
        if turn_to_start:
            dt.turn2(min_angle)
        # next
        dt_left.spin(REVERSE if direction==LEFT else FORWARD, 2, PERCENT)
        dt_right.spin(FORWARD if direction==LEFT else REVERSE, 2, PERCENT)
        # loop
        found = None
        running = True
        while not found:
            if min(min_angle, max_angle)-1 > orientation.heading(DEGREES) > max(min_angle, max_angle)+1:
                print(orientation.heading(DEGREES))
                found = 'ANGLED OUT'
            #print(self.distance_sensor.object_size())
            size = self.distance_sensor.object_size()
            print(size, size in [ObjectSizeType.MEDIUM, ObjectSizeType.SMALL])
            if size in [ObjectSizeType.MEDIUM, ObjectSizeType.SMALL]:
                distance = self.distance_sensor.object_distance(INCHES)
                print(distance)
                found = 'FOUND'
                running = (distance <= max_distance) and self.distance_sensor.is_object_detected()
            wait(10, MSEC)
         # stop all
        dt_left.stop()
        dt_right.stop()
        return found
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
# autonomous
def autonomous_6():
    #initialization
    dt_left.set_stopping(BRAKE)
    dt_right.set_stopping(BRAKE)
    orientation.set_heading(90)
    brain.timer.clear()
    # take out and score matchload + preload
    cp.intake(FORWARD)
    wait(300,MSEC)
    dt.drive4(-22,timeout=0.7)
    #checkTime('after going back')
    dt.turn2(90, tolerance=5)
    cp.wings()
    dt.arc_new(-50,-30,1)

    cp.wings()
    wait(100,MSEC)
    dt.arc_new(-100,-60,1)
    #checkTime('after arcs to push in matchload + preload')
    dt.drive4(4, speed=200, timeout = 0.2)
    dt.drive4(-8, speed = 200, timeout = 0.3)
    dt.drive4(4, speed=200, timeout = 0.2)

    # score elevation
    dt.turn2(180, tolerance = 3)
    cp.intake(REVERSE)
    dt.drive4(10,speed=200, timeout=0.3)
    # dt.drive4(-3,speed=200,timeout=0.2)
    # dt.drive4(3,speed=200,timeout=0.2)
    #checkTime('scoring elevation!!!')
    dt.turn2(180, tolerance = 3)
    dt.drive4(-12,timeout=0.5)

    # intake less impossible
    cp.intake(FORWARD)
    dt.turn2(105)
    dt.drive4(18)
    found = advanced.find_triball(105, 90, 37, direction=LEFT, turn_to_start=False)
    dist = advanced.distance_sensor.object_distance(INCHES) if found == 'FOUND' else 50
    print(dist)
    dt.drive4(dist, timeout=1.5)
    dt.turn2(30)
    cp.wings()

    # arc!
    #dt.drive4(-10,timeout=1)
    #dt.turn2(0)
    dt.arc_new(-41,-88, 1) 
    dt.turn2(90)

    # smash
    dt.drive4(-10,timeout=1)

    # score less impossible
    dt.drive4(4)
    cp.wings()
    dt.turn2(-90)
    cp.intake(REVERSE)
    dt.drive4(12)
    dt.drive4(-10)

    # stats
    print(brain.timer.time(SECONDS))
autonomous_6()
#driver_control()
#competition = Competition(driver_control, autonomous_6)
'''
    dt.turn2(-90)
    dt.drive4(-3)

    dt.turn2(180)
    dt.drive4(20)
    dt.turn2(175)
    dt.drive4(20)
    cp.intake(FORWARD)
    initial = brain.timer.time(SECONDS)
    advanced.find_triball('GREEN', -135, tol_angle=10)
    print(brain.timer.time(SECONDS)-initial)
    dt.drive4(7)
''' 
