from vex import *

# Brain should be defined by default
brain=Brain()
controller_1 = Controller(PRIMARY)
# motors
dt_left_1 = Motor(Ports.PORT11, GearSetting.RATIO_6_1, True)
dt_left_2 = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True)
dt_left_3 = Motor(Ports.PORT13, GearSetting.RATIO_6_1, True)
dt_right_1 = Motor(Ports.PORT20, GearSetting.RATIO_6_1, False)
dt_right_2 = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)
dt_right_3 = Motor(Ports.PORT18, GearSetting.RATIO_6_1, False)
# motor groups
dt_left = MotorGroup(
    dt_left_1,
    dt_left_2,
    dt_left_3
)
dt_right = MotorGroup(
    dt_right_1,
    dt_right_2,
    dt_right_3
)
# data printing fn
def data(m, logs):
    full = [
        str(m.velocity(PERCENT))+"/"+str(m.velocity(RPM)),
        m.temperature(PERCENT),
        m.efficiency(PERCENT),
        #m.rotation(),
        m.current(),
        m.power(),
        m.torque(),
        m.is_spinning(),
        m.is_done(),
    ]
    buff = ""
    for i in range(len(full)):
        if logs[i] == "1":
            buff += str(full[i]) + "\t\t\t\t"
    return buff
# spin!
dt_left.spin(FORWARD, 10, VOLT)
dt_right.spin(FORWARD, 10, VOLT)
# main loop
def logger():
    full_desc = ["velocity%", "temp C", "efficiency%", "current", "power", "torque", "is spinning", "is done"]
    log_state = "11100000" # 1 if you want to show it, 0 otherwise
    buff_desc = ""
    for i in range(len(full_desc)):
            if log_state[i] == "1":
                buff_desc += full_desc[i] + " "
    print(buff_desc + "\n" + '-'*20 + "\n\n")
    while True:
        wait(10, MSEC)
        all_motors = {
            'l1': dt_left_1,
            'l2': dt_left_2,
            'l3': dt_left_3,
            'r1': dt_right_1,
            'r2': dt_right_2,
            'r3': dt_right_3
        }
        for k, v in all_motors.items():
            print(k, data(v, log_state))
        print('\n\n\n')
# run loop
print("\033[2J")
logger()