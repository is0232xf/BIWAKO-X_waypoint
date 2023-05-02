
from const import parameter

const = parameter()

def power_corrector(power):
    if power > 50:
        power = 50
    elif power < 0:
        power = 0
    return power

def P_control(distance):
    Kp = const.distance_Kp
    power = int(power_corrector(Kp * distance))
    return power

def PD_heading_control(diff_deg, e):
    # diff_deg; type: float, unit: [deg]
    # e; type: object(list), unit:[deg]
    Kp = const.degree_Kp
    Kd = const.degree_Kd
    diff_e = e[len(e)-2]-e[len(e)-1]
    power = int(power_corrector(Kp * diff_deg + Kd * diff_e * (1/0.02)))
    return power

def comp_pwm(power, latest_pwm):
    accl = 1
    diff = power - latest_pwm
    if abs(diff) > accl:
        if diff > 0:
            power = latest_pwm + accl
        elif diff < 0:
            power = latest_pwm - accl
    return power

def stay_action():
    action_num = 0
    pwm = 0
    action = [action_num, pwm]

    if const.debug_mode is True:
        print("Keep the position")
        print("########################")
    return action

def omni_control_action(diff_deg, diff_distance, latest_pwm):
    power = P_control(diff_distance)
    # power = comp_pwm(power, latest_pwm)
    if -45.0 <= diff_deg < 45:
        action_num = 1

    elif -180.0 <= diff_deg < -135.0 or 135.0 <= diff_deg < 180.0:
        action_num = 2

    elif 45.0 <= diff_deg < 135.0:
        action_num = 4

    elif -135.0 <= diff_deg < -45.0:
        action_num = 3

    action = [action_num, power]

    if const.debug_mode is True:
        if action_num is 1:
            print("FORWARD")
        elif action_num is 2:
            print("BACKWARD")
        elif action_num is 3:
            print("RIGHT")
        elif action_num is 4:
            print("LEFT")

        print("power[%]: ", power)
        print("diff deg: ", diff_deg)
        print("diff distance: ", diff_distance)

    return action

def diagonal_action(diff_deg, diff_distance, latest_pwm):
    power = P_control(diff_distance)
    # power = comp_pwm(power, latest_pwm)
    # [0]:-180.0, [1]:-90.0, [2]:0.0, [3]:90.0, [4]:180.0
    if 0.0 <= diff_deg < 90.0:
        action_num = 7
    elif -90.0 <= diff_deg < 0.0:
        # forward second quadrant
        action_num = 8
    elif -180.0 <= diff_deg < -90.0:
        # forward third quadrant
        action_num = 9
    elif 90.0 <= diff_deg <= 180:
        # forward fourth quadrant
        action_num = 10

    if const.debug_mode is True:
        if action_num == 7:
            print('First Quadrant')
        elif action_num == 8:
            print('Second Quadrant')
        elif action_num == 9:
            print('Third Quadrant')
        elif action_num == 10:
            print('Fourth Quadrant')

        action = [action_num, power]
        print("action: ", action)
        print("diff deg: ", diff_deg)
        print("diff distance: ", diff_distance)

    return action

def oct_directional_action(diff_deg, diff_distance, latest_pwm):
    power = P_control(diff_distance)
    # power = comp_pwm(power, latest_pwm)
    # [0]:-180.0, [1]:-90.0, [2]:0.0, [3]:90.0, [4]:180.0
    action_num = 0
    if -22.0 <= diff_deg <22.0:
        action_num = 11

    elif -180.0 <= diff_deg < -157.0 or 157.0 <= diff_deg <= 180.0:
        action_num = 12

    elif -112.0 <= diff_deg < -67.0:
        action_num = 13

    elif 67.0 <= diff_deg < 112.0:
        action_num = 14

    elif 22.0 <= diff_deg < 67.0:
        action_num = 7

    elif -67.0 <= diff_deg < -22.0:
        action_num = 8

    elif -157.0 <= diff_deg < -112.0:
        action_num = 9

    elif 112.0 <= diff_deg < 157.0:
        action_num = 10

    if const.debug_mode is True:
        if action_num is 11:
            print("FORWARD")
        elif action_num is 12:
            print("BACKWARD")
        elif action_num is 13:
            print("RIGHT")
        elif action_num is 14:
            print("LEFT")
        elif action_num == 7:
            print('First Quadrant')
        elif action_num == 8:
            print('Second Quadrant')
        elif action_num == 9:
            print('Third Quadrant')
        elif action_num == 10:
            print('Fourth Quadrant')

    action = [action_num, power]
    # print("action: ", action)
    # print("diff deg: ", diff_deg)
    print("diff distance: ", diff_distance)

    return action

def fixed_head_action(diff_deg, diff_distance, deg_e):
    # heading_torelance = const.heading_torelance
    heading_torelance = 30
    if abs(diff_deg) < heading_torelance:
        action_num = 1
        power = P_control(diff_distance)
    elif diff_deg >= heading_torelance:
        action_num = 5
        power = PD_heading_control(diff_deg, deg_e)
    elif diff_deg < -1.0 * heading_torelance:
        action_num = 6
        power = PD_heading_control(diff_deg, deg_e)
    action = [action_num, power]

    if const.debug_mode is True:
        if action_num == 1:
            print('FORWARD')
        elif action_num == 2:
            print('CW')
        elif action_num == 3:
            print('CCW')
        else:
            print('No Command')
        print("action: ", action)
        print("diff deg: ", diff_deg)
        print("diff distance: ", diff_distance)

    return action
