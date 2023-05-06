# import library
import time
import mpu
import csv
import datetime
import math
import signal
import threading

import numpy as np
from const import parameter
from pymavlink import mavutil
from geopy.distance import geodesic
from robot import Robot
import calculate_degree as calculator
import robot_control_action as actions
from INA226 import INA226
from smbus import SMBus
from w1thermsensor import W1ThermSensor

stop_thread = True

sensor = W1ThermSensor()

arduino = 0x04
i2cbus = SMBus(1)
const = parameter()

control_mode = const.control_mode
strategy = const.strategy

if control_mode is 0:
    print('CONTROL MODE: OMNIDIRECTIONAL CONTROL')
elif control_mode is 1:
    print('CONTROL MODE: DIAGONAL CONTROL')

if strategy is 0:
    print('STRATEGY: SIMPLE STRATEGY')
elif strategy is 1:
    print('STRATEGY: FLEX STRATEGY')

# read waypoint file (csv)
target_point_file = const.way_point_file
print('WAY POINT FILE: ', target_point_file)
target_point = np.genfromtxt(target_point_file,
                          delimiter=',',
                          dtype='float',
                          encoding='utf-8')

state_data_log = const.data_log_mode
wt_log_mode = const.wt_log_mode
if (state_data_log is True):
    # get date time object
    detail = datetime.datetime.now()
    date = detail.strftime("%Y%m%d%H%M%S")
    # open csv file
    file = open('./csv/'+ date +'.csv', 'a', newline='')
    csvWriter = csv.writer(file)
    data_items = ['count', 'latitude', 'longitude', 'yaw', 'cmd',
                    'pwm', 'voltage', 'current', 'power_consumption', 'kJ_poewr_consumption', 'accum_power_consumption', 'distance']
    csvWriter.writerow(data_items)

    # open csv file
    if wt_log_mode == True:
        wt_file = open('./csv/'+ date +'_wt.csv', 'a', newline='')
        wt_csvWriter = csv.writer(wt_file)
        wt_data_items = ['wt']
        wt_csvWriter.writerow(wt_data_items)

# connect to the robot
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# wait heart beat
master.wait_heartbeat()

# Request all parameters
master.mav.param_request_list_send(
    master.target_system, master.target_component
)

# set robot mode to manual and armable
def set_mode(mode):
    # Get mode ID: return value should be 1 for acro mode
    mode_id = master.mode_mapping()[mode]
    # Set new mode
    master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

def set_arm_disarm(msg):
    if(msg=='ARM'):
        cmd = 1
    elif(msg=='DISARM'):
        cmd = 0

    master.mav.command_long_send(master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    cmd,
    1, 0, 0, 0, 0, 0, 0)

# update the robot state
def update_robot_state():
    attitude_message = master.recv_match(type='ATTITUDE', blocking=True).to_dict()
    yaw = float(attitude_message['yaw'])
    BIWAKO.yaw = yaw
    if int(BIWAKO.count*10) % 10 == 0:
        GPS_message = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()
        lon = float(GPS_message['lon'])/10**7
        lat = float(GPS_message['lat'])/10**7
        # lon = 135.00
        # lat = 35.00
        BIWAKO.lon = lon
        BIWAKO.lat = lat

# control thrusters
def thruster_initialization(action):
    interval = 0.02
    ch = action[0]
    pwm = action[1]
    rc_channel_values = [65535 for _ in range(8)]
    rc_channel_values[ch - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.
    time.sleep(interval)

# set RC mode
def set_RC_mode():
    i2cbus.write_i2c_block_data(arduino, 0, [0, 0])

# send control command to arduino
def control_thruster(cmd, same_pwm=False):
    if same_pwm == False:
        i2cbus.write_i2c_block_data(arduino, 2, cmd)
    else:
        pass

# calculate heading difference between current point to target point
def calc_heading_diff(way_point):
    t_lon = math.radians(way_point[0])
    t_lat = math.radians(way_point[1])
    c_lon = math.radians(BIWAKO.lon)
    c_lat = math.radians(BIWAKO.lat)
    d_lon = c_lon - t_lon

    diff_heading = 90 - math.degree(math.atan2(math.cos(t_lat)*math.sin(c_lat)
                  - math.sin(t_lat)*math.cos(c_lat)*math.cos(d_lon), math.sin(d_lon)*math.cos(c_lat)))
    return diff_heading

# kill signal process
def kill_signal_process(arg1, args2):
    global stop_thread
    if wt_log_mode == True:
        stop_thread = False
        print("KILL WT LOGGING PROCESS")
        update_wt_thread.join()
    exit(0)

# logging function
def logging():
    v = power_sensor.get_voltage()
    c = power_sensor.get_current()
    p = power_sensor.get_power()
    kJ = p/1000
    BIWAKO.count = BIWAKO.count + const.timer
    BIWAKO.consumed_energy = BIWAKO.consumed_energy + kJ
    data = [BIWAKO.count, BIWAKO.lat, BIWAKO.lon, math.degrees(BIWAKO.yaw),
            BIWAKO.cmd, BIWAKO.pwm, v, c, p, kJ, BIWAKO.consumed_energy, BIWAKO.diff_distance]
    log_data.append(data)

# calculate the target point when the robot is in the simple strategy
def calc_temp_target(current_point, target_point):
    current_point = np.array([current_point])
    target_point = np.array([target_point])
    temp_target = target_point-(current_point-target_point)/2
    temp_target = [temp_target[0][0], temp_target[0][1]]
    return temp_target

# calculate the target point when the robot is in the flexible strategy
def calc_flexible_temp_goal(current_point, target_point, prev_target, r):
    lat_t = target_point[0]
    lon_t = target_point[1]
    earth_R = 6378137
    theta = lat_t
    ex = 360/(2*math.pi*earth_R*math.cos(theta*math.pi/180)) # keido, longitude 1[deg] = ex[m]
    ey = 360/(2*math.pi*earth_R) # ido, latitude 1[deg] = ey[m]
    bearing = calculator.calculate_bearing(current_point, prev_target)
    lat_temp_next = lat_t+r*ey*math.sin(math.radians(90-bearing)) # 90-bearing means "coordinate transformation"
    lon_temp_next = lon_t+r*ex*math.cos(math.radians(90-bearing)) # 90-bearing means "coordinate transformation"
    target_point = np.array([lat_temp_next, lon_temp_next])
    return target_point

# update water temperature
def update_wt():
    global stop_thread
    while True:
        if stop_thread == False:
            break
        temperature_in_celsius = sensor.get_temperature()
        wt_data = [temperature_in_celsius]
        wt_log_data.append(wt_data)
        time.sleep(0.01)

# signal handler
def signal_handler(signal, frame):
    logging()
    update_robot_state()

# update robot state thread
if wt_log_mode == True:
    update_wt_thread = threading.Thread(target=update_wt)
    update_wt_thread.start()

if __name__ == '__main__':

    debug_mode = const.debug_mode
    wt_log_mode = const.wt_log_mode
    control_mode = const.control_mode
    if control_mode == 0:
        print('CONTROL MODE: OMNIDIRECTIONAL CONTROL')
    elif control_mode == 1:
        print('CONTROL MODE: DIAGONAL CONTROL')
    elif control_mode == 2:
        print('CONTROL MODE: FIXED-HEAD CONTROL')
    elif control_mode == 3:
        print('CONTROL MODE: OCT-DIRECTIONAL CONTROL')

    strategy = const.strategy

    main_target_distance_tolerance = const.main_target_distance_tolerance
    temp_target_distance_tolerance = const.temp_target_distance_tolerance
    distance_tolerance = main_target_distance_tolerance
    heading_torelance = const.heading_torelance
    keep_time = const.duration
    r = const.r

    BIWAKO = Robot(target_point)
    power_sensor_addr = 0x40
    power_sensor = INA226(power_sensor_addr)
    power_sensor.initial_operation()

    log_data = [] # log data
    wt_log_data = [] # water temperature log data
    action_log = [0, 0] # action log
    pwm_log = [0] # pwm log
    deg_e = [0] # degree error log
    is_first = 0 # flag for the first time to reach the target point
    is_first_return = 0 # flag for the first time to reach the target point
    way_point_num = 0 # waypoint number
    is_wait = 0 # flag for waiting
    s_time = 0.0 # start time
    wait_time = 0 # wait time for the next action 
    base_time = [0.0] # base time
    
    # set timer
    monitoring_time = 60 * 5 #[sec]
    breaking_time = 60 * 60 * 0.5#[sec] 15 -> 35
    end_duration = 60 * 60 * 3 #[sec]
        
    BIWAKO.next_goal = target_point[way_point_num]

    # Initialize the robot
    set_mode('MANUAL')
    print("Set mode to Manual")
    initial_action = [0, 0]
    print('Initialize...')
    print('Wait a second...')
    control_thruster(initial_action)
    time.sleep(1)
    master.arducopter_arm()
    print("Arm/Disarm: Arm")

    try:
        signal.signal(signal.SIGALRM, signal_handler)
        signal.setitimer(signal.ITIMER_REAL, 0.5, const.timer)
        start_time = time.time()

        # srtart main loop
        while True:
            now = time.time()
            print("target point: ", way_point_num)
            print("duration: ", now - s_time)
            print("wait time: ", wait_time)

            # if the duration is over, then finish the mission
            if now-start_time > end_duration+180:
                print("Finish")
                break

            # update robot state
            pose = [BIWAKO.lon, BIWAKO.lat, BIWAKO.yaw]

            # decide the next action from current robot status and the next waypoint
            current_point = np.array([pose[1], pose[0]])
            print(current_point)
            current_yaw = pose[2]
            diff_distance = round(mpu.haversine_distance(current_point, BIWAKO.next_goal), 5)*1000
            BIWAKO.diff_distance = diff_distance

            # if the robot is close to the target point, then the robot will wait for a while
            if abs(diff_distance) < distance_tolerance:
                if is_first == 0:
                    is_first = 1
                    s_time = time.time()
                    if way_point_num==len(target_point)-1:
                        break_time = breaking_time - (now - start_time)
                        wait_time = break_time # breaking time
                    else:
                        wait_time = monitoring_time
                
                # if the waiting time is over, then the robot will move to the next waypoint
                if now - s_time > wait_time:
                    is_first = 0
                    way_point_num += 1
                    if len(target_point)-1 < way_point_num:
                        start_time = now
                        way_point_num = 0
                        distance_tolerance = main_target_distance_tolerance
                    elif way_point_num == len(target_point)-1:
                        distance_tolerance = temp_target_distance_tolerance
                    BIWAKO.next_goal = target_point[way_point_num]

                # stay action
                action = actions.stay_action()
                BIWAKO.cmd = action[0]
                BIWAKO.pwm = action[1]
                pwm_log.append(BIWAKO.pwm)
                control_thruster(action)
                action_log.append(action)
                v = power_sensor.get_voltage()
                if v < 11.3:
                    print("Voltage: ", v)
                    print("LOW BATTERY!!!!!!!!")
                time.sleep(0.03)

            # if the robot is far from the target point, then the robot will move to the target point
            else:
                # if the waiting time is over, then the robot will move to the next waypoint
                if(is_first==1):
                    if now - s_time > wait_time:
                        is_first = 0
                        way_point_num += 1
                        if len(target_point)-1 < way_point_num:
                            start_time = now
                            way_point_num = 0
                            distance_tolerance = main_target_distance_tolerance
                        elif way_point_num == len(target_point)-1:
                            distance_tolerance = temp_target_distance_tolerance
                        BIWAKO.next_goal = target_point[way_point_num]

                # calculate the next action
                target_direction = math.radians(calculator.calculate_bearing(current_point, BIWAKO.next_goal))
                diff_deg =  math.degrees(calculator.limit_angle(target_direction - current_yaw))
                deg_e.append(diff_deg)
                action = actions.oct_directional_action(diff_deg, diff_distance, pwm_log[-1])
                action_log.append(action)

                BIWAKO.cmd = action[0]
                BIWAKO.pwm = action[1]
                pwm_log.append(BIWAKO.pwm)

                # if pwm value is same as previous one, then do not send command to arduino
                if pwm_log[-1] == pwm_log[-2]:
                    same_pwm = True
                control_thruster(action, same_pwm)

                v = power_sensor.get_voltage()
                if v < 11.3:
                    print("Voltage: ", v)
                    print("LOW BATTERY!!!!!!!!")
                time.sleep(0.03)

        # finish the mission
        set_RC_mode()

        master.arducopter_disarm()
        print("Arm/Disarm: Disarm")
        if (state_data_log is True):
            print("Writing log")
            for i in range(len(log_data)):
                csvWriter.writerow(log_data[i])
            file.close()

            if wt_log_mode == True:
                for j in range(len(wt_log_data)):
                    wt_csvWriter.writerow(wt_log_data[j])
                wt_file.close()
        signal.signal(signal.SIGALRM, kill_signal_process)
        signal.setitimer(signal.ITIMER_REAL, 0.1, 0.1)

    # if the program is interrupted by keyboard, then disarm the robot
    except KeyboardInterrupt:
        set_RC_mode()
        master.arducopter_disarm()
        print("Arm/Disarm: Disarm")
        if (state_data_log is True):
            print("Writing log")
            for i in range(len(log_data)):
                csvWriter.writerow(log_data[i])
            file.close()
            
            if wt_log_mode == True:
                for j in range(len(wt_log_data)):
                    wt_csvWriter.writerow(wt_log_data[j])
                wt_file.close()
        signal.signal(signal.SIGALRM, kill_signal_process)
        signal.setitimer(signal.ITIMER_REAL, 0.1, 0.1)
