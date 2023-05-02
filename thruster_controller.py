import time
from smbus import SMBus

# init the i2c device
arduino = 0x04
i2cbus = SMBus(1)

# mode determination method
def input_mode():
    while True:
        print("Please select a control mode")
        print("0: RC mode, 1: Auto mode, 2: Manual mode, 3: END")
        input_value = int(input())
        if input_value < 0 or input_value > 3:
            print("Invalid value. Input again")
        else:
            return input_value

# robot action determination method
def input_performance_number():
    while True:
        print("Please input robot performance number")
        print("0:Stop, 1: Forward, 2: Backward, 3: Left, 4: Right, 5:CW, 6: CCW")
        input_value = int(input())
        if input_value < 0 or input_value > 6:
            print("Invalid number. Input again")
        else:
            if input_value == 0:
                print("Stop")
            elif input_value == 1:
                print("Forward")
            elif input_value == 2:
                print("Backward")
            elif input_value == 3:
                print("Left")
            elif input_value == 4:
                print("Right")
            elif input_value == 5:
                print("CW")
            elif input_value == 6:
                print("CCW")
            return input_value

# thruster power determination method
def input_power():
    while True:
        print("Please input thruster power")
        print("0 - 100 [%]")
        input_value = int(input())
        if input_value < 0.0 or input_value > 100.0:
            print("Invalid value. Input again")
        else:
            return input_value

# init the variables value
mode_value = 0
performance_value = 0
power_value = 0

# main loop
while True:
    try:
        mode_value = int(input_mode())
        if mode_value == 0 or mode_value == 1 or mode_value == 3:
            cmd = [0, 0]
        if mode_value == 2:
            cmd = [performance_value, power_value]
            i2cbus.write_i2c_block_data(arduino, mode_value, cmd)
            performance_value = int(input_performance_number())
            if 1 <= performance_value <= 6:
                power_value = int(input_power())
            elif performance_value == 0:
                power_value = 0
            cmd = [performance_value, power_value]
        i2cbus.write_i2c_block_data(arduino, mode_value, cmd)
        if mode_value == 3:
            cmd = [0, 0]
            i2cbus.write_i2c_block_data(arduino, 0, cmd)
            break
        time.sleep(0.1)

    except KeyboardInterrupt:
        cmd = [0, 0]
        i2cbus.write_i2c_block_data(arduino, 0, cmd)
        time.sleep(1)
        break
