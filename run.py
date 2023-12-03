import matplotlib.pyplot as plt
import numpy as np
import time
from supernovacontroller.sequential import SupernovaDevice
import ctypes


def find_matching_item(data, target_pid):
    for item in data:
        if item.get('pid') == target_pid:
            return item
    return None

class ICM42605:
    pid = [0x04, 0x6A, 0x00, 0x00, 0x00, 0x00]
    address = None

    # Sensor resolutions
    a_scale = 0x03 # 2g full scale
    g_scale = 0x03 # 250 dps full scale
    a_res = 2.0 / 32768.0 # 2 g full scale
    g_res = 250.0 / 32768.0 # 250 dps full scale
    a_odr = 0x06 #AODR_1000Hz
    g_odr = 0x06 #GODR_1000Hz

    # Sensor status
    status = None

    # Calibration
    accel_bias = None
    gyro_bias = None

    def __init__(self, i3c):
        self.i3c = i3c

        (_, targets) = i3c.targets()

        icm_device = find_matching_item(targets, self.pid)

        if icm_device is None:
            print("ICM device not found in the I3C bus")
            return
        
        self.address = icm_device["dynamic_address"]

    def init_device(self):
        # Enable gyro and accel in low noise mode
        ICM42605_PWR_MGMT0 = 0x4E
        (_, power_management_register) = self.i3c.read(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_PWR_MGMT0], 1)
        self.i3c.write(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_PWR_MGMT0], [power_management_register[0] | 0x0F])

        # Gyro full scale and data rate
        ICM42605_GYRO_CONFIG0 = 0x4F
        (_, gyro_config_register0) = self.i3c.read(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_GYRO_CONFIG0], 1)
        self.i3c.write(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_GYRO_CONFIG0], [gyro_config_register0[0] | self.g_odr | self.g_scale << 5])

        # Set accel full scale and data rate
        ICM42605_ACCEL_CONFIG0 = 0x50
        (_, accel_config) = self.i3c.read(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_ACCEL_CONFIG0], 1)
        self.i3c.write(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_ACCEL_CONFIG0], [accel_config[0] | self.a_odr | self.a_scale << 5])

        # Set temperature sensor low pass filter to 5Hz, use first order gyro filter
        ICM42605_GYRO_CONFIG1 = 0x56
        (_, gyro_config_register1) = self.i3c.read(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_GYRO_CONFIG1], 1)
        self.i3c.write(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_GYRO_CONFIG1], [gyro_config_register1[0] | 0xD0])

        # Set both interrupts active high, push-pull, pulsed
        ICM42605_INT_CONFIG0 = 0x63
        (_, int_config0) = self.i3c.read(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_INT_CONFIG0], 1)
        self.i3c.write(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_INT_CONFIG0], [int_config0[0] | 0x18 | 0x03])

        # Set bit 4 to zero for proper function of INT1 and INT2
        ICM42605_INT_CONFIG1 = 0x64
        (_, int_config1) = self.i3c.read(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_INT_CONFIG1], 1)
        self.i3c.write(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_INT_CONFIG1], [int_config1[0] & ~(0x10)])

        # Route data ready interrupt to INT1
        ICM42605_INT_SOURCE0 = 0x65
        (_, int_source0) = self.i3c.read(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_INT_SOURCE0], 1)
        self.i3c.write(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_INT_SOURCE0], [int_source0[0] | 0x08])

        # Route AGC interrupt interrupt to INT2
        ICM42605_INT_SOURCE3 = 0x68
        (_, int_source3) = self.i3c.read(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_INT_SOURCE3], 1)
        self.i3c.write(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_INT_SOURCE3], [int_source3[0] | 0x01])

        # Select Bank 4
        ICM42605_REG_BANK_SEL = 0x76
        (_, reg_bank_sel) = self.i3c.read(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_REG_BANK_SEL], 1)
        self.i3c.write(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_REG_BANK_SEL], [reg_bank_sel[0] | 0x04])

        # Select unitary mounting matrix
        ICM42605_APEX_CONFIG5 = 0x7A
        (_, apex_config5) = self.i3c.read(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_APEX_CONFIG5], 1)
        self.i3c.write(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_APEX_CONFIG5], [apex_config5[0] & ~(0x07)])

        # Select Bank 0
        ICM42605_REG_BANK_SEL = 0x76
        (_, reg_bank_sel) = self.i3c.read(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_REG_BANK_SEL], 1)
        self.i3c.write(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_REG_BANK_SEL], [reg_bank_sel[0] & ~(0x07)])

        ## Read Status
        ICM42605_INT_STATUS = 0x19
        (_, self.status) = self.i3c.read(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_INT_STATUS], 1)

    def calibrate(self):
        sum_values = [0, 0, 0, 0, 0, 0, 0]
        accel_bias = [0, 0, 0]
        gyro_bias = [0, 0, 0]

        for i in range(128):
            # Read data
            imu_data = self._read_data()
            sum_values[1] += imu_data[1]
            sum_values[2] += imu_data[2]
            sum_values[3] += imu_data[3]
            sum_values[4] += imu_data[4]
            sum_values[5] += imu_data[5]
            sum_values[6] += imu_data[6]

        accel_bias[0] = sum_values[1] * self.a_res / 128.0
        accel_bias[1] = sum_values[2] * self.a_res / 128.0
        accel_bias[2] = sum_values[3] * self.a_res / 128.0
        gyro_bias[0] = sum_values[4] * self.g_res / 128.0
        gyro_bias[1] = sum_values[5] * self.g_res / 128.0
        gyro_bias[2] = sum_values[6] * self.g_res / 128.0

        if accel_bias[0] > 0.8:
            accel_bias[0] -= 1.0  # Remove gravity from the x-axis accelerometer bias calculation
        if accel_bias[0] < -0.8:
            accel_bias[0] += 1.0  # Remove gravity from the x-axis accelerometer bias calculation
        if accel_bias[1] > 0.8:
            accel_bias[1] -= 1.0  # Remove gravity from the y-axis accelerometer bias calculation
        if accel_bias[1] < -0.8:
            accel_bias[1] += 1.0  # Remove gravity from the y-axis accelerometer bias calculation
        if accel_bias[2] > 0.8:
            accel_bias[2] -= 1.0  # Remove gravity from the z-axis accelerometer bias calculation
        if accel_bias[2] < -0.8:
            accel_bias[2] += 1.0  # Remove gravity from the z-axis accelerometer bias calculation 

        self.accel_bias = accel_bias
        self.gyro_bias = gyro_bias

    def _read_data(self):
        ICM42605_TEMP_DATA1 = 0x1D

        (_, raw_data) = self.i3c.read(self.address, self.i3c.TransferMode.I3C_SDR, [ICM42605_TEMP_DATA1], 14)
        imu_data = [0,0,0,0,0,0,0]
        imu_data[0] = ctypes.c_int16((raw_data[0] << 8) | raw_data[1]).value
        imu_data[1] = ctypes.c_int16((raw_data[2] << 8) | raw_data[3]).value
        imu_data[2] = ctypes.c_int16((raw_data[4] << 8) | raw_data[5]).value
        imu_data[3] = ctypes.c_int16((raw_data[6] << 8) | raw_data[7]).value
        imu_data[4] = ctypes.c_int16((raw_data[8] << 8) | raw_data[9]).value
        imu_data[5] = ctypes.c_int16((raw_data[10] << 8) | raw_data[11]).value
        imu_data[6] = ctypes.c_int16((raw_data[12] << 8) | raw_data[13] ).value
        return imu_data

    def read(self):
        imu_data = self._read_data()

        ax = imu_data[1]*self.a_res - self.accel_bias[0]
        ay = imu_data[2]*self.a_res - self.accel_bias[1]
        az = imu_data[3]*self.a_res - self.accel_bias[2]

        gx = imu_data[4]*self.g_res - self.gyro_bias[0]
        gy = imu_data[5]*self.g_res - self.gyro_bias[0]
        gz = imu_data[6]*self.g_res - self.gyro_bias[0]

        return ((ax, ay, az), (gx, gy, gz))


def main():
    device = SupernovaDevice()

    info = device.open()

    print(info)

    i3c = device.create_interface("i3c.controller")

    i3c.set_parameters(i3c.I3cPushPullTransferRate.PUSH_PULL_12_5_MHZ, i3c.I3cOpenDrainTransferRate.OPEN_DRAIN_4_17_MHZ)
    (success, _) = i3c.init_bus(3300)

    if not success:
        print("I couldn't initialize the bus. Are you sure there's any target connected?")
        exit(1)

    sensor = ICM42605(i3c)

    sensor.init_device()

    sensor.calibrate()

    # Setup the matplotlib figure and axes
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1)
    fig.subplots_adjust(hspace=0.5)
    fig.suptitle('Supernova with ICM42605 Device Demo', fontsize=16)
    plt.get_current_fig_manager().set_window_title("Sensor Data Visualization")

    # Set static titles for the subplots

    # Initialize lists to store the data
    times = []
    acc_data = {'x': [], 'y': [], 'z': []}
    gyro_data = {'x': [], 'y': [], 'z': []}

    start_time = time.time()

    keep_running = True

    def on_key(event):
        nonlocal keep_running
        if event.key == 'q':
            keep_running = False

    fig.canvas.mpl_connect('key_press_event', on_key)

    while keep_running:
        current_time = time.time() - start_time
        times.append(current_time)

        # Read data from sensor
        acc, gyro = sensor.read()

        # Update accelerometer data
        acc_data['x'].append(acc[0])
        acc_data['y'].append(acc[1])
        acc_data['z'].append(acc[2])

        # Update gyroscope data
        gyro_data['x'].append(gyro[0])
        gyro_data['y'].append(gyro[1])
        gyro_data['z'].append(gyro[2])

        # Plot accelerometer data
        ax1.cla()
        ax1.plot(times, acc_data['x'], label='X')
        ax1.plot(times, acc_data['y'], label='Y')
        ax1.plot(times, acc_data['z'], label='Z')
        ax1.legend()
        ax1.set_title('Accelerometer Data')

        # Plot gyroscope data
        ax2.cla()
        ax2.plot(times, gyro_data['x'], label='X')
        ax2.plot(times, gyro_data['y'], label='Y')
        ax2.plot(times, gyro_data['z'], label='Z')
        ax2.legend()
        ax2.set_title('Gyroscope Data')

        plt.pause(0.1)

        # Limit the size of the data lists
        if len(times) > 50:
            times.pop(0)
            for data_list in acc_data.values():
                data_list.pop(0)
            for data_list in gyro_data.values():
                data_list.pop(0)

    plt.close(fig)

    device.close()

if __name__ == "__main__":
    main()