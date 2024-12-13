from pyb import I2C
import struct
import struct
import pyb

class IMU:
    """
    @brief Class to enable and use BNO055 imu
    @detail With this class you can set the duty of the motor from -100 to 100

    """
    
    I2C_ADDR = 0x28  # Update if necessary, could also be 0x29
    OPR_MODE_REG = 0x3D
    IMU_MODE = 0x08
    COMPASS_MODE = 0x09
    M4G_MODE = 0x0A
    NDOF_FMC_OFF_MODE = 0x0B
    NDOF_MODE = 0x0C
    CALIB_STAT_REG = 0x35
    
    # Register addresses for calibration data
    ACCEL_OFFSET_X_LSB = 0x55
    MAG_OFFSET_X_LSB = 0x5B
    GYRO_OFFSET_X_LSB = 0x61
    ACCEL_RADIUS_LSB = 0x67
    MAG_RADIUS_LSB = 0x69
    
    # Euler angle register addresses
    EULER_H_LSB = 0x1A
    EULER_R_LSB = 0x1C
    EULER_P_LSB = 0x1E
    SCALE_FACTOR = 1 / 16.0
    
    # Gyroscope data register addresses
    GYRO_X_LSB = 0x14
    GYRO_Y_LSB = 0x16
    GYRO_Z_LSB = 0x18
    GYRO_SCALE_FACTOR = 1 / 16.0
    
    def __init__(self, controller):
        self.controller = controller
        self.op_mode("ndof")
    
    def op_mode(self, mode):
        if mode.lower() == "imu":
            self.controller.mem_write(self.IMU_MODE, self.I2C_ADDR, self.OPR_MODE_REG)
        elif mode.lower() == "compass":
            self.controller.mem_write(self.COMPASS_MODE, self.I2C_ADDR, self.OPR_MODE_REG)
        elif mode.lower() == 'm4g':
            self.controller.mem_write(self.M4G_MODE, self.I2C_ADDR, self.OPR_MODE_REG)
        elif mode.lower() == 'ndof_fmc':
            self.controller.mem_write(self.NDOF_FMC_OFF_MODE, self.I2C_ADDR, self.OPR_MODE_REG)
        elif mode.lower() == 'ndof':
            self.controller.mem_write(self.NDOF_MODE, self.I2C_ADDR, self.OPR_MODE_REG)
        else:
            raise ValueError("Invalid Mode")
            
    def cal_status(self):
        # Read calib status byte from register
        calib_stat = self.controller.mem_read(1, self.I2C_ADDR, self.CALIB_STAT_REG)[0]
        
        # parse the data
        sys_calib = (calib_stat >> 6) & 0x03
        gyro_calib = (calib_stat >> 4) & 0x03
        accel_calib = (calib_stat >> 2) & 0x03
        mag_calib = calib_stat & 0x03
        
        return {
            'system': sys_calib,
            'gyroscope': gyro_calib,
            'accelerometer': accel_calib,
            'magnetometer': mag_calib
        }
    
    def write_calibration_data(self, calib_data):
        """
        Writes calibration data to the BNO055. Expects 22 bytes of binary data.
        :param calib_data: A bytes or bytearray object with the 22 bytes of calibration data.
        """
        if len(calib_data) != 22:
            raise ValueError("Calibration data must be 22 bytes.")

        # Set to CONFIG_MODE
        self.set_mode(self.CONFIG_MODE)
        
        # Write calibration data to IMU
        self.controller.mem_write(calib_data[0:6], self.I2C_ADDR, self.ACCEL_OFFSET_X_LSB)
        self.controller.mem_write(calib_data[6:12], self.I2C_ADDR, self.MAG_OFFSET_X_LSB)
        self.controller.mem_write(calib_data[12:18], self.I2C_ADDR, self.GYRO_OFFSET_X_LSB)
        self.controller.mem_write(calib_data[18:20], self.I2C_ADDR, self.ACCEL_RADIUS_LSB)
        self.controller.mem_write(calib_data[20:22], self.I2C_ADDR, self.MAG_RADIUS_LSB)

        # Return to the previous mode (e.g., NDOF_MODE)
        self.set_mode(self.NDOF_MODE)
        
    def read_euler_angles(self):
        """
        Reads the heading, roll, and pitch from the IMU.
        :return: A dictionary with 'heading', 'roll', and 'pitch' angles in degrees.
        """
        # Read 6 bytes starting from EULER_H_LSB
        euler_data = self.controller.mem_read(6, self.I2C_ADDR, self.EULER_H_LSB)
        
        # Convert bytes to angles with scaling
        #heading = int.from_bytes(euler_data[0:2], 'little', signed=True) * self.SCALE_FACTOR
        #roll = int.from_bytes(euler_data[2:4], 'little', signed=True) * self.SCALE_FACTOR
        #pitch = int.from_bytes(euler_data[4:6], 'little', signed=True) * self.SCALE_FACTOR
        heading, roll, pitch = struct.unpack('<hhh', euler_data)

        return {
            'heading': heading,
            'roll': roll,
            'pitch': pitch
        }

    def read_heading(self):
        """
        Reads only the heading angle from the IMU.
        :return: The heading angle in degrees.
        """
        # Read 2 bytes for the heading angle
        heading_data = self.controller.mem_read(2, self.I2C_ADDR, self.EULER_H_LSB)
        
        # Convert to angle with scaling
        #heading = int.from_bytes(heading_data, 'little', signed=True) * self.SCALE_FACTOR
        heading = struct.unpack('<h', heading_data)[0]

        return heading
    
    def read_angular_velocity(self):
        """
        Reads the angular velocity for all axes (X, Y, Z) from the IMU.
        :return: A dictionary with 'x', 'y', and 'z' angular velocities in degrees per second.
        """
        # Read 6 bytes starting from GYRO_X_LSB
        gyro_data = self.controller.mem_read(6, self.I2C_ADDR, self.GYRO_X_LSB)
        
        # Convert bytes to angular velocities with scaling
        #x_rate = int.from_bytes(gyro_data[0:2], 'little', signed=True) * self.GYRO_SCALE_FACTOR
        #y_rate = int.from_bytes(gyro_data[2:4], 'little', signed=True) * self.GYRO_SCALE_FACTOR
        #z_rate = int.from_bytes(gyro_data[4:6], 'little', signed=True) * self.GYRO_SCALE_FACTOR
        x_rate, y_rate, z_rate = struct.unpack('<hhh', gyro_data)

        return {
            'x': x_rate,
            'y': y_rate,
            'z': z_rate
        }

    def read_yaw_rate(self):
        """
        Reads only the yaw rate (Z-axis angular velocity) from the IMU.
        :return: The yaw rate in degrees per second.
        """
        # Read 2 bytes for the Z-axis (yaw) angular velocity
        yaw_data = self.controller.mem_read(2, self.I2C_ADDR, self.GYRO_Z_LSB)
        
        # Convert to angular velocity with scaling
        #yaw_rate = int.from_bytes(yaw_data, 'little', signed=True) * self.GYRO_SCALE_FACTOR
        yaw_rate = struct.unpack('<h', yaw_data)[0]/16
        return yaw_rate

    # Existing class methods remain unchanged
    
    CONFIG_MODE = 0x00  # Config mode for writing calibration data
    
    def calibrate(self, wait_until_calibrated=True):
        """
        Waits until the IMU is fully calibrated or returns current calibration status.
        If wait_until_calibrated is True, the method blocks until all sensors are calibrated.
        """
        print("Calibrating IMU... Please keep the robot stationary.")
        
        while True:
            calib_status = self.cal_status()
            print(f"Calibration Status: {calib_status}")
            
            if wait_until_calibrated:
                # Wait until all subsystems are fully calibrated (status 3)
                if calib_status['gyroscope'] == 3:
                    print("IMU fully calibrated!")
                    break
            else:
                # Return the current calibration status without blocking
                return calib_status
            

    def read_calibration_data(self):
        """
        Reads the current calibration coefficients from the IMU.
        :return: A bytearray containing the 22 bytes of calibration data.
        """
        # Read calibration offsets from the IMU
        accel_offsets = self.controller.mem_read(6, self.I2C_ADDR, self.ACCEL_OFFSET_X_LSB)
        mag_offsets = self.controller.mem_read(6, self.I2C_ADDR, self.MAG_OFFSET_X_LSB)
        gyro_offsets = self.controller.mem_read(6, self.I2C_ADDR, self.GYRO_OFFSET_X_LSB)
        accel_radius = self.controller.mem_read(2, self.I2C_ADDR, self.ACCEL_RADIUS_LSB)
        mag_radius = self.controller.mem_read(2, self.I2C_ADDR, self.MAG_RADIUS_LSB)

        # Combine all data into a single bytearray
        return bytearray(accel_offsets + mag_offsets + gyro_offsets + accel_radius + mag_radius)

    def save_calibration_data(self, filename="imu_calib.dat"):
        """
        Saves the calibration data to a file.
        :param filename: Name of the file to save the calibration data.
        """
        calib_data = self.read_calibration_data()
        with open(filename, 'wb') as f:
            f.write(calib_data)
        print(f"Calibration data saved to {filename}.")

    def load_calibration_data(self, filename="imu_calib.dat"):
        """
        Loads calibration data from a file and writes it to the IMU.
        :param filename: Name of the file to load calibration data from.
        """
        try:
            with open(filename, 'rb') as f:
                calib_data = f.read()
            if len(calib_data) != 22:
                raise ValueError("Calibration data must be 22 bytes.")
            self.write_calibration_data(calib_data)
            print(f"Calibration data loaded from {filename}.")
        except FileNotFoundError:
            print(f"Calibration file {filename} not found. Please calibrate the IMU manually.")

    def auto_calibrate(self, filename="imu_calib.dat"):
        """
        Automatically handles IMU calibration:
        - Checks if calibration data is available.
        - If not, performs calibration and saves the data.
        - If available, loads the calibration data.
        """
        try:
            # Try to load calibration data
            self.load_calibration_data(filename)
        except Exception as e:
            print(f"Loading calibration data failed: {e}")
            # Perform calibration if loading fails
            self.calibrate(wait_until_calibrated=True)
            self.save_calibration_data(filename)
            
    def calibrate_2(self):
        """
        Waits until the IMU is fully calibrated.
        Prints calibration progress for system, gyroscope, accelerometer, and magnetometer.
        """
        print("Starting IMU calibration...")
        while True:
            status = self.cal_status()
            print(f"Calibration Status - System: {status['system']}, Gyro: {status['gyroscope']}, Accel: {status['accelerometer']}, Mag: {status['magnetometer']}")

            if all(value == 3 for value in status.values()):  # Fully calibrated when all are 3
                print("IMU fully calibrated!")
                break

            pyb.delay(100)  # Wait before checking again

    

if __name__=="__main__":
    i2c = I2C(1, I2C.CONTROLLER)
    i2c.init(I2C.CONTROLLER, baudrate=20000)
    imu = IMU(i2c)
