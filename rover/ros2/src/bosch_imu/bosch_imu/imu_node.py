#!/usr/bin/env python3
# --------------------------------------------------------------------------- #
"""
Code Information:
    Programmer: Camilo Alvis B.
	Mail: camiloalvis@kiwicampus.com
	Kiwibot AI Team

Note:
    Wrapper for the IMU Bosch BNO055 in ROS 2. Wrapper implemented for UART 
    communication.

Sources:
    Original version: https://github.com/mdrwiega/bosch_imu_driver
"""

# --------------------------------------------------------------------------- #

import rclpy 
import os
import sys
import math
import serial
import binascii
import numpy as np

from rclpy.node import Node
from time import time

from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import MagneticField

from .utils.transformations import euler_from_quaternion
from .utils.transformations import quaternion_from_euler

# --------------------------------------------------------------------------- #
# BOSCH BNO055 IMU Registers map and other information
# Page 0 registers
CHIP_ID = 0x00
PAGE_ID = 0x07
ACCEL_DATA = 0x08
MAG_DATA = 0x0e
GYRO_DATA = 0x14
FUSED_EULER = 0x1a
FUSED_QUAT = 0x20
LIA_DATA = 0x28
GRAVITY_DATA = 0x2e
TEMP_DATA = 0x34
CALIB_STAT = 0x35
SYS_STATUS = 0x39
SYS_ERR = 0x3a
UNIT_SEL = 0x3b
OPER_MODE = 0x3d
PWR_MODE = 0x3e
SYS_TRIGGER = 0x3f
TEMP_SOURCE = 0x440
AXIS_MAP_CONFIG = 0x41
AXIS_MAP_SIGN = 0x42

ACC_OFFSET = 0x55
MAG_OFFSET = 0x5b
GYR_OFFSET = 0x61
ACC_RADIUS = 0x68
MAG_RADIUS = 0x69

# Page 1 registers
ACC_CONFIG = 0x08
MAG_CONFIG = 0x09
GYR_CONFIG0 = 0x0a
GYR_CONFIG1 = 0x0b

# Operation modes
OPER_MODE_CONFIG = 0x00
OPER_MODE_ACCONLY = 0x01
OPER_MODE_MAGONLY = 0x02
OPER_MODE_GYROONLY = 0x03
OPER_MODE_ACCMAG = 0x04
OPER_MODE_ACCGYRO = 0x05
OPER_MODE_MAGGYRO = 0x06
OPER_MODE_AMG = 0x07
OPER_MODE_IMU = 0x08
OPER_MODE_COMPASS = 0x09
OPER_MODE_M4G = 0x0a
OPER_MODE_NDOF_FMC_OFF = 0x0b
OPER_MODE_NDOF = 0x0C

# Power modes
PWR_MODE_NORMAL = 0x00
PWR_MODE_LOW = 0x01
PWR_MODE_SUSPEND  = 0x02

# Communication constants
BNO055_ID = 0xa0
START_BYTE_WR = 0xaa
START_BYTE_RESP = 0xbb
READ = 0x01
WRITE = 0x00

class ImuNode(Node):

    def __init__(self):

        super().__init__("imu_node")

        self._set_process_name(name="imu_node")

        # Publishers
        self.pub_imu_data = self.create_publisher(
            Imu, 'imu/data', 1)             # Filtered data - imu_msg
        self.pub_imu_mag = self.create_publisher(
            MagneticField, 'imu/mag', 1)    # Magnetometer data - mag_msg
        self.pub_imu_temp = self.create_publisher(
            Temperature, 'imu/temp', 1)     # Temperature data - tmp_msg
        self.pub_bot_data = self.create_publisher(
            Imu, 'imu/data_bot', 1)         # Kiwibot filtered data - imu_bot_msg 
        self.pub_bot_ekf = self.create_publisher(
            Imu, 'imu/data_bot_ekf', 1)     # Kiwibot filtered data for EKF - imu_ekf_msg
        self.pub_bot_move = self.create_publisher(
            Bool, 'imu/move_state', 1)      # Kiwibot movement status - move_msg 
        self.pub_imu_status = self.create_publisher(
            String, 'imu/status', 1)        # Kiwibot IMU status - status_msg 
            
        """
        self.pub_imu_raw = self.create_publisher(
            Imu, 'imu/raw_data', 1)         # Raw data - imu_raw_msg
        """

        # Publisher flags
        self._counter = 0
        self._acc_values = np.array([])
        self._move_counter = 0
        self._moving_old = False
        self._move_pub = False
        self._first_orient = False
        self._initial_orient = 0.0
        self._alpha = 0.995
        self._azimuth = 0.0

        # Parameters setup
        self.port = '/dev/ttyTHS2'
        self.frame_id = "imu_link"
        self.frequency = 30
        self.opr_mode = 0x0C
        self.pub_rate = 1.0 / self.frequency

        # Data objects
        self.imu_msg = Imu()            # Filtered data
        self.imu_raw_msg = Imu()        # Raw data
        self.imu_bot_msg = Imu()        # Kiwibot filtered data
        self.imu_ekf_msg = Imu()        # Kiwibot filtered data for EKF
        self.tmp_msg = Temperature()    # Temperature
        self.mag_msg = MagneticField()  # Magnetometer data
        self.move_msg = Bool()          # Move status data
        self.status_msg = String()      # IMU status data

        # IMU factors
        self.acc_fact_ = 1000.0
        self.mag_fact_ = 16.0
        self.gyr_fact_ = 900.0

        # Angles initialization
        self._pitch_bot = 0.0
        self._roll_bot = 0.0
        self._yaw_bot = 0.0

        # Magnetic field for Berkeley - For Medellin it is -8.2 , -14.3
        self._mgn_decl = float(
            os.getenv("IMU_MAGNETIC_DECLINATION", -8.2)) * (math.pi / 180.0)
        self._move_factor = float(os.getenv("IMU_MOVING_FACTOR", 0.014))

        # Open Serial Port
        self.get_logger().info("Opening Serial Port: " + self.port)

        try:
            self.ser = serial.Serial(
                port=self.port, baudrate=115200, timeout=0.02)

        except serial.serialutil.SerialException:
            self.get_logger().error(
                "IMU not found at port " + self.port + ". Check the port number")
            sys.exit(0)

        # Check if IMU ID is correct
        if not self._check_id():
            self.get_logger().error("Device ID is incorrect. Shutdown.")
            correct_id = False

            for x in range(30):
                self.get_logger().error("Rechecking IMU device ID attempt {}".format(x))
                if self._check_id():                  
                    correct_id = True              
                    break
        
        # IMU Setup
        if not(self._write_to_dev(OPER_MODE, 1, OPER_MODE_CONFIG)):
            self.get_logger().error("Unable to set IMU into configuration mode.")

        if not(self._write_to_dev(PWR_MODE, 1, PWR_MODE_NORMAL)):
            self.get_logger().error("Unable to set IMU normal power mode.")

        if not(self._write_to_dev(PAGE_ID, 1, 0x00)):
            self.get_logger().error("Unable to set IMU register page 0.")

        if not(self._write_to_dev(SYS_TRIGGER, 1, 0x00)):
            self.get_logger().error("Unable to start IMU.")

        if not(self._write_to_dev(UNIT_SEL, 1, 0x83)):
            self.get_logger().error("Unable to set IMU units.")

        if not(self._write_to_dev(AXIS_MAP_CONFIG, 1, 0x24)):
            self.get_logger().error("Unable to remap IMU axis.")

        if not(self._write_to_dev(AXIS_MAP_SIGN, 1, 0x06)):
            self.get_logger().error("Unable to set IMU axis signs.")

        if not(self._write_to_dev(OPER_MODE, 1, OPER_MODE_NDOF)):
            self.get_logger().error("Unable to set IMU operation into operation mode.")
        
        self.get_logger().info("Bosch BNO055 IMU configuration completed.")
        
        # Timers
        self.pub_timer_ = self.create_timer(
            timer_period_sec=self.pub_rate,
            callback=self.cb_publish_imu
        )

# --------------------------------------------------------------------------- #   
## Member functions 
    def _set_process_name(self, name):
        """
            Desc: Kills all the processes
            Args: Name of the process
            Returns: Nan
        """

        if sys.platform in ['linux2', 'linux']:
            import ctypes
            libc = ctypes.cdll.LoadLibrary('libc.so.6')
            libc.prctl(15, name, 0, 0, 0)

        else:
            raise Exception(
                "Can not set the process name on non-linux systems: " + \
                str(sys.platform))

    def _check_id(self):
        """
            Desc: Checks if the IMU ID is correct
            Args: NaN
            Returns: 
                Buffer ID if succesfully identifies the ID. False if the ID is 
                not correct (Bool)
        """

        buf = self._read_from_dev(CHIP_ID, 1)
        
        if buf == 0:
            return False
        
        return buf[0] == BNO055_ID
        
    def _read_from_dev(self, reg_addr, length):
        """
            Desc: Read the buffer at the Serial Port
            Args: Register address and message length
            Returns: Buffer with the reading data
        """

        # Data allocation in the exit buffer
        buf_out = bytearray()
        buf_out.append(START_BYTE_WR)
        buf_out.append(READ)
        buf_out.append(reg_addr)
        buf_out.append(length)

        try: 
            self.ser.write(buf_out)
            buf_in = bytearray(self.ser.read(2 + length))
        except:
            return 0
        
        # Response checking
        if (buf_in.__len__() != (2 + length)) or (buf_in[0] != START_BYTE_RESP):
            return 0
        
        buf_in.pop(0)
        buf_in.pop(0)

        return buf_in

    def _write_to_dev(self, reg_addr, length, data):
        """
            Desc: Read the buffer at the Serial Port
            Args: Register address and message length
            Returns: Writing status flag
        """

        # Data allocation in the exit buffer
        buf_out = bytearray()
        buf_out.append(START_BYTE_WR)
        buf_out.append(WRITE)
        buf_out.append(reg_addr)
        buf_out.append(length)
        buf_out.append(data)

        try: 
            self.ser.write(buf_out)
            buf_in = bytearray(self.ser.read(2))
        except:
            return False

        # Response checking
        if (buf_in.__len__() != 2) or (buf_in[1] != 0x01):
            return False
        
        return True

    def _update_imu_data(self):
        """
            Desc: Returns the euler angles from a given quaternion
            Args: Quaterion (X, Y, Z, W)
            Returns: Euler angles (Roll, Ptich, Yaw)
        """

        buf = self._read_from_dev(ACCEL_DATA, 45)
        if buf != 0:
            # --------------------------------------------------------------- #
            # Gravity values from the IMU buffer
            gravity_x = float(st.unpack('h', st.pack(
                'BB', buf[38], buf[39]))[0]) / 100.0
            gravity_y = float(st.unpack(
                'h', st.pack('BB', buf[40], buf[41]))[0]) / 100.0
            gravity_z = float(st.unpack(
                'h', st.pack('BB', buf[42], buf[43]))[0]) / 100.0

            # --------------------------------------------------------------- #
            """
            # Raw data
            self.imu_raw_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.imu_raw_msg.header.frame_id = self.frame_id
            self.imu_raw_msg.orientation_covariance[0] = -1
            self.imu_raw_msg.linear_acceleration.x = float(st.unpack(
                'h', st.pack('BB', buf[0], buf[1]))[0]) / self.acc_fact_
            self.imu_raw_msg.linear_acceleration.y = float(st.unpack(
                'h', st.pack('BB', buf[2], buf[3]))[0]) / self.acc_fact_
            self.imu_raw_msg.linear_acceleration.z = float(st.unpack(
                'h', st.pack('BB', buf[4], buf[5]))[0]) / self.acc_fact_
            self.imu_raw_msg.linear_acceleration_covariance[0] = -1
            self.imu_raw_msg.angular_velocity.x = float(st.unpack(
                'h', st.pack('BB', buf[12], buf[13]))[0]) / self.gyr_fact_
            self.imu_raw_msg.angular_velocity.y = float(st.unpack(
                'h', st.pack('BB', buf[14], buf[15]))[0]) / self.gyr_fact_
            self.imu_raw_msg.angular_velocity.z = float(st.unpack(
                'h', st.pack('BB', buf[16], buf[17]))[0]) / self.gyr_fact_
            self.imu_raw_msg.angular_velocity_covariance[0] = -1
            """

            # --------------------------------------------------------------- #
            # Filtered data
            self.imu_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.imu_msg.header.frame_id = self.frame_id
            self.imu_msg.orientation.w = float(st.unpack('h', st.pack(
                'BB', buf[24], buf[25]))[0])
            self.imu_msg.orientation.x = float(st.unpack('h', st.pack(
                'BB', buf[26], buf[27]))[0])
            self.imu_msg.orientation.y = float(st.unpack('h', st.pack(
                'BB', buf[28], buf[29]))[0])
            self.imu_msg.orientation.z = float(st.unpack('h', st.pack(
                'BB', buf[30], buf[31]))[0])

            self.imu_msg.linear_acceleration.x = float(st.unpack('h', st.pack(
                'BB', buf[32], buf[33]))[0]) / self.acc_fact_ + gravity_x
            self.imu_msg.linear_acceleration.y = float(st.unpack('h', st.pack(
                'BB', buf[34], buf[35]))[0]) / self.acc_fact_ + gravity_y
            self.imu_msg.linear_acceleration.z = float(st.unpack('h', st.pack(
                'BB', buf[36], buf[37]))[0]) / self.acc_fact_ + gravity_z
            self.imu_msg.linear_acceleration_covariance[0] = -1

            self.imu_msg.angular_velocity.x = float(st.unpack('h', st.pack(
                'BB', buf[12], buf[13]))[0]) / self.gyr_fact_
            self.imu_msg.angular_velocity.y = float(st.unpack('h', st.pack(
                'BB', buf[14], buf[15]))[0]) / self.gyr_fact_
            self.imu_msg.angular_velocity.z = float(st.unpack('h', st.pack(
                'BB', buf[16], buf[17]))[0]) / self.gyr_fact_
            self.imu_msg.angular_velocity_covariance[0] = -1

            # --------------------------------------------------------------- #
            # Quaternion value allocation
            quaternion_orig = [
                self.imu_msg.orientation.x,
                self.imu_msg.orientation.y,
                self.imu_msg.orientation.z,
                self.imu_msg.orientation.w]

            # --------------------------------------------------------------- #
            # Magnetometer data
            self.mag_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.mag_msg.header.frame_id = self.frame_id
            self.mag_msg.magnetic_field.x = float(st.unpack('h', st.pack(
                'BB', buf[6], buf[7]))[0]) / self.mag_fact_
            self.mag_msg.magnetic_field.y = float(st.unpack('h', st.pack(
                'BB', buf[8], buf[9]))[0]) / self.mag_fact_
            self.mag_msg.magnetic_field.z = float(st.unpack('h', st.pack(
                'BB', buf[10], buf[11]))[0]) / self.mag_fact_
 
            # --------------------------------------------------------------- #
            # Kiwibot filtered data
            """
                For kiwibot the axes are reassigned as follows due to the
                orientation of the IMU in the robot.
                    * X -> Z
                    * Y -> X
                    * Z -> Y
            """

            self.imu_bot_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.imu_bot_msg.header.frame_id = self.frame_id
            self.imu_bot_msg.linear_acceleration.x = \
                self.imu_msg.linear_acceleration.z - gravity_z
            self.imu_bot_msg.linear_acceleration.y = \
                self.imu_msg.linear_acceleration.x - gravity_x
            self.imu_bot_msg.linear_acceleration.z = \
                self.imu_msg.linear_acceleration.y - gravity_y
            self.imu_bot_msg.linear_acceleration_covariance = [
                3e-1, 0.0, 0.0, 
                0.0, 3e-1, 0.0, 
                0.0, 0.0, 3e-1]
            self.imu_bot_msg.angular_velocity.x = self.imu_msg.angular_velocity.z
            self.imu_bot_msg.angular_velocity.y = self.imu_msg.angular_velocity.x
            self.imu_bot_msg.angular_velocity.z = self.imu_msg.angular_velocity.y
            self.imu_bot_msg.angular_velocity_covariance = [
                1e-6, 0.0, 0.0, 
                0.0, 1e-6, 0.0, 
                0.0, 0.0, 1e-6]

            # IMU Euler angles
            (pitch_imu, roll_imu, yaw_imu) = euler_from_quaternion(
                quaternion=quaternion_orig, axes='sxyz')

            # --------------------------------------------------------------- #
            # Finding the current azimuth (Yaw)
            """
                Azimuth is the angle (Between the current heading and the magnetic 
                north) in the horizontal plane
            """

            current_azimuth = \
                (math.atan2(self.mag_msg.magnetic_field.x, self.mag_msg.magnetic_field.z))

            if (self._azimuth != 0.0):
                current_azimuth = \
                    ((current_azimuth - self._azimuth) + math.pi) % \
                    (2.0 * math.pi) - math.pi + self._azimuth
                self._azimuth = ((1.0 - self._alpha) * current_azimuth) + \
                (self._alpha * self._azimuth)

                if (self._counter < 205):
                    self._counter += 1
            else:

                if (self.mag_msg.magnetic_field.x != 0.0):
                    self._azimuth = current_azimuth
                
                # Else condition to handle an error in the IMU when calculating the orientation
            
            if ((not self._first_orient) and (self._azimuth != 0.0) and (self._counter > 200)):
                self._initial_orient = -self._azimuth - self._mgn_decl
                self._alpha = 0.75
                self._first_orient = True

            # --------------------------------------------------------------- #
            # Reassigning the value for the Robot axes
            self._pitch_bot = (math.pi / 2) - pitch_imu
            self._roll_bot = -roll_imu
            self._yaw_bot  = math.atan2(math.sin(yaw_imu - math.pi), math.cos(yaw_imu - math.pi)) - self._initial_orient
            
            quaternion_bot = quaternion_from_euler(
                pitch=self._pitch_bot,
                roll=self._roll_bot,
                yaw=self._yaw_bot
            )
    
            # Kiwibot filtered data
            self.imu_bot_msg.orientation.x = quaternion_bot[0]
            self.imu_bot_msg.orientation.y = quaternion_bot[1]
            self.imu_bot_msg.orientation.z = quaternion_bot[2]
            self.imu_bot_msg.orientation.w = quaternion_bot[3]
            self.imu_bot_msg.orientation_covariance = [
                1e-5, 0.0, 0.0, 
                0.0, 1e-5, 0.0, 
                0.0, 0.0, 1e-3
            ]

            # --------------------------------------------------------------- #
            # Quaternion bot for EKF
            quaternion_bot_ekf = quaternion_from_euler(
                pitch=self._pitch_bot, 
                roll=-self._roll_bot,
                yaw=-self._azimuth + self._mgn_decl)
    
            # Kiwibot filtered data for EKF
            self.imu_ekf_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.imu_ekf_msg.header.frame_id = self.frame_id
            self.imu_ekf_msg.orientation.x = quaternion_bot_ekf[0]
            self.imu_ekf_msg.orientation.y = quaternion_bot_ekf[1]
            self.imu_ekf_msg.orientation.z = quaternion_bot_ekf[2]
            self.imu_ekf_msg.orientation.w = quaternion_bot_ekf[3]
            self.imu_ekf_msg.orientation_covariance = [
                1e-3, 0.0, 0.0, 
                0.0, 1e-3, 0.0, 
                0.0, 0.0, 1e-2
            ]
    
            # --------------------------------------------------------------- #
            # Publish IMU motion status
            self._acc_values = np.append(
                self._acc_values, abs(self.imu_bot_msg.linear_acceleration.x))
    
            self._move_counter += 1
            if (self._move_counter == 50):
                moving_now = np.mean(self._acc_values) > self._move_factor
    
                if (moving_now != self._moving_old):
                    self.move_msg.data = bool(moving_now)
                    self._move_pub = True
                
                self._moving_old = moving_now
                self._move_pub = False           
                self._acc_values = np.array([])
                self._move_counter = 0   
    
            # --------------------------------------------------------------- #
            # Temperature data
            self.tmp_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.tmp_msg.header.frame_id = self.frame_id
            self.tmp_msg.temperature = float(buf[44])
    
            self.status_msg.data = "IMU Ok"
    
        else:
            self.status_msg.data = "Unavailable data"

    def cb_publish_imu(self):
        """
            Desc: Publisher for the IMU node
        """

        # Function for updating IMU values
        self._update_imu_data()
        # Filtered data publisher
        self.pub_imu_data.publish(self.imu_msg)
        # Filtered data publisher
        if (self._counter > 200):
            self.pub_bot_data.publish(self.imu_bot_msg)
            self.pub_bot_ekf.publish(self.imu_ekf_msg)
        # Movement publisher
        if (self._move_pub == True):
            self.pub_bot_move.publish(self.move_msg)
        # Magnetometer data publisher
        self.pub_imu_mag.publish(self.mag_msg)
        # Temperature data publisher
        self.pub_imu_temp.publish(self.tmp_msg)
        # IMU status publisher
        self.pub_imu_status.publish(self.status_msg)

        """
        # Raw data publisher
        self.pub_imu_raw.publish(self.imu_raw_msg)
        """
        
# --------------------------------------------------------------------------- #
def main(args = None):
    # Node instantiation
    rclpy.init(args = args)
    imu_node = ImuNode()
    rclpy.spin(imu_node)
    
    # Closing Serial Port
    imu_node.ser.close()
    imu_node.destoy_node()
    rclpy.shutdown()

# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    main()