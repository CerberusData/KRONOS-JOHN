import rclpy 
import serial
import sys
import struct as st
import binascii

from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.parameter import get_parameter

from time import time
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import MagneticField

from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue


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

#  Operation modes
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

#  Power modes
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

        super().__init__("ImuNode")

        # Publishers
        self.pub_data_ = self.create_publisher(msg_type = Imu, topic = "imu/data", queue_size = 1)
        self.pub_raw_ = self.create_publisher(msg_type = Imu, topic = "imu/raw", queue_size = 1)
        self.pub_mag_ = self.create_publisher(msg_type = MagneticField, topic = "imu/mag", queue_size = 1)
        self.pub_temp_ = self.create_publisher(msg_type = Temperature, topic = "imu/temp", queue_size = 1)

        # Get parameters
        self.port = self.get_parameter("port").value 
        self.frame_id_ = self.get_parameter("frame_id").value
        self.frequency_ = self.get_parameter("frequency").value
        self.opr_mode_ = self.get_parameter("operation_mode").value

        # Data objects
        self.raw_msg = Imu()           # Raw data
        self.imu_msg = Imu()           # Filtered data
        self.tmp_msg = Temperature()   # Temperature
        self.mag_msg = MagneticField() # Magnetometer data

        # IMU factors
        self.seq_ = 0
        self.acc_fact_ = 1000.0
        self.mag_fact_ = 16.0
        self.gyr_fact_ = 900.0

        # Open Serial Port
        self.get_logger.info("Opening Serial Port:  %s", self.port)
        try:
            self.ser = serial.Serial(port = self.port, baudrate = 115200, timeout = 0.02)
        except serial.serialutil.SerialException:
            self.get_logger.error("IMU not found at port " + self.port + ". Check the port number")
            sys.exit(0)

        # Check IMU ID
        buf = self.read_from_dev(CHIP_ID, 1)
        if (buf == 0) or (buf[0] != BNO055_ID):
            sys.exit(0)

        # IMU Setup
        if not(self.write_to_dev(OPER_MODE, 1, OPER_MODE_CONFIG)):
            self.get_logger.error("Unable to set IMU into configuration mode.")

        if not(self.write_to_dev(PWR_MODE, 1, PWR_MODE_NORMAL)):
            self.get_logger.error("Unable to set IMU normal power mode.")

        if not(self.write_to_dev(PAGE_ID, 1, 0x00)):
            self.get_logger.error("Unable to set IMU register page 0.")

        if not(self.write_to_dev(SYS_TRIGGER, 1, 0x00)):
            self.get_logger.error("Unable to start IMU.")

        if not(self.write_to_dev(UNIT_SEL, 1, 0x83)):
            self.get_logger.error("Unable to set IMU units.")

        if not(self.write_to_dev(AXIS_MAP_CONFIG, 1, 0x24)):
            self.get_logger.error("Unable to remap IMU axis.")

        if not(self.write_to_dev(AXIS_MAP_SIGN, 1, 0x06)):
            self.get_logger.error("Unable to set IMU axis signs.")

        if not(self.write_to_dev(OPER_MODE, 1, OPER_MODE_NDOF))
            self.get_logger.error("Unable to set IMU operation into operation mode.")
        
        self.get_logger.info("Bosch BNO055 IMU configuration completed.")
        
        # Timers
        self.pub_timer_ = self.create_timer(time_period_sec = 0.1, callback = self.cb_publish_imu)



    def update_imu_data(self):
        buf = self.read_from_dev(ACCEL_DATA, 45)
        if buf != 0:
            # Raw data
            self.raw_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.raw_msg.header.frame_id = self.frame_id_
            self.raw_msg.header.seq = self.seq_
            self.raw_msg.orientation_covariance[0] = -1
            self.raw_msg.linear_acceleration.x = float(st.unpack('h', st.pack('BB', buf[0], buf[1]))[0]) / self.acc_fact_
            self.raw_msg.linear_acceleration.y = float(st.unpack('h', st.pack('BB', buf[2], buf[3]))[0]) / self.acc_fact_
            self.raw_msg.linear_acceleration.z = float(st.unpack('h', st.pack('BB', buf[4], buf[5]))[0]) / self.acc_fact_
            self.raw_msg.linear_acceleration_covariance[0] = -1
            self.raw_msg.angular_velocity.x = float(st.unpack('h', st.pack('BB', buf[12], buf[13]))[0]) / self.gyr_fact_
            self.raw_msg.angular_velocity.y = float(st.unpack('h', st.pack('BB', buf[14], buf[15]))[0]) / self.gyr_fact_
            self.raw_msg.angular_velocity.z = float(st.unpack('h', st.pack('BB', buf[16], buf[17]))[0]) / self.gyr_fact_
            self.raw_msg.angular_velocity_covariance[0] = -1

            # ------------------------------------------------------------ #
            # Filtered data
            self.imu_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.imu_msg.header.frame_id = self.frame_id_
            self.imu_msg.header.seq = self.seq_
            self.imu_msg.orientation.w = float(st.unpack('h', st.pack('BB', buf[24], buf[25]))[0])
            self.imu_msg.orientation.x = float(st.unpack('h', st.pack('BB', buf[26], buf[27]))[0])
            self.imu_msg.orientation.y = float(st.unpack('h', st.pack('BB', buf[28], buf[29]))[0])
            self.imu_msg.orientation.z = float(st.unpack('h', st.pack('BB', buf[30], buf[31]))[0])
            self.imu_msg.linear_acceleration.x = float(st.unpack('h', st.pack('BB', buf[32], buf[33]))[0]) / self.acc_fact_
            self.imu_msg.linear_acceleration.y = float(st.unpack('h', st.pack('BB', buf[34], buf[35]))[0]) / self.acc_fact_
            self.imu_msg.linear_acceleration.z = float(st.unpack('h', st.pack('BB', buf[36], buf[37]))[0]) / self.acc_fact_
            self.imu_msg.linear_acceleration_covariance[0] = -1
            self.imu_msg.angular_velocity.x = float(st.unpack('h', st.pack('BB', buf[12], buf[13]))[0]) / self.gyr_fact_
            self.imu_msg.angular_velocity.y = float(st.unpack('h', st.pack('BB', buf[14], buf[15]))[0]) / self.gyr_fact_
            self.imu_msg.angular_velocity.z = float(st.unpack('h', st.pack('BB', buf[16], buf[17]))[0]) / self.gyr_fact_
            self.imu_msg.angular_velocity_covariance[0] = -1

            # ------------------------------------------------------------ #
            # Magnetometer data
            self.mag_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.mag_msg.header.frame_id = self.frame_id_
            self.mag_msg.header.seq = self.seq_
            self.mag_msg.magnetic_field.x = float(st.unpack('h', st.pack('BB', buf[6], buf[7]))[0]) / self.mag_fact_
            self.mag_msg.magnetic_field.y = float(st.unpack('h', st.pack('BB', buf[8], buf[9]))[0]) / self.mag_fact_
            self.mag_msg.magnetic_field.z = float(st.unpack('h', st.pack('BB', buf[10], buf[11]))[0]) / self.mag_fact_
  
            # ------------------------------------------------------------ #
            # Temperatura data
            self.tmp_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.tmp_msg.header.frame_id = self.frame_id_
            self.tmp_msg.header.seq = self.seq_
            self.tmp_msg.temperature = buf[44]

            self.seq_ += 1


    def cb_publish_imu(self):
        self.update_imu_data()

        self.get_logger.info("Publishing!")
        # Publishing data
        self.pub_data_.publish(self.imu_msg)    # Filtered data
        self.pub_raw_.publish(self.raw_msg)     # Raw data
        self.pub_mag_.publish(self.mag_msg)     # Magnetometer data
        self.pub_temp_.publish(self.tmp_msg)    # Temperature data
    
    def read_from_dev(self, reg_addr, length):
        buf_out = bytearray()
        buf_out.append(START_BYTE_WR)
        buf_out.append(WRITE)
        buf_out.append(reg_addr)
        buf_out.append(length)

        try: 
            self.ser.write(buf_out)
            buf_in = bytearray(self.ser.read(2 + length))
        except:
            return 0
        
        # Response checking
        if (bug_in.__len__ != (2 + length)) or (buf_in[0] != START_BYTE_RESP):
            return 0
        
        # Can I improve it?
        buf_in.pop(0)
        buf_in.pop(0)
        return buf_in

    def write_to_dev(self, reg_addr, length, data):
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

        if (buf_in.__len__() != 2) or (buf_in[1] != 0x01):
            return False
        
        return True

def main(args = None):
    rclpy.init(args = args)

    imu_node = ImuNode()
    rclpy.spin(imu_node)
    
    # Closing Serial Port
    imu_node.ser.close()

    imu_node.destoy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
   
    
