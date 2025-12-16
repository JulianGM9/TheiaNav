import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import IMU
import time
import math

RAD_TO_DEG = 57.29578
G_GAIN = 0.070  # deg/s/LSB
AA = 0.40       # Complementary filter constant

class BerryIMUPublisher(Node):
    def __init__(self):
        super().__init__("berryimu_publisher")
        self.publisher_ = self.create_publisher(Imu, "/imu_raw", 10)
        self.timer = self.create_timer(0.02, self.publish_imu)

        # Initialize IMU
        IMU.detectIMU()
        if IMU.BerryIMUversion == 99:
            self.get_logger().error("No BerryIMU found. Exiting.")
            exit()
        IMU.initIMU()

        self.gyroXangle = 0.0
        self.gyroYangle = 0.0
        self.gyroZangle = 0.0
        self.CFangleX = 0.0
        self.CFangleY = 0.0
        self.last_time = time.time()
        self.get_logger().info("BerryIMU ROS2 Publisher started")

    def publish_imu(self):
        # Timing
        now = time.time()
        LP = now - self.last_time
        self.last_time = now

        # Read sensors
        ACCx = IMU.readACCx()
        ACCy = IMU.readACCy()
        ACCz = IMU.readACCz()
        GYRx = IMU.readGYRx()
        GYRy = IMU.readGYRy()
        GYRz = IMU.readGYRz()
        #MAGx = IMU.readMAGx()
        #MAGy = IMU.readMAGy()
        #MAGz = IMU.readMAGz()
        #heading = math.atan2(MAGy, MAGx)  # radians

        # Gyro rates in deg/s
        rate_gyr_x = GYRx * G_GAIN
        rate_gyr_y = GYRy * G_GAIN
        rate_gyr_z = GYRz * G_GAIN

        # Integrate gyro
        self.gyroXangle += rate_gyr_x * LP
        self.gyroYangle += rate_gyr_y * LP
        self.gyroZangle += rate_gyr_z * LP

        # Accelerometer angles
        AccXangle = math.atan2(ACCy, ACCz) * RAD_TO_DEG
        AccYangle = math.atan2(ACCz, ACCx) * RAD_TO_DEG
        if AccYangle > 90:
            AccYangle -= 270
        else:
            AccYangle += 90

        # Complementary filter
        self.CFangleX = AA*(self.CFangleX + rate_gyr_x*LP) + (1-AA)*AccXangle
        self.CFangleY = AA*(self.CFangleY + rate_gyr_y*LP) + (1-AA)*AccYangle
        #CFangleZ = AA*(self.gyroZangle + rate_gyr_z*LP) + (1-AA)*heading*RAD_TO_DEG

        # Create IMU message
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Linear acceleration (in g)
        msg.linear_acceleration.x = float(ACCx)
        msg.linear_acceleration.y = float(ACCy)
        msg.linear_acceleration.z = float(ACCz)

        # Angular velocity (deg/s -> rad/s)
        from math import pi
        msg.angular_velocity.x = float(rate_gyr_x) * pi/180
        msg.angular_velocity.y = float(rate_gyr_y) * pi/180
        msg.angular_velocity.z = float(rate_gyr_z) * pi/180

        # Orientation unknown
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BerryIMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
