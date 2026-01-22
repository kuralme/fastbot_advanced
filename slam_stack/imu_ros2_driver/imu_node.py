import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time
import board
import digitalio
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR
)

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('bno085_imu_driver')
        
        try:
            # 1. Setup Physical Reset Pin (Pin 11 / GPIO4_B2)
            self.get_logger().info("Performing hardware reset on Pin 11...")
            reset_pin = digitalio.DigitalInOut(board.D11)
            reset_pin.direction = digitalio.Direction.OUTPUT
            reset_pin.value = False # Pull low to reset
            time.sleep(0.2)
            reset_pin.value = True  # Pull high to run
            time.sleep(0.5) # Wait for BNO bootloader

            # 2. Setup I2C Bus 5
            i2c = I2C(5)
            
            # 3. Initialize with Interrupt Pin (Pin 13 / GPIO4_B3)
            # The library will now wait for Pin 13 to go LOW before trying to read.
            # This prevents the "No such device" timeout error.
            self.get_logger().info("Initializing BNO08X with INT pin 13...")
            int_pin = digitalio.DigitalInOut(board.D13)
            
            self.bno = BNO08X_I2C(i2c, reset_pin=None, int_pin=int_pin)
            self.get_logger().info("BNO085 Hardware Handshake Successful!")

        except Exception as e:
            self.get_logger().error(f"BNO085 Critical Failure: {e}")
            raise e

        # Enable features
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        try:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            quat = self.bno.quaternion
            msg.orientation.x = float(quat[0])
            msg.orientation.y = float(quat[1])
            msg.orientation.z = float(quat[2])
            msg.orientation.w = float(quat[3])

            gyro = self.bno.gyro
            msg.angular_velocity.x = float(gyro[0])
            msg.angular_velocity.y = float(gyro[1])
            msg.angular_velocity.z = float(gyro[2])

            accel = self.bno.acceleration
            msg.linear_acceleration.x = float(accel[0])
            msg.linear_acceleration.y = float(accel[1])
            msg.linear_acceleration.z = float(accel[2])

            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()