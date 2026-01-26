import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import digitalio
import time
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR
)

class BNO085Driver(Node):
    def __init__(self):
        super().__init__('bno085_driver')
        
        """
        Hardware Pin Setup
        1. Reset Pin (Pin 11)     -  LOW=Reset, HIGH=Normal Operation
        2. Interrupt Pin (Pin 13) -  HIGH=Idle, LOW=Data Ready
        """
        self.rst = digitalio.DigitalInOut(board.D11)
        self.rst.direction = digitalio.Direction.OUTPUT
        self.int_pin = digitalio.DigitalInOut(board.D13)
        self.int_pin.direction = digitalio.Direction.INPUT

        self.bno = None
        self.last_update_time = time.time()

        # Trigger BNO085 Initialization
        self.init_sensor()

        # ROS 2 Setup
        self.publisher_ = self.create_publisher(Imu, '/fastbot/imu', 10)
        self.timer = self.create_timer(0.01, self.timer_callback) # 100Hz

        self.get_logger().warn("BNO085-IMU Driver Node Started")

    def init_sensor(self):
        """Physically resets and (re)enables the IMU with bus-clearance logic."""
        try:
            # Clean power state
            self.rst.value = False
            time.sleep(0.5)
            self.rst.value = True
            time.sleep(1.0) # Wait for internal bootloader to finish
            
            # (Re)Instantiate the I2C connection and enable features
            self.i2c_obj = I2C(5) # bus 5 on Opi5B
            
            self.bno = BNO08X_I2C(self.i2c_obj, address=0x4a)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            # self.bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR) # ignore magnetometer
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            
            self.last_update_time = time.time()
            self.get_logger().info("BNO085 is online")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize BNO085: {e}")
            self.last_update_time = time.time() - 1.0

    def timer_callback(self):
        """Polls the BNO085 for new data and publishes it as a ROS2 Imu message."""

        # WATCHDOG: If no data for 2 seconds, the I2C bus is likely hung
        if (time.time() - self.last_update_time) > 2.0:
            self.get_logger().warn("IMU Stall Detected! Triggering Recovery...")
            self.init_sensor()
            return

        # GATEKEEPER: Check the INT pin
        # If it's HIGH, the BNO085 has no new data - Do NOT touch the I2C bus
        if self.int_pin.value:
            return

        try:
            quat = self.bno.quaternion
            accel = self.bno.acceleration
            gyro = self.bno.gyro

            if quat:
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'imu_link'
                
                msg.orientation.x = float(quat[0])
                msg.orientation.y = float(quat[1])
                msg.orientation.z = float(quat[2])
                msg.orientation.w = float(quat[3])
                
                msg.linear_acceleration.x = float(accel[0])
                msg.linear_acceleration.y = float(accel[1])
                msg.linear_acceleration.z = float(accel[2])
                
                msg.angular_velocity.x = float(gyro[0])
                msg.angular_velocity.y = float(gyro[1])
                msg.angular_velocity.z = float(gyro[2])
                
                self.publisher_.publish(msg)
                self.last_update_time = time.time() # Pet the watchdog

        except Exception:
            # ignore small I2C glitches and let the watchdog handle the total lockups.
            pass

def main(args=None):
    rclpy.init(args=args)
    node = BNO085Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()