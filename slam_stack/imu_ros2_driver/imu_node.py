import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time
import smbus2
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
            # Check if device exists
            import os
            i2c_dev = '/dev/i2c-5'
            self.get_logger().info(f"Checking for {i2c_dev}...")
            if os.path.exists(i2c_dev):
                self.get_logger().info(f"{i2c_dev} exists")
            else:
                self.get_logger().error(f"{i2c_dev} does not exist!")
                # Try other i2c devices
                import glob
                devices = sorted(glob.glob('/dev/i2c-*'))
                self.get_logger().info(f"Available I2C devices: {devices}")
                if devices:
                    i2c_dev = devices[-1]
                    self.get_logger().info(f"Using {i2c_dev} instead")
            
            # Setup I2C Bus 5 using smbus2 directly
            self.get_logger().info(f"Opening I2C Bus {i2c_dev}...")
            i2c = smbus2.SMBus(5)
            
            # Scan for BNO085 device - try common addresses
            bno_addresses = [0x4a, 0x4b, 0x36, 0x39, 0x3c, 0x3d]
            found_address = None
            
            for addr in bno_addresses:
                try:
                    i2c.read_byte(addr)
                    found_address = addr
                    self.get_logger().info(f"BNO085 found at address: 0x{addr:02x}")
                    break
                except:
                    pass
            
            if found_address is None:
                self.get_logger().error("BNO085 not found on any expected address")
                raise RuntimeError("BNO085 device not found")
            
            i2c.close()
            
            # Now use the correct I2C interface with adafruit
            from adafruit_extended_bus import ExtendedI2C as I2C
            i2c_obj = I2C(5)
            
            self.get_logger().info(f"Initializing BNO08X at address 0x{found_address:02x}...")
            self.bno = BNO08X_I2C(i2c_obj)
            self.get_logger().info("BNO085 initialized successfully!")
            
            # Wait for device to settle after initialization
            time.sleep(0.5)

        except Exception as e:
            self.get_logger().error(f"BNO085 Initialization failed: {e}")
            raise e

        # Enable features with delays between each
        self.get_logger().info("Enabling sensor features...")
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        time.sleep(0.1)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        time.sleep(0.1)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        time.sleep(0.1)
        self.get_logger().info("Sensor features enabled")

        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.read_failures = 0
        self.last_quat = None
        self.stale_count = 0
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

    def timer_callback(self):
        try:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            # Read quaternion with retry and error recovery
            quat = None
            for attempt in range(3):
                try:
                    quat = self.bno.quaternion
                    if quat is not None and len(quat) >= 4:
                        break
                except Exception as e:
                    if attempt < 2:
                        time.sleep(0.002)
                    
            if quat is None or len(quat) < 4:
                self.stale_count += 1
                if self.stale_count > 50:
                    self.get_logger().error("Device unresponsive, attempting recovery...")
                    self._attempt_recovery()
                    self.stale_count = 0
                return
            
            # Check for stale data (identical quaternion for too long)
            if self.last_quat is not None and self.last_quat == quat:
                self.stale_count += 1
                if self.stale_count > 30:
                    self.get_logger().warn(f"Stale data detected ({self.stale_count} repeats), resetting device...")
                    self._attempt_recovery()
                    self.stale_count = 0
                    return
            else:
                self.stale_count = 0
                
            self.last_quat = tuple(quat)
            
            msg.orientation.x = float(quat[0])
            msg.orientation.y = float(quat[1])
            msg.orientation.z = float(quat[2])
            msg.orientation.w = float(quat[3])

            # Read gyroscope with retry
            gyro = None
            for attempt in range(3):
                try:
                    gyro = self.bno.gyro
                    if gyro is not None and len(gyro) >= 3:
                        break
                except:
                    if attempt < 2:
                        time.sleep(0.001)
                        
            if gyro is None or len(gyro) < 3:
                return
                
            msg.angular_velocity.x = float(gyro[0])
            msg.angular_velocity.y = float(gyro[1])
            msg.angular_velocity.z = float(gyro[2])

            # Read accelerometer with retry
            accel = None
            for attempt in range(3):
                try:
                    accel = self.bno.acceleration
                    if accel is not None and len(accel) >= 3:
                        break
                except:
                    if attempt < 2:
                        time.sleep(0.001)
                        
            if accel is None or len(accel) < 3:
                return
                
            msg.linear_acceleration.x = float(accel[0])
            msg.linear_acceleration.y = float(accel[1])
            msg.linear_acceleration.z = float(accel[2])

            # Set covariance matrices (unknown variance)
            msg.orientation_covariance = [0.0] * 9
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity_covariance = [0.0] * 9
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration_covariance = [0.0] * 9
            msg.linear_acceleration_covariance[0] = -1.0

            self.publisher_.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f"Unexpected error in timer callback: {e}")

    def _attempt_recovery(self):
        """Attempt to recover from device lockup by re-enabling features"""
        try:
            self.get_logger().info("Attempting device recovery...")
            time.sleep(0.1)
            
            # Re-enable features to kick the device back into action
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            time.sleep(0.05)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            time.sleep(0.05)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            time.sleep(0.05)
            
            self.get_logger().info("Device recovery complete")
            self.last_quat = None
        except Exception as e:
            self.get_logger().error(f"Recovery failed: {e}")

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