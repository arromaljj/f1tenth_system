import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuLowPassFilter(Node):
    def __init__(self):
        super().__init__('imu_lowpass_filter')
        self.declare_parameter('cutoff_frequency', 5.0)
        self.declare_parameter('imu_topic_in', '/sensors/imu/raw')
        self.declare_parameter('imu_topic_out', '/sensors/imu/filtered')

        self.cutoff = self.get_parameter('cutoff_frequency').value
        self.imu_pub = self.create_publisher(Imu,
            self.get_parameter('imu_topic_out').value, 10)
        self.subscription = self.create_subscription(
            Imu,
            self.get_parameter('imu_topic_in').value,
            self.listener_callback,
            10)
        self.prev = None

    def listener_callback(self, msg: Imu):
        if self.prev is None:
            self.prev = msg
        # Simple exponential smoothing filter
        alpha = min(max(self.cutoff / (self.cutoff + 1.0), 0.0), 1.0)
        filt = Imu()
        filt.header = msg.header
        filt.orientation.x = alpha * msg.orientation.x + (1 - alpha) * self.prev.orientation.x
        filt.orientation.y = alpha * msg.orientation.y + (1 - alpha) * self.prev.orientation.y
        filt.orientation.z = alpha * msg.orientation.z + (1 - alpha) * self.prev.orientation.z
        filt.orientation.w = alpha * msg.orientation.w + (1 - alpha) * self.prev.orientation.w
        filt.angular_velocity.x = alpha * msg.angular_velocity.x + (1 - alpha) * self.prev.angular_velocity.x
        filt.angular_velocity.y = alpha * msg.angular_velocity.y + (1 - alpha) * self.prev.angular_velocity.y
        filt.angular_velocity.z = alpha * msg.angular_velocity.z + (1 - alpha) * self.prev.angular_velocity.z
        filt.linear_acceleration.x = alpha * msg.linear_acceleration.x + (1 - alpha) * self.prev.linear_acceleration.x
        filt.linear_acceleration.y = alpha * msg.linear_acceleration.y + (1 - alpha) * self.prev.linear_acceleration.y
        filt.linear_acceleration.z = alpha * msg.linear_acceleration.z + (1 - alpha) * self.prev.linear_acceleration.z
        self.prev = filt
        self.imu_pub.publish(filt)

def main(args=None):
    rclpy.init(args=args)
    node = ImuLowPassFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
