import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FakeScanPublisher(Node):
    def __init__(self):
        super().__init__('fake_scan_publisher')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_timer(0.1, self.publish_scan)

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.angle_min = -3.14159
        msg.angle_max = 3.14159
        msg.angle_increment = 0.0175
        msg.range_min = 0.1
        msg.range_max = 10.0
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        msg.ranges = [msg.range_max] * num_readings
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeScanPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
