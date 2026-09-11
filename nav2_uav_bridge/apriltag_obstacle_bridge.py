import struct

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

TAG_FRAMES = ['tag36h11:0']
MAX_TRANSFORM_AGE = 0.3  # seconds; discard stale detections instead of re-marking old positions


class AprilTagObstacleBridge(Node):
    def __init__(self):
        super().__init__('apriltag_obstacle_bridge')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PointCloud2, '/apriltag_obstacles', 10)

        self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        points = []
        now = self.get_clock().now()
        for frame in TAG_FRAMES:
            try:
                t = self.tf_buffer.lookup_transform('map', frame, Time())
                age = (now - Time.from_msg(t.header.stamp)).nanoseconds / 1e9
                if age > MAX_TRANSFORM_AGE:
                    continue
                p = t.transform.translation
                points.append((p.x, p.y, p.z))
            except (LookupException, ConnectivityException, ExtrapolationException):
                continue

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * max(len(points), 1)
        msg.is_dense = True
        buf = bytearray()
        for x, y, z in points:
            buf += struct.pack('fff', x, y, z)
        msg.data = bytes(buf)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagObstacleBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
