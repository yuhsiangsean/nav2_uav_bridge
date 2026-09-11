import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import math

from px4_msgs.msg import VehicleLocalPosition
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class OdomBridge(Node):
    def __init__(self):

        super().__init__('odom_bridge')
        self.create_subscription(   
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            qos,
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def position_callback(self, msg: VehicleLocalPosition): 
        # msg.x, msg.y, msg.z         → NED 位置 (north, east, down)
        # msg.vx, msg.vy, msg.vz      → NED 速度
        # msg.heading                 → NED yaw（從正北算起，順時針為正）
        enu_x = msg.y          # east
        enu_y = msg.x          # north
        enu_z = -msg.z         # up

        enu_vx = msg.vy
        enu_vy = msg.vx
        enu_vz = -msg.vz

        yaw = math.pi / 2.0 - msg.heading
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = enu_x
        odom.pose.pose.position.y = enu_y
        odom.pose.pose.position.z = enu_z
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = enu_vx
        odom.twist.twist.linear.y = enu_vy
        odom.twist.twist.linear.z = enu_vz

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = enu_x
        t.transform.translation.y = enu_y
        t.transform.translation.z = enu_z
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()