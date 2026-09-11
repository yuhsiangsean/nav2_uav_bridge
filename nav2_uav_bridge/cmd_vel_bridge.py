
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import math

from geometry_msgs.msg import Twist
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition

px4_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.position_callback, px4_qos)

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)

        self.current_yaw = 0.0
        self.current_z = 0.0
        self.latest_cmd = Twist()
        self.tick_count = 0
        self.offboard_engaged = False
        self.airborne = False

        self.TAKEOFF_ALTITUDE = -2.0   # NED，負值代表高度（2 公尺高）
        self.ALT_TOLERANCE = 0.3

        self.create_timer(0.1, self.timer_callback)

    def cmd_vel_callback(self, msg: Twist):
        self.latest_cmd = msg

    def position_callback(self, msg: VehicleLocalPosition):
        self.current_yaw = msg.heading
        self.current_z = msg.z

    def timer_callback(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = True
        self.offboard_pub.publish(offboard_msg)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = offboard_msg.timestamp
        setpoint.position = [float('nan')] * 3
        setpoint.yaw = float('nan')

        if not self.offboard_engaged:
            setpoint.velocity = [0.0, 0.0, 0.0]
            setpoint.yawspeed = 0.0
        elif not self.airborne:
            setpoint.velocity = [0.0, 0.0, -0.5]
            setpoint.yawspeed = 0.0
            if self.current_z <= self.TAKEOFF_ALTITUDE + self.ALT_TOLERANCE:
                self.airborne = True
        else:
            vx_body = self.latest_cmd.linear.x
            vy_body = self.latest_cmd.linear.y
            yaw = self.current_yaw

            north = vx_body * math.cos(yaw) + vy_body * math.sin(yaw)
            east = vx_body * math.sin(yaw) - vy_body * math.cos(yaw)

            setpoint.velocity = [north, east, 0.0]
            setpoint.yawspeed = -self.latest_cmd.angular.z

        self.setpoint_pub.publish(setpoint)

        self.tick_count += 1
        if self.tick_count == 20 and not self.offboard_engaged:
            self.engage_offboard()
            self.arm()
            self.offboard_engaged = True

    def engage_offboard(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def arm(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 0
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()