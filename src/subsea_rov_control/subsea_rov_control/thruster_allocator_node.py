"""ROS 2 node: body wrench / direct commands -> 8 thruster force commands.

Inputs (priority, highest first)
  cmd_thrust_direct  std_msgs/Float64MultiArray  one value per thruster, N
  cmd_wrench         geometry_msgs/WrenchStamped body frame (FLU), N / N.m

An input is used only while it is fresh (newer than ``input_timeout``).
With no fresh input every thruster is commanded to zero, which is also the
startup state.

Outputs
  thrusters/<name>/cmd      std_msgs/Float64  bridged to Gazebo cmd_thrust
  thrusters/command         std_msgs/Float64MultiArray  all 8, after saturation
  allocator/achieved_wrench geometry_msgs/WrenchStamped  B * u (what the
                            thrusters actually deliver, after saturation)
"""

from geometry_msgs.msg import WrenchStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64, Float64MultiArray

from .allocation import ThrusterAllocator, thrusters_from_arrays

SOURCE_NONE, SOURCE_WRENCH, SOURCE_DIRECT = 'none', 'wrench', 'direct'


class ThrusterAllocatorNode(Node):
    def __init__(self):
        super().__init__('thruster_allocator')
        self.declare_parameter('thruster_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('thruster_positions', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('thruster_directions', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('thruster_groups', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('max_thrust', 15.0)
        self.declare_parameter('min_thrust', -15.0)
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('input_timeout', 0.5)

        gp = self.get_parameter
        thrusters = thrusters_from_arrays(
            gp('thruster_names').value, gp('thruster_positions').value,
            gp('thruster_directions').value, gp('thruster_groups').value,
            gp('max_thrust').value, gp('min_thrust').value)
        self.alloc = ThrusterAllocator(thrusters)
        self.timeout = gp('input_timeout').value

        self.thrust_pubs = [self.create_publisher(Float64, f'thrusters/{t.name}/cmd', 10)
                            for t in thrusters]
        self.vector_pub = self.create_publisher(Float64MultiArray, 'thrusters/command', 10)
        self.achieved_pub = self.create_publisher(WrenchStamped, 'allocator/achieved_wrench', 10)

        self.create_subscription(WrenchStamped, 'cmd_wrench', self.on_wrench, 10)
        self.create_subscription(Float64MultiArray, 'cmd_thrust_direct', self.on_direct, 10)

        self.wrench = None
        self.wrench_time = None
        self.direct = None
        self.direct_time = None
        self.source = SOURCE_NONE

        self.create_timer(1.0 / gp('rate_hz').value, self.on_timer)
        limits = ', '.join(f'{v:.1f}' for v in self.alloc.max_wrench())
        self.get_logger().info(
            f'{len(thrusters)} thrusters, groups {sorted(self.alloc.groups)}; '
            f'max single-axis wrench [{limits}]')

    def on_wrench(self, msg: WrenchStamped) -> None:
        f, t = msg.wrench.force, msg.wrench.torque
        self.wrench = np.array([f.x, f.y, f.z, t.x, t.y, t.z])
        self.wrench_time = self.get_clock().now()

    def on_direct(self, msg: Float64MultiArray) -> None:
        n = len(self.alloc.names)
        if len(msg.data) != n:
            self.get_logger().error(
                f'cmd_thrust_direct needs {n} values, got {len(msg.data)}; ignored',
                throttle_duration_sec=2.0)
            return
        self.direct = np.clip(np.asarray(msg.data, dtype=float),
                              self.alloc.min_thrust, self.alloc.max_thrust)
        self.direct_time = self.get_clock().now()

    def _fresh(self, stamp) -> bool:
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9 < self.timeout

    def on_timer(self) -> None:
        if self._fresh(self.direct_time):
            source, thrust = SOURCE_DIRECT, self.direct
        elif self._fresh(self.wrench_time):
            source = SOURCE_WRENCH
            thrust, _ = self.alloc.allocate(self.wrench)
        else:
            source, thrust = SOURCE_NONE, np.zeros(len(self.alloc.names))

        if source != self.source:
            self.get_logger().info(f'command source: {self.source} -> {source}')
            self.source = source

        for pub, value in zip(self.thrust_pubs, thrust):
            pub.publish(Float64(data=float(value)))
        self.vector_pub.publish(Float64MultiArray(data=[float(v) for v in thrust]))

        achieved = self.alloc.wrench_from_thrust(thrust)
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        (msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
         msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z) = map(float, achieved)
        self.achieved_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterAllocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
