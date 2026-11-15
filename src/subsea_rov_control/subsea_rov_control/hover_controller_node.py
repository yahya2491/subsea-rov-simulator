"""ROS 2 node: closed-loop hover on the navigation estimate.

Subscribes
  nav/odom         nav_msgs/Odometry          pose in world, twist in body
  hover/setpoint   geometry_msgs/PoseStamped  world-frame position + heading
Publishes
  cmd_wrench            geometry_msgs/WrenchStamped  body-frame demand
  hover/active_setpoint geometry_msgs/PoseStamped    what is being held
  hover/error           std_msgs/Float64MultiArray   [x, y, z, roll, pitch, yaw]
Services
  hover/enable  std_srvs/SetBool  engage / disengage (engaging resets integrators)
  hover/hold    std_srvs/Trigger  hold the current estimated pose

The controller only ever sees the navigation estimate, never ground truth.
If the estimate goes stale it commands zero wrench until it recovers.
"""

import math

from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from .hover import AXES, HoverConfig, HoverController, HoverSetpoint, VehicleState
from .pid import PidGains


class HoverControllerNode(Node):
    def __init__(self):
        super().__init__('hover_controller')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('nav_timeout', 0.5)
        self.declare_parameter('auto_enable', True)
        self.declare_parameter('buoyancy_feedforward', 0.0)
        self.declare_parameter('wrench_limit', [40.0, 40.0, 55.0, 15.0, 15.0, 25.0])
        gains = {}
        for axis in AXES:
            vals = {}
            for term, default in (('kp', 0.0), ('ki', 0.0), ('kd', 0.0),
                                  ('i_limit', math.inf), ('out_limit', math.inf)):
                vals[term] = self.declare_parameter(f'gains.{axis}.{term}', default).value
            gains[axis] = PidGains(**vals)

        gp = self.get_parameter
        self.ctrl = HoverController(HoverConfig(
            gains=gains,
            buoyancy_feedforward=gp('buoyancy_feedforward').value,
            wrench_limit=gp('wrench_limit').value))
        self.nav_timeout = gp('nav_timeout').value
        self.enabled = False
        self.auto_enable = gp('auto_enable').value
        self.state = None
        self.state_time = None
        self.setpoint = None
        self.last_tick = None

        self.wrench_pub = self.create_publisher(WrenchStamped, 'cmd_wrench', 10)
        self.sp_pub = self.create_publisher(PoseStamped, 'hover/active_setpoint', 10)
        self.err_pub = self.create_publisher(Float64MultiArray, 'hover/error', 10)
        self.create_subscription(Odometry, 'nav/odom', self.on_odom, 10)
        self.create_subscription(PoseStamped, 'hover/setpoint', self.on_setpoint, 10)
        self.create_service(SetBool, 'hover/enable', self.on_enable)
        self.create_service(Trigger, 'hover/hold', self.on_hold)
        self.create_timer(1.0 / gp('rate_hz').value, self.on_timer)

    # ------------------------------------------------------------------ inputs
    def on_odom(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        quat = np.array([q.x, q.y, q.z, q.w])
        v_body = msg.twist.twist.linear
        w_body = msg.twist.twist.angular
        self.state = VehicleState(
            position=np.array([p.x, p.y, p.z]),
            orientation=quat,
            velocity_world=Rotation.from_quat(quat).apply([v_body.x, v_body.y, v_body.z]),
            angular_velocity_body=np.array([w_body.x, w_body.y, w_body.z]))
        self.state_time = self.get_clock().now()
        if self.auto_enable and not self.enabled:
            self.auto_enable = False
            self._hold_current()
            self._engage(True)

    def on_setpoint(self, msg: PoseStamped) -> None:
        if msg.header.frame_id not in ('', 'world'):
            self.get_logger().warn(f'setpoint frame "{msg.header.frame_id}" is not world; '
                                   'treating it as world')
        p, q = msg.pose.position, msg.pose.orientation
        yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('ZYX')[0]
        new = HoverSetpoint(position=np.array([p.x, p.y, p.z]), yaw=float(yaw))
        old = self.setpoint
        self.setpoint = new
        if old is None or not np.allclose(old.position, new.position, atol=1e-4) or \
                abs(old.yaw - new.yaw) > 1e-4:
            self.get_logger().info(
                f'setpoint ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) m, yaw {math.degrees(yaw):.1f} deg')

    def on_enable(self, req, resp):
        if req.data and self.setpoint is None:
            if self.state is None:
                resp.success, resp.message = False, 'no navigation estimate yet'
                return resp
            self._hold_current()
        self._engage(req.data)
        resp.success, resp.message = True, 'enabled' if req.data else 'disabled'
        return resp

    def on_hold(self, req, resp):
        if self.state is None:
            resp.success, resp.message = False, 'no navigation estimate yet'
            return resp
        self._hold_current()
        resp.success, resp.message = True, 'holding current pose'
        return resp

    # ----------------------------------------------------------------- helpers
    def _hold_current(self) -> None:
        yaw = Rotation.from_quat(self.state.orientation).as_euler('ZYX')[0]
        self.setpoint = HoverSetpoint(position=self.state.position.copy(), yaw=float(yaw))
        p = self.setpoint.position
        self.get_logger().info(f'holding ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}) m, '
                               f'yaw {math.degrees(yaw):.1f} deg')

    def _engage(self, on: bool) -> None:
        if on and not self.enabled:
            self.ctrl.reset()
            self.last_tick = None
        if not on and self.enabled:
            self._publish_wrench(np.zeros(6))
        self.enabled = on
        self.get_logger().info('hover ' + ('engaged' if on else 'disengaged'))

    def _publish_wrench(self, w) -> None:
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        (msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
         msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z) = map(float, w)
        self.wrench_pub.publish(msg)

    # -------------------------------------------------------------------- loop
    def on_timer(self) -> None:
        if not self.enabled or self.setpoint is None:
            return
        now = self.get_clock().now()
        stale = self.state_time is None or \
            (now - self.state_time).nanoseconds * 1e-9 > self.nav_timeout
        if stale:
            self.get_logger().warn('navigation estimate stale; commanding zero wrench',
                                   throttle_duration_sec=2.0)
            self._publish_wrench(np.zeros(6))
            self.last_tick = None
            return

        dt = 0.0 if self.last_tick is None else (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now
        dt = min(max(dt, 0.0), 0.1)
        wrench = self.ctrl.update(self.state, self.setpoint, dt)
        self._publish_wrench(wrench)

        sp = PoseStamped()
        sp.header.stamp = now.to_msg()
        sp.header.frame_id = 'world'
        sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = map(
            float, self.setpoint.position)
        qx, qy, qz, qw = Rotation.from_euler('z', self.setpoint.yaw).as_quat()
        sp.pose.orientation.x, sp.pose.orientation.y = float(qx), float(qy)
        sp.pose.orientation.z, sp.pose.orientation.w = float(qz), float(qw)
        self.sp_pub.publish(sp)
        self.err_pub.publish(Float64MultiArray(
            data=[float(self.ctrl.last_errors[a]) for a in AXES]))


def main(args=None):
    rclpy.init(args=args)
    node = HoverControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
