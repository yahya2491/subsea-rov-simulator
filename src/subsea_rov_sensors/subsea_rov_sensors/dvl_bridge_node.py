"""ROS 2 node: relay Gazebo's DVL sensor into ROS 2.

ros_gz_bridge has no mapping for gz.msgs.DVLVelocityTracking, so this node
subscribes over gz-transport directly and republishes:

  dvl/twist     geometry_msgs/TwistWithCovarianceStamped  bottom-track velocity
                of the DVL head over ground, DVL frame (FLU, aligned with
                base_link); only published while bottom lock is valid
  dvl/altitude  sensor_msgs/Range  altitude above the seabed

The DVL itself is simulated by Gazebo (gz-sim-dvl-system): it ray-casts its
four beams against the rendered seabed, so altitude, bottom lock and the
loss of lock out of range all come from the real terrain.
"""

import math
import threading

from geometry_msgs.msg import TwistWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

try:
    from gz.msgs10.dvl_velocity_tracking_pb2 import DVLVelocityTracking
    from gz.transport13 import Node as GzNode
except ImportError as exc:  # pragma: no cover - depends on the Gazebo install
    raise SystemExit(
        'dvl_bridge needs the Gazebo Harmonic Python bindings (python3-gz-transport13, '
        f'python3-gz-msgs10): {exc}')


class DvlBridgeNode(Node):
    def __init__(self):
        super().__init__('dvl_bridge')
        dp = self.declare_parameter
        dp('gz_topic', '/model/dummy_rov/dvl/velocity')
        dp('frame_id', 'dvl')
        dp('min_locked_beams', 3)
        dp('velocity_std_floor', 0.005)
        dp('beam_tilt_deg', 25.0)
        dp('min_range', 0.05)
        dp('max_range', 50.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.frame_id = g('frame_id')
        self.min_beams = g('min_locked_beams')
        self.var_floor = g('velocity_std_floor') ** 2
        self.cos_tilt = math.cos(math.radians(g('beam_tilt_deg')))
        self.min_range, self.max_range = g('min_range'), g('max_range')

        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, 'dvl/twist',
                                               qos_profile_sensor_data)
        self.alt_pub = self.create_publisher(Range, 'dvl/altitude', qos_profile_sensor_data)
        self.lock = threading.Lock()
        self.locked = None
        self.counts = {'valid': 0, 'no_lock': 0}

        self.gz = GzNode()
        topic = g('gz_topic')
        if not self.gz.subscribe(DVLVelocityTracking, topic, self.on_dvl):
            raise RuntimeError(f'could not subscribe to gz topic {topic}')
        self.get_logger().info(f'relaying DVL from gz topic {topic}')

    def on_dvl(self, msg) -> None:
        """Handle one DVL sample (called on a gz-transport thread)."""
        locked = [b for b in msg.beams if b.locked]
        valid = len(locked) >= self.min_beams and msg.HasField('velocity')
        with self.lock:
            if valid != self.locked:
                self.get_logger().info('DVL bottom lock ' + ('acquired' if valid else 'lost'))
                self.locked = valid
            self.counts['valid' if valid else 'no_lock'] += 1

        stamp = rclpy.time.Time(seconds=msg.header.stamp.sec,
                                nanoseconds=msg.header.stamp.nsec).to_msg()
        if locked:
            alt = Range()
            alt.header.stamp = stamp
            alt.header.frame_id = self.frame_id
            alt.radiation_type = Range.ULTRASOUND
            alt.field_of_view = 2.0 * math.acos(self.cos_tilt)
            alt.min_range, alt.max_range = self.min_range, self.max_range
            # Vertical altitude from the mean slant range of the locked beams.
            alt.range = float(sum(b.range.mean for b in locked) / len(locked) * self.cos_tilt)
            self.alt_pub.publish(alt)
        if not valid:
            return

        out = TwistWithCovarianceStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.frame_id
        v = msg.velocity.mean
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = \
            v.x, v.y, v.z
        cov = [0.0] * 36
        c = list(msg.velocity.covariance)
        for i in range(3):
            for j in range(3):
                cov[6 * i + j] = c[3 * i + j] if len(c) == 9 else 0.0
            cov[7 * i] += self.var_floor
        for i in range(3, 6):
            cov[7 * i] = -1.0   # angular part not measured
        out.twist.covariance = cov
        self.twist_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DvlBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
