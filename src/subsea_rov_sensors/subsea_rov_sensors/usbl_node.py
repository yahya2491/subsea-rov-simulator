"""ROS 2 node: simulated USBL acoustic positioning.

Subscribes  ground_truth/odom  nav_msgs/Odometry (from Gazebo)
Publishes   usbl/fix           geometry_msgs/PoseWithCovarianceStamped

Each fix is the transponder position in the world frame. Only the position
part of the message is meaningful; orientation is identity with a huge
covariance. The header stamp is the time of the ping (when the position was
valid), and the message is published ``latency`` seconds later, exactly as
a real USBL delivers it. Consumers must use the stamp, not arrival time.
"""

from collections import deque
import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation

from .models import UsblModel

UNKNOWN_ORIENTATION_VAR = 1e6


class UsblNode(Node):
    def __init__(self):
        super().__init__('usbl')
        dp = self.declare_parameter
        dp('transceiver_position', [0.0, 0.0, 9.5])
        dp('usbl_mount', [-0.35, 0.0, 0.25])
        dp('ping_interval', 1.0)
        dp('range_std', 0.03)
        dp('range_std_per_m', 0.002)
        dp('angle_std_deg', 0.5)
        dp('sound_speed', 1500.0)
        dp('processing_delay', 0.15)
        dp('max_range', 150.0)
        dp('dropout_probability', 0.05)
        dp('outlier_probability', 0.02)
        dp('outlier_std', 1.5)
        dp('seed', 0)
        dp('frame_id', 'world')
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.model = UsblModel(
            transceiver_position=g('transceiver_position'), range_std=g('range_std'),
            range_std_per_m=g('range_std_per_m'), angle_std=math.radians(g('angle_std_deg')),
            sound_speed=g('sound_speed'), processing_delay=g('processing_delay'),
            max_range=g('max_range'), dropout_probability=g('dropout_probability'),
            outlier_probability=g('outlier_probability'), outlier_std=g('outlier_std'),
            mount=g('usbl_mount'))
        self.rng = np.random.default_rng(g('seed') or None)
        self.interval = g('ping_interval')
        self.frame_id = g('frame_id')
        self.last_ping = None
        self.queue = deque()   # (release_time, msg)
        self.counts = {'pings': 0, 'dropouts': 0, 'outliers': 0}

        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'usbl/fix',
                                         qos_profile_sensor_data)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_truth, 10)

    def on_truth(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_ping is None or t - self.last_ping >= self.interval - 1e-6:
            self.last_ping = t
            self.ping(msg, t)
        while self.queue and self.queue[0][0] <= t + 1e-9:
            self.pub.publish(self.queue.popleft()[1])

    def ping(self, msg: Odometry, t: float) -> None:
        self.counts['pings'] += 1
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        fix = self.model.measure([p.x, p.y, p.z], rot, self.rng)
        if fix is None:
            self.counts['dropouts'] += 1
            return
        self.counts['outliers'] += int(fix.outlier)

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = \
            map(float, fix.position)
        out.pose.pose.orientation.w = 1.0
        cov = np.eye(6) * UNKNOWN_ORIENTATION_VAR
        cov[:3, :3] = fix.covariance
        out.pose.covariance = [float(v) for v in cov.ravel()]
        self.queue.append((t + fix.latency, out))


def main(args=None):
    rclpy.init(args=args)
    node = UsblNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
