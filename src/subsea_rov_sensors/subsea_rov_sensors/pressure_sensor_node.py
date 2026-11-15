"""ROS 2 node: simulated pressure (depth) sensor.

Subscribes  ground_truth/odom  nav_msgs/Odometry (from Gazebo)
Publishes   pressure           sensor_msgs/FluidPressure, Pa, at ``rate`` Hz

Gazebo has no water-pressure sensor, so this samples the true sensor-port
depth and applies the hydrostatic model in models.PressureSensorModel.
"""

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import FluidPressure

from .models import PressureSensorModel


class PressureSensorNode(Node):
    def __init__(self):
        super().__init__('pressure_sensor')
        dp = self.declare_parameter
        dp('surface_z', 10.0)
        dp('water_density', 997.0)
        dp('gravity', 9.81)
        dp('atmospheric_pressure', 101325.0)
        dp('pressure_mount', [-0.28, 0.0, 0.08])
        dp('rate', 20.0)
        dp('noise_std', 50.0)
        dp('offset', 0.0)
        dp('seed', 0)
        dp('frame_id', 'pressure_sensor')
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.model = PressureSensorModel(
            surface_z=g('surface_z'), water_density=g('water_density'), gravity=g('gravity'),
            atmospheric_pressure=g('atmospheric_pressure'), noise_std=g('noise_std'),
            offset=g('offset'), mount=g('pressure_mount'))
        self.rng = np.random.default_rng(g('seed') or None)
        self.period = 1.0 / g('rate')
        self.frame_id = g('frame_id')
        self.last = None
        self.pub = self.create_publisher(FluidPressure, 'pressure', qos_profile_sensor_data)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_truth, 10)

    def on_truth(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last is not None and t - self.last < self.period - 1e-6:
            return
        self.last = t
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        out = FluidPressure()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.fluid_pressure = float(self.model.measure([p.x, p.y, p.z], rot, self.rng))
        out.variance = float(self.model.noise_std ** 2)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PressureSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
