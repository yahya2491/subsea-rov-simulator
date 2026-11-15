"""ROS 2 node: vehicle navigation from onboard sensors.

Subscribes
  imu/data     sensor_msgs/Imu                         gyro + accelerometer
  imu/mag      sensor_msgs/MagneticField               magnetometer
  dvl/twist    geometry_msgs/TwistWithCovarianceStamped DVL bottom-track velocity
  pressure     sensor_msgs/FluidPressure               pressure (depth) sensor
  usbl/fix     geometry_msgs/PoseWithCovarianceStamped acoustic position fix
Publishes
  nav/odom     nav_msgs/Odometry  pose in world (ENU), twist in base_link (FLU)
  TF           world -> base_link (optional)

The IMU drives the filter: every IMU sample advances the AHRS and the
Kalman prediction, and an estimate is published at ``publish_rate``. Aiding
sensors are applied as they arrive. Orientation is never taken from the
simulator; it comes from the AHRS on raw gyro/accel/mag data.
"""

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, \
    TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Imu, MagneticField
from tf2_ros import TransformBroadcaster

from .ahrs import MahonyAhrs, quat_to_matrix
from .kalman import PositionVelocityKF


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation')
        dp = self.declare_parameter
        # environment
        dp('surface_z', 10.0)
        dp('water_density', 997.0)
        dp('gravity', 9.81)
        dp('atmospheric_pressure', 101325.0)
        dp('magnetic_field', [0.41e-6, 15.6e-6, -48.1e-6])
        # vehicle (sensor mounts in base_link)
        dp('dvl_mount', [-0.12, 0.0, -0.171])
        dp('pressure_mount', [-0.28, 0.0, 0.08])
        dp('usbl_mount', [-0.35, 0.0, 0.25])
        # AHRS
        dp('ahrs.kp_acc', 0.5)
        dp('ahrs.kp_mag', 0.5)
        dp('ahrs.ki', 0.02)
        dp('ahrs.acc_gate', 0.3)
        dp('ahrs.mag_timeout', 0.2)
        dp('ahrs.dvl_accel_time_constant', 0.3)
        # filter
        dp('kf.accel_noise', 0.1)
        dp('kf.use_imu_acceleration', True)
        dp('kf.dvl_velocity_std_floor', 0.005)
        dp('kf.depth_std_floor', 0.005)
        dp('kf.initial_velocity_std', 0.3)
        dp('kf.max_consecutive_usbl_rejects', 5)
        # output
        dp('publish_rate', 50.0)
        dp('publish_tf', True)
        dp('orientation_std', 0.01)
        dp('world_frame', 'world')
        dp('base_frame', 'base_link')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.surface_z = g('surface_z')
        self.rho_g = g('water_density') * g('gravity')
        self.gravity = g('gravity')
        self.p_atm = g('atmospheric_pressure')
        self.r_dvl = np.array(g('dvl_mount'))
        self.r_pressure = np.array(g('pressure_mount'))
        self.r_usbl = np.array(g('usbl_mount'))
        self.mag_timeout = g('ahrs.mag_timeout')
        self.use_accel = g('kf.use_imu_acceleration')
        self.dvl_var_floor = g('kf.dvl_velocity_std_floor') ** 2
        self.depth_var_floor = g('kf.depth_std_floor') ** 2
        self.init_vel_std = g('kf.initial_velocity_std')
        self.publish_period = 1.0 / g('publish_rate')
        self.orientation_var = g('orientation_std') ** 2
        self.world_frame = g('world_frame')
        self.base_frame = g('base_frame')

        self.ahrs = MahonyAhrs(g('magnetic_field'), kp_acc=g('ahrs.kp_acc'),
                               kp_mag=g('ahrs.kp_mag'), ki=g('ahrs.ki'),
                               gravity=self.gravity, acc_gate=g('ahrs.acc_gate'))
        self.kf = PositionVelocityKF(accel_noise=g('kf.accel_noise'),
                                     max_consecutive_rejects=g('kf.max_consecutive_usbl_rejects'))

        self.mag = None
        self.mag_time = None
        self.imu_time = None
        self.rate_body = np.zeros(3)
        self.pending_depth = None      # (z, var) received before KF initialised
        self.last_publish = None
        self.dvl_prev = None
        self.dvl_acc = np.zeros(3)
        self.dvl_acc_time = None
        self.dvl_acc_tau = g('ahrs.dvl_accel_time_constant')
        self.dvl_acc_timeout = 0.5

        self.odom_pub = self.create_publisher(Odometry, 'nav/odom', 10)
        self.tf = TransformBroadcaster(self) if g('publish_tf') else None
        qos = qos_profile_sensor_data
        self.create_subscription(Imu, 'imu/data', self.on_imu, qos)
        self.create_subscription(MagneticField, 'imu/mag', self.on_mag, qos)
        self.create_subscription(TwistWithCovarianceStamped, 'dvl/twist', self.on_dvl, qos)
        self.create_subscription(FluidPressure, 'pressure', self.on_pressure, qos)
        self.create_subscription(PoseWithCovarianceStamped, 'usbl/fix', self.on_usbl, qos)
        self.create_timer(5.0, self.report)

    # -------------------------------------------------------------- callbacks
    def on_mag(self, msg: MagneticField) -> None:
        f = msg.magnetic_field
        self.mag = np.array([f.x, f.y, f.z])
        self.mag_time = stamp_to_sec(msg.header.stamp)

    def on_imu(self, msg: Imu) -> None:
        t = stamp_to_sec(msg.header.stamp)
        w, a = msg.angular_velocity, msg.linear_acceleration
        gyro = np.array([w.x, w.y, w.z])
        acc = np.array([a.x, a.y, a.z])

        if not self.ahrs.initialised:
            if self.mag is not None:
                self.ahrs.initialise(acc, self.mag)
                self.imu_time = t
                self.get_logger().info('AHRS aligned from accelerometer and magnetometer')
            return

        dt = t - self.imu_time
        self.imu_time = t
        if dt <= 0.0 or dt > 0.5:
            return
        mag_fresh = self.mag_time is not None and t - self.mag_time < self.mag_timeout
        lin_acc = None
        if self.dvl_acc_time is not None and t - self.dvl_acc_time < self.dvl_acc_timeout:
            lin_acc = self.dvl_acc
        self.rate_body = self.ahrs.update(gyro, acc, self.mag if mag_fresh else None, dt,
                                          linear_accel_body=lin_acc)

        if not self.kf.initialised:
            return
        accel_world = None
        if self.use_accel:
            accel_world = quat_to_matrix(self.ahrs.q) @ acc - np.array([0.0, 0.0, self.gravity])
        self.kf.predict(t, accel_world)

        if self.last_publish is None or t - self.last_publish >= self.publish_period - 1e-6:
            self.last_publish = t
            self.publish(msg.header.stamp)

    def on_dvl(self, msg: TwistWithCovarianceStamped) -> None:
        if not self.ahrs.initialised:
            return
        v = msg.twist.twist.linear
        v_sensor = np.array([v.x, v.y, v.z])
        # Remove the lever-arm term: the DVL sees the velocity of its own mount.
        v_body = v_sensor - np.cross(self.rate_body, self.r_dvl)
        self._update_dvl_acceleration(stamp_to_sec(msg.header.stamp), v_body)
        if not self.kf.initialised:
            return
        cov = np.array(msg.twist.covariance).reshape(6, 6)[:3, :3]
        cov = cov + np.eye(3) * self.dvl_var_floor
        self.kf.update_body_velocity(v_body, quat_to_matrix(self.ahrs.q), cov)

    def _update_dvl_acceleration(self, t: float, v_body: np.ndarray) -> None:
        """Low-passed inertial acceleration (body frame) from DVL velocity.

        a = dv/dt + w x v. It lets the AHRS separate real tilt from the
        vehicle being pushed around (see MahonyAhrs.update).
        """
        if self.dvl_prev is not None:
            t0, v0 = self.dvl_prev
            dt = t - t0
            if 0.0 < dt < 1.0:
                a = (v_body - v0) / dt + np.cross(self.rate_body, v_body)
                alpha = dt / (self.dvl_acc_tau + dt)
                self.dvl_acc = self.dvl_acc + alpha * (a - self.dvl_acc)
                self.dvl_acc_time = t
        self.dvl_prev = (t, v_body)

    def _depth_measurement(self, msg: FluidPressure):
        depth = (msg.fluid_pressure - self.p_atm) / self.rho_g
        var = max(msg.variance, 0.0) / self.rho_g ** 2 + self.depth_var_floor
        z_sensor = self.surface_z - depth
        return z_sensor, var

    def on_pressure(self, msg: FluidPressure) -> None:
        if not self.ahrs.initialised:
            return
        z_sensor, var = self._depth_measurement(msg)
        lever_z = (quat_to_matrix(self.ahrs.q) @ self.r_pressure)[2]
        z_base = z_sensor - lever_z
        if not self.kf.initialised:
            self.pending_depth = (z_base, var)
            return
        self.kf.update_depth(z_base, var)

    def on_usbl(self, msg: PoseWithCovarianceStamped) -> None:
        if not self.ahrs.initialised:
            return
        p = msg.pose.pose.position
        cov = np.array(msg.pose.covariance).reshape(6, 6)[:3, :3]
        base = np.array([p.x, p.y, p.z]) - quat_to_matrix(self.ahrs.q) @ self.r_usbl
        stamp = stamp_to_sec(msg.header.stamp)
        if not self.kf.initialised:
            if self.imu_time is None:
                return
            sigma = np.sqrt(np.diag(cov))
            if self.pending_depth is not None:
                base[2] = self.pending_depth[0]
                sigma[2] = np.sqrt(self.pending_depth[1])
            self.kf.initialise(base, self.imu_time, sigma_position=sigma,
                               sigma_velocity=self.init_vel_std)
            self.get_logger().info(
                f'navigation initialised at ({base[0]:.2f}, {base[1]:.2f}, {base[2]:.2f}) m')
            return
        res = self.kf.update_position(base, cov, stamp)
        if not res.accepted:
            self.get_logger().warn(f'USBL fix rejected (NIS {res.nis:.1f})',
                                   throttle_duration_sec=1.0)

    # ----------------------------------------------------------------- output
    def publish(self, stamp) -> None:
        x, P = self.kf.x, self.kf.P
        w, qx, qy, qz = self.ahrs.q
        rot = quat_to_matrix(self.ahrs.q)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        msg.child_frame_id = self.base_frame
        pose = msg.pose.pose
        pose.position.x, pose.position.y, pose.position.z = map(float, x[:3])
        pose.orientation.w, pose.orientation.x = float(w), float(qx)
        pose.orientation.y, pose.orientation.z = float(qy), float(qz)
        pc = np.zeros((6, 6))
        pc[:3, :3] = P[:3, :3]
        pc[3:, 3:] = np.eye(3) * self.orientation_var
        msg.pose.covariance = [float(v) for v in pc.ravel()]

        v_body = rot.T @ x[3:]
        tw = msg.twist.twist
        tw.linear.x, tw.linear.y, tw.linear.z = map(float, v_body)
        tw.angular.x, tw.angular.y, tw.angular.z = map(float, self.rate_body)
        tc = np.zeros((6, 6))
        tc[:3, :3] = rot.T @ P[3:, 3:] @ rot
        tc[3:, 3:] = np.eye(3) * 1e-5
        msg.twist.covariance = [float(v) for v in tc.ravel()]
        self.odom_pub.publish(msg)

        if self.tf is not None:
            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = pose.position.x
            tf.transform.translation.y = pose.position.y
            tf.transform.translation.z = pose.position.z
            tf.transform.rotation = pose.orientation
            self.tf.sendTransform(tf)

    def report(self) -> None:
        if not self.kf.initialised:
            self.get_logger().info('waiting for IMU, magnetometer and first USBL fix...',
                                   throttle_duration_sec=10.0)
            return
        s = self.kf.stats
        sig = np.sqrt(np.diag(self.kf.P))
        self.get_logger().info(
            f'updates accepted/rejected: dvl {s["dvl"]}, depth {s["depth"]}, usbl {s["usbl"]}; '
            f'1-sigma pos ({sig[0]:.3f}, {sig[1]:.3f}, {sig[2]:.3f}) m',
            throttle_duration_sec=30.0)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
