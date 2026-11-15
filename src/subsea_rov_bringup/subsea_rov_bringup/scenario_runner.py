"""ROS 2 node: play one disturbance scenario, record it, score it, exit.

Phases
  wait    until the navigation estimate and the hover controller are up
  settle  command the scenario setpoint and let the vehicle settle
  run     t = 0: play the scenario's events (currents, waves, pushes,
          setpoint changes) and record at ``record_rate``
  done    write <scenario>.csv / .json / .png into ``output_dir`` and exit

Disturbances go into Gazebo through ros_gz_bridge:
  /ocean_current                     drag-based current (Hydrodynamics system)
  /world/default/wrench/persistent   external force on base_link
  /world/default/wrench/clear
"""

import copy
import json
import math
import os
import time

from geometry_msgs.msg import PoseStamped, Vector3, WrenchStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity, EntityWrench
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64MultiArray
import yaml

from . import analysis


def load_scenario(path: str, name: str) -> dict:
    with open(path) as f:
        doc = yaml.safe_load(f)
    if name not in doc['scenarios']:
        raise KeyError(f'unknown scenario "{name}"; have {sorted(doc["scenarios"])}')
    spec = copy.deepcopy(doc.get('defaults', {}))
    spec.update(copy.deepcopy(doc['scenarios'][name]))
    spec.setdefault('events', [])
    spec['events'] = sorted(spec['events'], key=lambda e: e['t'])
    return spec


def euler(q):
    yaw, pitch, roll = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('ZYX')
    return roll, pitch, yaw


class ScenarioRunner(Node):
    def __init__(self):
        super().__init__('scenario_runner')
        self.declare_parameter('scenario', 'calm_hold')
        self.declare_parameter('scenarios_file', '')
        self.declare_parameter('output_dir', 'validation_results')
        self.declare_parameter('record_rate', 20.0)
        self.declare_parameter('startup_timeout', 90.0)
        self.declare_parameter('link_name', 'dummy_rov::base_link')
        self.declare_parameter('world_name', 'default')
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.name = g('scenario')
        self.spec = load_scenario(g('scenarios_file'), self.name)
        self.out_dir = os.path.abspath(os.path.expanduser(g('output_dir')))
        os.makedirs(self.out_dir, exist_ok=True)
        self.link = g('link_name')
        self.startup_timeout = g('startup_timeout')
        world = g('world_name')

        self.gt = None
        self.nav = None
        self.thrust = [0.0] * 8
        self.wrench = [0.0] * 6
        self.controller_up = False
        self.subs = [
            self.create_subscription(Odometry, '/rov/ground_truth/odom', self._set('gt'), 10),
            self.create_subscription(Odometry, '/rov/nav/odom', self._set('nav'), 10),
            self.create_subscription(Float64MultiArray, '/rov/thrusters/command',
                                     self.on_thrust, 10),
            self.create_subscription(WrenchStamped, '/rov/cmd_wrench', self.on_wrench, 10),
            self.create_subscription(PoseStamped, '/rov/hover/active_setpoint',
                                     self.on_active_sp, 10),
        ]
        self.sp_pub = self.create_publisher(PoseStamped, '/rov/hover/setpoint', 10)
        self.cur_pub = self.create_publisher(Vector3, '/ocean_current', 10)
        self.push_pub = self.create_publisher(EntityWrench,
                                              f'/world/{world}/wrench/persistent', 10)
        self.clear_pub = self.create_publisher(Entity, f'/world/{world}/wrench/clear', 10)

        self.phase = 'wait'
        self.phase_start = None
        self.t0 = None
        self.setpoint = self.spec['setpoint']
        self.steady_current = np.zeros(3)
        self.push = np.zeros(3)
        self.push_end = None
        self.waves = []                 # (start, end, amplitude, period)
        self.fired = set()
        self.rows = []
        self.created_wall = time.monotonic()   # sim clock may not be running yet
        self.create_timer(1.0 / g('record_rate'), self.tick)
        self.get_logger().info(f'scenario "{self.name}": {self.spec.get("description", "")}')

    # --------------------------------------------------------------- inputs
    def _set(self, attr):
        def cb(msg):
            setattr(self, attr, msg)
        return cb

    def on_thrust(self, msg):
        if len(msg.data) == 8:
            self.thrust = list(msg.data)

    def on_wrench(self, msg):
        f, t = msg.wrench.force, msg.wrench.torque
        self.wrench = [f.x, f.y, f.z, t.x, t.y, t.z]

    def on_active_sp(self, msg):
        self.controller_up = True

    # -------------------------------------------------------------- outputs
    def publish_setpoint(self):
        sp = self.setpoint
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, sp['position'])
        qx, qy, qz, qw = Rotation.from_euler('z', math.radians(sp['yaw_deg'])).as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y = float(qx), float(qy)
        msg.pose.orientation.z, msg.pose.orientation.w = float(qz), float(qw)
        self.sp_pub.publish(msg)

    def publish_current(self, v):
        self.cur_pub.publish(Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2])))

    def apply_push(self, force):
        msg = EntityWrench()
        msg.entity.name = self.link
        msg.entity.type = Entity.LINK
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = map(float, force)
        self.push_pub.publish(msg)
        self.push = np.asarray(force, dtype=float)

    def clear_push(self):
        self.clear_pub.publish(Entity(name=self.link, type=Entity.LINK))
        self.push = np.zeros(3)

    # ----------------------------------------------------------------- loop
    def elapsed(self, since):
        return (self.get_clock().now() - since).nanoseconds * 1e-9

    def tick(self):
        if self.phase == 'wait':
            if self.gt is not None and self.nav is not None and self.controller_up:
                self.phase, self.phase_start = 'settle', self.get_clock().now()
                self.get_logger().info(f'stack is up; settling for {self.spec["settle"]:.0f} s')
                self.publish_current([0.0, 0.0, 0.0])
                self.clear_push()
            elif time.monotonic() - self.created_wall > self.startup_timeout:
                self.get_logger().error('stack did not come up in time '
                                        f'(gt={self.gt is not None}, nav={self.nav is not None}, '
                                        f'controller={self.controller_up})')
                self.finish()
            return

        if self.phase == 'settle':
            self.publish_setpoint()          # repeated: robust to late subscribers
            t = self.elapsed(self.phase_start) - self.spec['settle']
            self.record(t)
            if t >= 0.0:
                self.phase, self.t0 = 'run', self.get_clock().now()
                self.get_logger().info('disturbance phase started')
            return

        if self.phase == 'run':
            t = self.elapsed(self.t0)
            self.play_events(t)
            self.record(t)
            if t >= self.spec['duration']:
                self.finish()

    def play_events(self, t):
        for i, ev in enumerate(self.spec['events']):
            if i in self.fired or t < ev['t']:
                continue
            self.fired.add(i)
            if 'current' in ev:
                self.steady_current = np.asarray(ev['current'], dtype=float)
                self.get_logger().info(f't={t:.1f}s current {self.steady_current.tolist()} m/s')
            if 'wave' in ev:
                w = ev['wave']
                self.waves.append((ev['t'], ev['t'] + w['duration'],
                                   np.asarray(w['amplitude'], dtype=float), w['period']))
                self.get_logger().info(f't={t:.1f}s wave {w}')
            if 'push' in ev:
                self.apply_push(ev['push']['force'])
                self.push_end = ev['t'] + ev['push']['duration']
                self.get_logger().info(f't={t:.1f}s push {ev["push"]}')
            if 'setpoint' in ev:
                self.setpoint = ev['setpoint']
                self.get_logger().info(f't={t:.1f}s setpoint {ev["setpoint"]}')
            self.publish_setpoint()

        if self.push_end is not None and t >= self.push_end:
            self.clear_push()
            self.push_end = None

        current = self.steady_current.copy()
        for start, end, amp, period in self.waves:
            if start <= t < end:
                # Orbital motion, ramped in over the first period so it
                # starts without a step.
                ph = 2.0 * math.pi * (t - start) / period
                envelope = min(1.0, (t - start) / period)
                current += envelope * amp * np.array([math.sin(ph), math.sin(ph), math.cos(ph)])
        self.current = current
        self.publish_current(current)

    def record(self, t):
        if self.gt is None or self.nav is None:
            return
        gp, np_ = self.gt.pose.pose.position, self.nav.pose.pose.position
        sp = self.setpoint
        cur = getattr(self, 'current', np.zeros(3))
        self.rows.append(
            [t, gp.x, gp.y, gp.z, *euler(self.gt.pose.pose.orientation),
             np_.x, np_.y, np_.z, *euler(self.nav.pose.pose.orientation),
             *sp['position'], math.radians(sp['yaw_deg']),
             *cur, *self.push, *self.thrust, *self.wrench])

    def finish(self):
        self.publish_current([0.0, 0.0, 0.0])
        self.clear_push()
        base = os.path.join(self.out_dir, self.name)
        analysis.write_csv(base + '.csv', self.rows)
        data = analysis.load_csv(base + '.csv')
        result = analysis.evaluate(data, self.spec)
        result['description'] = self.spec.get('description', '')
        with open(base + '.json', 'w') as f:
            json.dump(result, f, indent=2)
        if len(self.rows) > 2:
            analysis.plot(data, self.spec, self.name, base + '.png')
        m = result['metrics']
        verdict = 'PASS' if result['passed'] else 'FAIL'
        self.get_logger().info(
            f'{verdict} {self.name}: horizontal rms {m["horizontal_rms"]:.3f} m, '
            f'max {m["horizontal_max"]:.3f} m, depth max {m["depth_max"]:.3f} m, '
            f'yaw max {m["yaw_max_deg"]:.2f} deg, nav rms {m["nav_position_rms"]:.3f} m'
            + ('' if result['passed'] else f' | {"; ".join(result["failures"])}'))
        self.phase = 'done'
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioRunner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
