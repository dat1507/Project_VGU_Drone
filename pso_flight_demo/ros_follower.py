"""Opt-in live ROS 2 follower. Importing this module creates no ROS entities.

    Deliberately contains NO Arm or Takeoff command. A future approved native
    takeoff must reach hover before the explicitly enabled live follower starts.
"""

import math
import subprocess
import time

import numpy as np

from .frames import RESET_FIELDS
from .safety import (Freshness, Observation, RouteTracker, check_observation,
                     matching_ack, mode_confirmed, quaternion_tilt, topic_name)


def run_live(path, metadata, anchor, environment, confirmation, namespace=""):
    if confirmation != "CONTROL_APPROVED":
        raise ValueError("Live control requires explicit --confirm-live CONTROL_APPROVED")
    # Imports are delayed so offline dry-run works without ROS installed/sourced.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from px4_msgs.msg import (EstimatorStatusFlags, OffboardControlMode, TrajectorySetpoint,
                             VehicleAttitude, VehicleCommand, VehicleCommandAck,
                             VehicleLandDetected, VehicleLocalPosition, VehicleStatus)

    prefix = "/" + namespace.strip("/") if namespace.strip("/") else ""

    class Follower(Node):
        def __init__(self):
            super().__init__("pso_flight_demo")
            if self.get_parameter("use_sim_time").value:
                raise ValueError("This version requires OS clock + PX4 DDS timesync, RTF near 1")
            self.samples, self.fresh = {}, Freshness()
            self.input_error = None
            self.acks = []
            self.stage = "waiting"
            self.started = time.monotonic()
            self.phase_started = self.started
            self.sim_started = None
            self.finished = False
            self.paused = False
            self.success = False
            self.route_completed = False
            self.aborted = False
            self.land_sent = False
            self.mode_sent = False
            self.tracker = RouteTracker(path)
            self.target = path[0].copy()
            self.yaw = None
            sub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
            pub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
            types = dict(position=VehicleLocalPosition, status=VehicleStatus,
                         attitude=VehicleAttitude, land=VehicleLandDetected,
                         estimator=EstimatorStatusFlags)
            names = dict(position="vehicle_local_position", status="vehicle_status",
                         attitude="vehicle_attitude", land="vehicle_land_detected",
                         estimator="estimator_status_flags")
            self.subscriptions_keepalive = []
            for key, cls in types.items():
                topic = prefix + topic_name("/fmu/out/" + names[key], cls)
                self.subscriptions_keepalive.append(self.create_subscription(
                    cls, topic, lambda msg, k=key: self.receive(k, msg), sub_qos))
            self.ack_sub = self.create_subscription(
                VehicleCommandAck, prefix + topic_name("/fmu/out/vehicle_command_ack", VehicleCommandAck),
                lambda msg: self.acks.append((time.monotonic(), msg)), sub_qos)
            self.mode_pub = self.create_publisher(OffboardControlMode, prefix + "/fmu/in/offboard_control_mode", pub_qos)
            self.setpoint_pub = self.create_publisher(TrajectorySetpoint, prefix + topic_name("/fmu/in/trajectory_setpoint", TrajectorySetpoint), pub_qos)
            self.command_pub = self.create_publisher(VehicleCommand, prefix + topic_name("/fmu/in/vehicle_command", VehicleCommand), pub_qos)
            self.timer = self.create_timer(0.05, self.tick)

        def receive(self, key, msg):
            try:
                self.fresh.record(key, int(msg.timestamp), time.monotonic())
                self.samples[key] = msg
            except ValueError as exc:
                self.input_error = str(exc)

        def observation(self):
            self.fresh.check(("position", "status", "attitude", "land", "estimator"), time.monotonic())
            p, s, a, l, e = [self.samples[k] for k in ("position", "status", "attitude", "land", "estimator")]
            fault = any(getattr(e, k) for k in e.get_fields_and_field_types() if k.startswith("fs_"))
            fault = fault or e.cs_mag_fault or e.cs_gnss_fault
            valid = all((p.xy_valid, p.z_valid, p.v_xy_valid, p.v_z_valid,
                         e.cs_tilt_align, e.cs_yaw_align))
            if s.system_id != 1 or s.component_id != 1:
                raise ValueError("This first-demo controller is restricted to PX4 sysid/component 1/1")
            return Observation(tuple([p.x, p.y, p.z]), tuple(a.q), tuple([p.vx, p.vy, p.vz]),
                               tuple(getattr(p, k) for k in RESET_FIELDS), int(p.timestamp),
                               s.arming_state == s.ARMING_STATE_ARMED, l.landed, s.failsafe,
                               s.gcs_connection_lost, valid, bool(fault), s.pre_flight_checks_pass)

        def stamp(self):
            return self.get_clock().now().nanoseconds // 1000

        def publish_target(self):
            mode = OffboardControlMode()
            mode.timestamp, mode.position = self.stamp(), True
            self.mode_pub.publish(mode)
            sp = TrajectorySetpoint()
            sp.timestamp = self.stamp()
            sp.position = anchor.to_ned(self.target).tolist()
            sp.velocity = [float("nan")] * 3
            sp.acceleration = [float("nan")] * 3
            sp.jerk = [float("nan")] * 3
            sp.yaw, sp.yawspeed = self.yaw, float("nan")
            self.setpoint_pub.publish(sp)

        def command(self, number, p1=float("nan"), p2=float("nan")):
            msg = VehicleCommand()
            msg.timestamp, msg.command = self.stamp(), number
            msg.param1, msg.param2 = float(p1), float(p2)
            msg.target_system, msg.target_component = 1, 1
            msg.source_system, msg.source_component = 245, 191
            msg.from_external = True
            self.phase_started = time.monotonic()
            self.command_pub.publish(msg)  # exactly one send; no automatic retry

        def ack_result(self, command):
            return matching_ack(self.acks, command, self.phase_started)

        def pause(self, reason):
            self.get_logger().error("PAUSE requested: " + reason)
            try:
                result = subprocess.run(["gz", "service", "-s", "/world/warehouse_world/control",
                                         "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                                         "--timeout", "2000", "--req", "pause: true"],
                                        capture_output=True, text=True, timeout=3)
                self.paused = result.returncode == 0 and "data: true" in result.stdout
            except (OSError, subprocess.TimeoutExpired):
                self.paused = False
            if not self.paused:
                self.get_logger().error("PAUSE NOT CONFIRMED: use Gazebo GUI Pause immediately")
            self.finished = True

        def land(self, reason):
            if self.land_sent:
                return
            self.get_logger().warning("Native LAND request: " + reason)
            self.land_sent = True
            self.command(21)
            self.stage = "landing"

        def abort(self, reason):
            self.aborted = True
            if self.stage == "waiting":
                self.pause(reason)
                return
            try:
                obs = self.observation()
                check_observation(obs, anchor, environment)
                if not obs.armed or obs.landed:
                    self.pause(reason)
                    return
                self.land(reason)
            except (ValueError, KeyError):
                self.pause(reason)

        def tick(self):
            if self.finished:
                return
            now = time.monotonic()
            try:
                if self.input_error:
                    raise ValueError(self.input_error)
                if self.stage == "landing":
                    # Descent intentionally leaves cruise bounds. Never apply
                    # cruise floor clearance to a legitimate native landing.
                    obs = self.observation()
                    anchor.check_resets(obs.resets)
                    roll, pitch = quaternion_tilt(obs.quaternion)
                    if not obs.valid or obs.estimator_fault or obs.failsafe or max(abs(roll), abs(pitch)) > math.radians(15):
                        raise ValueError("Unsafe state during native landing")
                    ack = self.ack_result(21)
                    if ack is not None and ack != 0:
                        raise ValueError("LAND rejected")
                    if obs.landed and not obs.armed:
                        self.success = self.route_completed and not self.aborted
                        self.pause("Landed=True and Armed=False")
                    else:
                        mode_confirmed(ack, self.samples["status"].nav_state, 18, now-self.phase_started)
                        if now - self.phase_started > 30:
                            raise ValueError("Landing/disarm timeout")
                    return
                try:
                    obs = self.observation()
                except ValueError:
                    if self.stage == "waiting" and now - self.started < 3:
                        return
                    raise
                actual = check_observation(obs, anchor, environment)
                if not obs.armed or obs.landed:
                    raise ValueError("Live follower requires an already-armed, airborne native hover")
                if np.linalg.norm(obs.velocity_ned) > 0.7:
                    raise ValueError("Actual speed exceeds 0.7 m/s")
                if actual[2] - anchor.gazebo_enu[2] > 3.2:
                    raise ValueError("Relative height exceeds 3.2 m")
                if np.linalg.norm(actual - self.target) > 0.25:
                    raise ValueError("Tracking error exceeds 0.25 m before mode/target update")
                sim_time = obs.timestamp_us / 1e6
                if self.stage == "waiting":
                    if not obs.preflight or np.linalg.norm(actual - path[0]) > 0.15:
                        raise ValueError("Preflight/start-position gate failed")
                    self.sim_started = sim_time
                    self.yaw = float(self.samples["position"].heading)
                    if not math.isfinite(self.yaw):
                        raise ValueError("Missing finite heading for yaw hold")
                    self.stage = "prestream"
                if sim_time - self.sim_started > 120:
                    raise ValueError("120 simulated-second route limit reached")
                if self.stage == "flying":
                    if self.samples["status"].nav_state != 14:
                        raise ValueError("Unexpected exit from Offboard")
                    self.target, done = self.tracker.update(actual, sim_time)
                    if done:
                        self.route_completed = True
                        self.land("Route complete, including 5 simulated seconds hold at each waypoint")
                        return
                self.publish_target()
                if self.stage == "prestream" and sim_time - self.sim_started >= 1.2:
                    self.mode_sent = True
                    self.command(176, 1, 6)
                    self.stage = "mode_pending"
                elif self.stage == "mode_pending":
                    ack = self.ack_result(176)
                    if mode_confirmed(ack, self.samples["status"].nav_state, 14, now-self.phase_started):
                        self.stage = "flying"
            except (ValueError, KeyError) as exc:
                if self.stage == "landing":
                    self.pause(str(exc))
                else:
                    self.abort(str(exc))

    rclpy.init()
    node = None
    try:
        node = Follower()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.success or not node.paused:
            raise RuntimeError("Flight aborted or final Pause unconfirmed; do not retry automatically")
    finally:
        if node is not None:
            if not node.finished:
                node.pause("Controller interrupted/exception; preserving simulation state")
            node.destroy_node()
        rclpy.shutdown()
