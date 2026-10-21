"""Pure decisions for an independent observer. No command/physics execution.

Land requests require usable state/link/ground truth and a clear descent column.
All other faults request operator emergency Pause; neither request is an ACK.
"""

from dataclasses import dataclass
import numpy as np

from .geometry import vector


@dataclass(frozen=True)
class WatchSample:
    groundtruth_enu: tuple
    ekf_ned: tuple
    target_enu: tuple
    native_gt_us: int
    native_ekf_us: int
    received_gt_wall: float
    received_dds_wall: float
    setpoint_wall: float
    resets: tuple
    estimator_valid: bool = True
    dds_connected: bool = True
    armed: bool = True
    landed: bool = False
    gcs_lost: bool = False


class GroundTruthWatchdog:
    def __init__(self, anchor, environment, *, project_ceiling_required=True):
        self.anchor, self.environment = anchor, environment
        self.project_ceiling_required = project_ceiling_required
        self.previous = None

    def evaluate(self, s, now, landing_column_clear=False, offboard_stream_required=True,
                 tracking_required=True, landing_mode=False,
                 landing_vertical_speed_limit=.9):
        reasons = []
        reliable = True
        try:
            self.anchor.check_resets(s.resets)
        except ValueError:
            reasons.append('EKF_RESET')
            reliable = False
        ages = [(now-s.received_gt_wall, .25, 'GROUNDTRUTH_STALE'),
                (now-s.received_dds_wall, .5, 'DDS_STALE')]
        if offboard_stream_required:
            ages.append((now-s.setpoint_wall, .15, 'SETPOINT_STALE'))
        for age, limit, label in ages:
            if not np.isfinite(age) or age < 0 or age > limit:
                reasons.append(label)
                if label != 'SETPOINT_STALE':
                    reliable = False
        if not s.dds_connected or not s.estimator_valid or s.gcs_lost:
            reasons.append('DDS_OR_ESTIMATOR_INVALID')
            reliable = False
        if (min(s.native_gt_us, s.native_ekf_us) <= 0 or
                abs(s.native_gt_us-s.native_ekf_us) > 20_000):
            reasons.append('NATIVE_TIME_PAIR_INVALID')
            reliable = False
        try:
            gt, target = vector(s.groundtruth_enu), vector(s.target_enu)
            ekf = self.anchor.to_enu(vector(s.ekf_ned))
            if np.linalg.norm(gt-ekf) > .15:
                reasons.append('EKF_GROUNDTRUTH_DIVERGENCE')
                reliable = False
            if tracking_required and np.linalg.norm(gt-target) > .25:
                reasons.append('GROUNDTRUTH_TRACKING')
            self.environment.validate([gt, gt] if self.previous is None else [self.previous[0], gt])
            if self.previous is not None:
                dt = (s.native_gt_us-self.previous[1])/1e6
                if dt <= 0:
                    reasons.append('GROUNDTRUTH_TIME_FROZEN_OR_BACKWARDS')
                    reliable = False
                else:
                    velocity=(gt-self.previous[0])/dt
                    speed_exceeded = (np.linalg.norm(velocity) > .7)
                    if landing_mode:
                        # Once PX4 has accepted Land mode, its controlled
                        # vertical descent is allowed a bounded envelope; the
                        # ordinary 0.7 m/s full-vector gate remains in force
                        # for cruise and for the pre-ACK landing transition.
                        speed_exceeded = (np.linalg.norm(velocity[:2]) > .7 or
                                          abs(float(velocity[2])) > landing_vertical_speed_limit)
                    if speed_exceeded:
                        reasons.append('GROUNDTRUTH_SPEED')
            if self.project_ceiling_required and gt[2]-self.anchor.gazebo_enu[2] > 3.2:
                reasons.append('RELATIVE_HEIGHT')
            self.previous = (gt.copy(), s.native_gt_us)
        except (ValueError, TypeError):
            reasons.append('GROUNDTRUTH_GEOMETRY_OR_NUMERIC')
            reliable = False
        action = 'CONTINUE_OBSERVING'
        if reasons:
            action = ('REQUEST_LAND' if reliable and s.armed and not s.landed and landing_column_clear
                      else 'REQUEST_EMERGENCY_PAUSE')
        return dict(action=action, reasons=reasons, executed=False, land_confirmed=False,
                    live_control_go=False)


class LandConfirmation:
    """One request, ACK+mode, then landed AND disarmed; never retry implicitly."""
    def __init__(self):
        self.sent = None
        self.mode_accepted = False

    def request(self, wall):
        if self.sent is not None:
            raise ValueError('LAND already requested; no automatic retry')
        self.sent = wall

    def evaluate(self, now, ack, mode, landed, armed):
        if self.sent is None:
            return 'NOT_REQUESTED'
        if now < self.sent:
            return 'REQUEST_EMERGENCY_PAUSE'
        if ack is not None and ack != 0:
            return 'REQUEST_EMERGENCY_PAUSE'
        self.mode_accepted |= ack == 0 and mode == 18
        confirmed = self.mode_accepted
        if confirmed and landed and not armed:
            return 'LAND_CONFIRMED'
        if (not confirmed and now-self.sent > 3) or now-self.sent > 30:
            return 'REQUEST_EMERGENCY_PAUSE'
        return 'WAIT_LAND_CONFIRMATION'
