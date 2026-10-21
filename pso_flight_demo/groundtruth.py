"""Independent Gazebo transport subscriber. Never requests world control."""

import math
import threading
import time


def pose_sample(message, vehicle, expected_id=None):
    matches = [p for p in message.pose if p.name == vehicle]
    if len(matches) != 1:
        raise ValueError('Ground truth must contain exactly one named vehicle')
    p = matches[0]
    if expected_id is not None and p.id != expected_id:
        raise ValueError('Ground truth model entity ID changed')
    stamp = message.header.stamp.sec*1_000_000+message.header.stamp.nsec//1000
    q = [p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]
    xyz = [p.position.x, p.position.y, p.position.z]
    if stamp <= 0 or not all(math.isfinite(x) for x in xyz+q) or abs(sum(x*x for x in q)-1) > .04:
        raise ValueError('Invalid ground truth timestamp/pose')
    return dict(native_timestamp_us=stamp, position_enu=xyz, quaternion_enu=q, entity_id=int(p.id))


class GazeboPoseObserver:
    def __init__(self, world, vehicle, expected_id=None):
        self.world, self.vehicle, self.expected_id = world, vehicle, expected_id
        self.samples, self.errors = [], []
        self.lock = threading.Lock()
        self.callbacks_idle = threading.Condition(self.lock)
        self.active_callbacks = 0
        self.closing = False
        self.node = None
        self.subscribed = False
        self.topic = f'/world/{world}/pose/info'

    def start(self):
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.node = Node()
        if self.node.subscribe(Pose_V, self.topic, self.receive) is False:
            raise RuntimeError('Gazebo subscription creation failed: '+self.topic)
        self.subscribed = True

    def receive(self, message):
        with self.callbacks_idle:
            # Gazebo Transport invokes this callback from a native worker
            # thread. Do not let an already queued callback touch Python state
            # while finish() is tearing down the transport node.
            if self.closing:
                return
            self.active_callbacks += 1
        try:
            try:
                sample = pose_sample(message, self.vehicle, self.expected_id)
                sample['received_monotonic'] = time.monotonic()
                with self.lock:
                    if len(self.samples) >= 10000:
                        raise ValueError('Ground truth evidence capacity exceeded')
                    self.samples.append(sample)
            except ValueError as exc:
                with self.lock:
                    self.errors.append(str(exc))
        finally:
            with self.callbacks_idle:
                self.active_callbacks -= 1
                self.callbacks_idle.notify_all()

    def finish(self):
        # Unsubscribe and release the Gazebo node before rclpy/Python shutdown.
        # Merely unsubscribing leaves the native Node alive; a queued callback
        # can then enter the Python binding during interpreter finalization.
        with self.callbacks_idle:
            self.closing = True
            node, self.node = self.node, None
        unsubscribe_result = None
        if node is not None:
            unsubscribe_result = node.unsubscribe(self.topic)
            with self.callbacks_idle:
                # receive() is deliberately short and never waits on external
                # I/O. Drain it fully before destroying the native transport
                # node so an in-flight callback cannot outlive Python state.
                self.callbacks_idle.wait_for(lambda: self.active_callbacks == 0)
            # Drop the last observer-owned reference now, while Python is live.
            del node
        with self.lock:
            return dict(topic=self.topic, samples=list(self.samples), errors=sorted(set(self.errors)),
                        subscriber_created=self.subscribed, unsubscribe_result=unsubscribe_result,
                        callbacks_drained=self.active_callbacks == 0,
                        publishes_commands=False, controls_physics=False)
