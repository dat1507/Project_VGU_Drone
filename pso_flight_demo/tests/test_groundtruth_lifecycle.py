import threading
import time
import unittest
from unittest.mock import patch

from pso_flight_demo.groundtruth import GazeboPoseObserver


class FakeGazeboNode:
    def __init__(self):
        self.unsubscribed = []

    def unsubscribe(self, topic):
        self.unsubscribed.append(topic)
        return True


class GazeboPoseObserverLifecycleTests(unittest.TestCase):
    def test_finish_unsubscribes_and_releases_native_node(self):
        observer = GazeboPoseObserver('warehouse_world', 'x500', 64)
        node = FakeGazeboNode()
        observer.node = node
        observer.subscribed = True

        result = observer.finish()

        self.assertEqual(node.unsubscribed, ['/world/warehouse_world/pose/info'])
        self.assertIsNone(observer.node)
        self.assertTrue(result['callbacks_drained'])
        self.assertTrue(result['unsubscribe_result'])

    def test_finish_waits_for_inflight_callback_before_returning(self):
        observer = GazeboPoseObserver('warehouse_world', 'x500', 64)
        node = FakeGazeboNode()
        observer.node = node
        observer.subscribed = True
        callback_entered = threading.Event()
        release_callback = threading.Event()

        def slow_pose_sample(*_args):
            callback_entered.set()
            self.assertTrue(release_callback.wait(2.0))
            return dict(position_enu=[0., 0., 1.], received_monotonic=time.monotonic())

        with patch('pso_flight_demo.groundtruth.pose_sample', side_effect=slow_pose_sample):
            callback = threading.Thread(target=observer.receive, args=(object(),))
            callback.start()
            self.assertTrue(callback_entered.wait(1.0))

            result_box = []
            finished = threading.Event()

            def finish():
                result_box.append(observer.finish())
                finished.set()

            shutdown = threading.Thread(target=finish)
            shutdown.start()
            self.assertFalse(finished.wait(.05))
            release_callback.set()
            callback.join(1.0)
            shutdown.join(1.0)

        self.assertTrue(finished.is_set())
        self.assertEqual(observer.active_callbacks, 0)
        self.assertTrue(result_box[0]['callbacks_drained'])
        self.assertIsNone(observer.node)

    def test_late_callback_after_finish_is_ignored(self):
        observer = GazeboPoseObserver('warehouse_world', 'x500', 64)
        observer.node = FakeGazeboNode()
        observer.finish()

        with patch('pso_flight_demo.groundtruth.pose_sample', side_effect=AssertionError('late callback parsed')):
            observer.receive(object())

        self.assertEqual(observer.samples, [])
        self.assertEqual(observer.errors, [])


if __name__ == '__main__':
    unittest.main()
