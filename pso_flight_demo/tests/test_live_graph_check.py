"""Exercise the exact DEMO-ONLY live executor graph_check method without ROS."""
from dataclasses import dataclass
import unittest

from pso_flight_demo.dds_evidence import REQUIRED
from pso_flight_demo.integrated_ros import GraphCheckMixin


def message_type(name, version=0):
    return type(name, (), {'MESSAGE_VERSION': version})


@dataclass
class Endpoint:
    endpoint_gid: list
    topic_type: str


class FakeExecutor(GraphCheckMixin):
    def __init__(self, demo_only=True):
        output_names = (*REQUIRED, 'vehicle_command_ack')
        self.types = {name: message_type(name.title().replace('_', ''), i+1)
                      for i, name in enumerate(output_names)}
        self.inputs = {name: message_type(name.title().replace('_', ''), i+1)
                       for i, name in enumerate(('offboard_control_mode', 'trajectory_setpoint', 'vehicle_command'))}
        self.demo_only = demo_only
        self.publishers_control = {}
        self.expected_participant = None
        self.subscriptions_api_calls = []
        self.logs = []
        self.prefix = [1, 15, 86, 115, 115, 164, 50, 30, 0, 0, 0, 0]

    def get_publishers_info_by_topic(self, name):
        # Graph endpoint schema is inferred from the fully qualified input/output key.
        key = name.rsplit('/', 1)[-1].rsplit('_v', 1)[0]
        cls = next((t for t in (*self.types.values(), *self.inputs.values()) if
                    t.__name__.lower() == ''.join(part.title() for part in key.split('_')).lower()), None)
        if name.startswith('/fmu/in/'):
            return []
        assert cls is not None, name
        return [Endpoint(self.prefix+[len(name) % 200, 3], 'px4_msgs/msg/'+cls.__name__)]

    def get_subscriptions_info_by_topic(self, name):
        self.subscriptions_api_calls.append(name)
        key = name.rsplit('/', 1)[-1].rsplit('_v', 1)[0]
        cls = next(t for t in self.inputs.values()
                   if t.__name__.lower() == ''.join(part.title() for part in key.split('_')).lower())
        return [Endpoint(self.prefix+[91, 4], 'px4_msgs/msg/'+cls.__name__)]

    def count_publishers(self, name):
        return 0 if name.startswith('/fmu/in/') else 1

    def count_subscribers(self, name):
        return 1

    def log(self, event, **fields):
        self.logs.append((event, fields))


class LiveGraphCheckTests(unittest.TestCase):
    def test_demo_only_graph_check_uses_jazzy_subscriptions_api(self):
        executor = FakeExecutor()
        executor.graph_check()
        self.assertEqual(len(executor.subscriptions_api_calls), 3)
        self.assertEqual(executor.expected_participant, tuple(executor.prefix))
        self.assertEqual(len(executor.logs), 1)
        event, fields = executor.logs[0]
        self.assertEqual(event, 'GRAPH')
        self.assertEqual(len(fields['graph']), 10)

    def test_default_graph_check_does_not_query_px4_readers(self):
        executor = FakeExecutor(demo_only=False)
        executor.graph_check()
        self.assertEqual(executor.subscriptions_api_calls, [])

    def test_demo_graph_rejects_reader_from_different_participant(self):
        executor = FakeExecutor()
        original = executor.get_subscriptions_info_by_topic

        def other_participant(name):
            endpoints = original(name)
            endpoints[0].endpoint_gid[0] = 9
            return endpoints

        executor.get_subscriptions_info_by_topic = other_participant
        with self.assertRaisesRegex(ValueError, 'CONTROL_READER_NOT_THE_VERIFIED_SITL_PARTICIPANT'):
            executor.graph_check()


if __name__ == '__main__':
    unittest.main()
