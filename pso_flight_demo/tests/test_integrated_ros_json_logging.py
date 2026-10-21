import json
import unittest

import numpy as np

from pso_flight_demo.integrated_ros import _json_safe


class IntegratedRosJsonLoggingTests(unittest.TestCase):
    def test_numpy_scalar_fields_from_ros_are_json_serializable(self):
        fields = {
            'heading': np.float32(1.25),
            'valid': np.bool_(True),
            'counter': np.uint8(3),
            'q': np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            # rosidl's fixed-size float sequence supports list(field), which
            # yields NumPy float32 scalar elements in this px4_msgs build.
            'position_ned': list(np.asarray([1.0, 2.0, -3.0], dtype=np.float32)),
        }

        encoded = json.dumps(_json_safe(fields), allow_nan=False)

        self.assertEqual(json.loads(encoded), {
            'heading': 1.25,
            'valid': True,
            'counter': 3,
            'q': [1.0, 0.0, 0.0, 0.0],
            'position_ned': [1.0, 2.0, -3.0],
        })

    def test_nonfinite_numpy_scalars_remain_strict_json_null(self):
        encoded = json.dumps(_json_safe({'value': np.float32(np.nan)}), allow_nan=False)

        self.assertEqual(json.loads(encoded), {'value': None})


if __name__ == '__main__':
    unittest.main()
