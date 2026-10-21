"""Read installation/schema evidence without initializing ROS or executing Agent."""

import os
from pathlib import Path
import re
import shutil


def inventory(profile):
    root = Path(profile.px4_root).expanduser()
    result = dict(ros_distro=os.environ.get("ROS_DISTRO"), rclpy_importable=False,
                  agent_executable=shutil.which("MicroXRCEAgent"), schemas={}, errors=[],
                  ros_initialized=False, publishes_commands=False)
    config = root/"build/px4_sitl_default/src/modules/uxrce_dds_client/include/uxr/client/config.h"
    if config.is_file():
        version = re.search(r'UXR_CLIENT_VERSION_STR\s+"([^"]+)"', config.read_text())
        result["px4_xrce_client_version"] = version[1] if version else None
    try:
        import rclpy
        result["rclpy_importable"] = True
        result["rclpy_path"] = rclpy.__file__
        import px4_msgs.msg as messages
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("px4_msgs"))
        result["px4_msgs_share"] = str(share)
        for name in ("OffboardControlMode", "TrajectorySetpoint", "VehicleLocalPosition",
                     "VehicleStatus", "VehicleCommand", "VehicleCommandAck", "TimesyncStatus",
                     "VehicleAttitude", "VehicleLandDetected", "EstimatorStatusFlags", "FailsafeFlags"):
            cls = getattr(messages, name)
            installed = share/"msg"/f"{name}.msg"
            source = root/"msg/versioned"/f"{name}.msg"
            if not source.is_file():
                source = root/"msg"/f"{name}.msg"
            def wire(p):
                lines = [re.sub(r"\s+", " ", s.split("#")[0].strip()) for s in p.read_text().splitlines()]
                return [s for s in lines if s and "=" not in s]
            snake = re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()
            header = root/"build/px4_sitl_default/uORB/topics"/f"{snake}.h"
            built_version = 0
            if header.is_file():
                match = re.search(r"MESSAGE_VERSION\s*=\s*(\d+)", header.read_text())
                built_version = int(match[1]) if match else 0
            version = int(getattr(cls, "MESSAGE_VERSION", 0))
            result["schemas"][name] = dict(serialized_fields_match=wire(installed) == wire(source),
                                          ros_version=version, px4_built_version=built_version,
                                          built_version_matches=header.is_file() and version == built_version)
    except (ImportError, OSError, AttributeError, LookupError, RuntimeError) as exc:
        result["errors"].append(f"{type(exc).__name__}: {exc}")
    return result
