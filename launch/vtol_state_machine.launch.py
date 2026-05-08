import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_instances(context) -> list[int]:
    """Resolve the PX4 instance list from launch arguments.

    Priority (highest to lowest):
    1. ``px4_instances``: explicit comma-separated list, e.g. ``"1,2,3"`` or ``"10,11"``.
    2. ``px4_instance_offset`` + ``num_robots``: offset-based shorthand,
       e.g. offset=1 + n=3  →  [1, 2, 3].
    3. ``num_robots`` alone (legacy): treated as offset=0, giving [0, 1, …, n-1].
    """
    raw_instances = LaunchConfiguration("px4_instances").perform(context).strip()
    if raw_instances:
        return sorted(int(x.strip()) for x in raw_instances.split(",") if x.strip())

    n = int(LaunchConfiguration("num_robots").perform(context))
    offset = int(LaunchConfiguration("px4_instance_offset").perform(context))
    return list(range(offset, offset + n))


def _instance_to_prefix(inst: int) -> str:
    """Convert a PX4 instance number to a ROS 2 topic/namespace prefix.

    Instance 0 → ``""``  (unprefixed, compatible with single-robot convention)
    Instance N → ``"/px4_N"``
    """
    return "" if inst == 0 else f"/px4_{inst}"


def _check_yaml_instance(config_path: str, expected_inst: int) -> None:
    """Warn if the YAML file's declared instance doesn't match the launch-time instance."""
    import logging

    logger = logging.getLogger(__name__)
    try:
        with open(config_path) as f:
            data = yaml.safe_load(f) or {}
        # The YAML may be flat or node-scoped (any top-level key → ros__parameters).
        ros_params = None
        for v in data.values():
            if isinstance(v, dict) and "ros__parameters" in v:
                ros_params = v["ros__parameters"]
                break
            elif isinstance(v, dict):
                ros_params = v
                break
        if ros_params and "instance" in ros_params:
            yaml_inst = int(ros_params["instance"])
            if yaml_inst != expected_inst:
                print(
                    f"[px4_state_machine launch] WARNING: "
                    f"'{os.path.basename(config_path)}' declares instance={yaml_inst} "
                    f"but launch expects instance={expected_inst} — launch value takes precedence."
                )
    except Exception as exc:
        logger.warning(f"Could not validate instance in '{config_path}': {exc}")


def launch_setup(context, *args, **kwargs):
    instances = _parse_instances(context)
    config_dir = LaunchConfiguration("offboard_config_dir").perform(context)

    nodes = []
    for seq_idx, inst in enumerate(instances):
        prefix = _instance_to_prefix(inst)

        # Look up offboard_params_<inst>.yaml first; fall back to the default.
        instance_cfg = os.path.join(config_dir, f"offboard_params_{inst}.yaml")
        fallback_cfg = os.path.join(config_dir, "offboard_params_default.yaml")
        config_path = instance_cfg if os.path.exists(instance_cfg) else fallback_cfg

        _check_yaml_instance(config_path, inst)
        print(
            f"[px4_state_machine launch] instance {inst}: "
            f"loading config '{os.path.basename(config_path)}' ({config_path})",
            flush=True,
        )

        nodes.append(
            Node(
                package="px4_state_machine",
                executable="vtol_offboard.py",
                name=f"vtol_sm_{inst}",
                namespace=prefix,
                output="screen",
                parameters=[
                    config_path,
                    # Only inject instance — fmu_prefix and target_system are
                    # derived inside vtol_offboard.py from this single value.
                    {"instance": inst},
                ],
            )
        )

    return nodes


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("px4_state_machine")
    default_config_dir = os.path.join(package_share, "config")

    instances = "1,2"

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            DeclareLaunchArgument(
                "px4_instances",
                default_value=instances,
                description=(
                    "Comma-separated list of PX4 instance numbers, e.g. '1,2,3'. "
                    "When set, overrides num_robots and px4_instance_offset."
                ),
            ),
            DeclareLaunchArgument(
                "num_robots",
                default_value="1",
                description=(
                    "Number of offboard state machine nodes to spawn. "
                    "Used together with px4_instance_offset when px4_instances is not set."
                ),
            ),
            DeclareLaunchArgument(
                "px4_instance_offset",
                default_value="0",
                description=(
                    "PX4 instance number for the first robot. Subsequent robots get "
                    "offset+1, offset+2, ... "
                    "Set to 1 when your fleet uses instances 1,2,3,... (recommended for multi-robot). "
                    "Ignored if px4_instances is set explicitly."
                ),
            ),
            DeclareLaunchArgument(
                "offboard_config_dir",
                default_value=default_config_dir,
                description=(
                    "Directory containing per-robot offboard_params_<instance>.yaml config files. "
                    "Falls back to offboard_params_default.yaml if no per-instance file is found."
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
