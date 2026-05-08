import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_instances(context) -> list[int]:
    raw_instances = LaunchConfiguration("px4_instances").perform(context).strip()
    if raw_instances:
        return sorted(int(x.strip()) for x in raw_instances.split(",") if x.strip())

    n = int(LaunchConfiguration("num_robots").perform(context))
    offset = int(LaunchConfiguration("px4_instance_offset").perform(context))
    return list(range(offset, offset + n))


def _instance_to_prefix(inst: int) -> str:
    return "" if inst == 0 else f"/px4_{inst}"


def _check_yaml_instance(config_path: str, expected_inst: int) -> None:
    import logging

    logger = logging.getLogger(__name__)
    try:
        with open(config_path) as f:
            data = yaml.safe_load(f) or {}
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
                    f"[lawnmower_offboard launch] WARNING: "
                    f"'{os.path.basename(config_path)}' declares instance={yaml_inst} "
                    f"but launch expects instance={expected_inst} — launch value takes precedence."
                )
    except Exception as exc:
        logger.warning(f"Could not validate instance in '{config_path}': {exc}")


def launch_setup(context, *args, **kwargs):
    instances = _parse_instances(context)
    config_dir = LaunchConfiguration("offboard_config_dir").perform(context)

    nodes = []
    for inst in instances:
        prefix = _instance_to_prefix(inst)
        instance_cfg = os.path.join(
            config_dir, f"lawnmower_offboard_params_{inst}.yaml"
        )
        fallback_cfg = os.path.join(
            config_dir, "lawnmower_offboard_params_default.yaml"
        )
        config_path = instance_cfg if os.path.exists(instance_cfg) else fallback_cfg

        _check_yaml_instance(config_path, inst)
        print(
            f"[lawnmower_offboard launch] instance {inst}: "
            f"loading config '{os.path.basename(config_path)}' ({config_path})",
            flush=True,
        )

        nodes.append(
            Node(
                package="px4_state_machine",
                executable="lawnmower_offboard.py",
                name=f"lawnmower_offboard_{inst}",
                namespace=prefix,
                output="screen",
                parameters=[
                    config_path,
                    {"instance": inst},
                ],
            )
        )

    return nodes


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("px4_state_machine")
    default_config_dir = os.path.join(package_share, "config")

    instances = "3,4,5,6"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "px4_instances",
                default_value=instances,
                description=(
                    "Comma-separated list of PX4 instance numbers, e.g. '4,5,6'. "
                    "When set, overrides num_robots and px4_instance_offset."
                ),
            ),
            DeclareLaunchArgument(
                "num_robots",
                default_value="1",
                description=(
                    "Number of offboard nodes to spawn. "
                    "Used together with px4_instance_offset when px4_instances is not set."
                ),
            ),
            DeclareLaunchArgument(
                "px4_instance_offset",
                default_value="0",
                description=(
                    "PX4 instance number for the first robot. Subsequent robots get "
                    "offset+1, offset+2, ... Ignored if px4_instances is set explicitly."
                ),
            ),
            DeclareLaunchArgument(
                "offboard_config_dir",
                default_value=default_config_dir,
                description=(
                    "Directory containing per-robot lawnmower_offboard_params_<instance>.yaml files. "
                    "Falls back to lawnmower_offboard_params_default.yaml if no per-instance file exists."
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
