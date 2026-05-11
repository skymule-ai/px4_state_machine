from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("px4_state_machine")
    default_params = f"{package_share}/config/fleet_tf.yaml"
    default_node_name = "fleet_tf_broadcaster"
    default_rviz = f"{package_share}/rviz/default.rviz"

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to fleet TF broadcaster params yaml",
    )
    node_name_arg = DeclareLaunchArgument(
        "node_name",
        default_value=default_node_name,
        description="ROS node name for fleet TF broadcaster",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="Path to RViz config file",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Whether to launch RViz2",
    )

    tf_node = Node(
        package="px4_state_machine",
        executable="fleet_tf_broadcaster_node.py",
        name=LaunchConfiguration("node_name"),
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [params_file_arg, 
        rviz_config_arg, 
        use_rviz_arg, 
        node_name_arg, 
        tf_node, 
        rviz_node,
        ]
    )
