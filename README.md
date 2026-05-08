# px4_state_machine

ROS 2 package for PX4 offboard control and fleet TF broadcasting.

## What is in this package

- `scripts/vtol_offboard.py`: VTOL offboard state machine node.
- `scripts/quad_offboard.py`: multicopter (X500-style) offboard state machine node.
- `scripts/lawnmower_offboard.py`: lawnmower/rover-style offboard hold node.
- `scripts/fleet_tf_broadcaster_node.py`: fleet-level `map -> odom_i` and `odom_i -> base_link_i` TF broadcaster.
- `launch/vtol_state_machine.launch.py`: launch VTOL controllers for one or more PX4 instances.
- `launch/quad_state_machine.launch.py`: launch multicopter controllers for one or more PX4 instances.
- `launch/lawnmower_offboard.launch.py`: launch lawnmower controllers for one or more PX4 instances.
- `launch/fleet_tf_broadcaster.launch.py`: launch fleet TF broadcaster, with optional RViz.

## Build

From workspace root (`ws`):

```bash
colcon build --packages-up-to px4_state_machine
source install/setup.bash
```

## Launch examples

VTOL controllers:

```bash
ros2 launch px4_state_machine vtol_state_machine.launch.py px4_instances:=1,2
```

Multicopter controllers:

```bash
ros2 launch px4_state_machine quad_state_machine.launch.py px4_instances:=1,2
```

Lawnmower controllers:

```bash
ros2 launch px4_state_machine lawnmower_offboard.launch.py px4_instances:=4,5,6
```

Fleet TF broadcaster (RViz enabled by default):

```bash
ros2 launch px4_state_machine fleet_tf_broadcaster.launch.py
```

Fleet TF broadcaster without RViz:

```bash
ros2 launch px4_state_machine fleet_tf_broadcaster.launch.py use_rviz:=false
```

## Config notes

- Per-instance offboard config files are supported via `*_params_<instance>.yaml` with fallback to `*_params_default.yaml` in `config/`.
- Fleet TF defaults are in `config/fleet_tf.yaml`.
- RViz config used by fleet TF launch is `rviz/default.rviz`.

For detailed topic/parameter behavior of the offboard nodes, see `README_offboard.md`.
