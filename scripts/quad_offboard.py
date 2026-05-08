#!/usr/bin/env python3
"""X500 multicopter offboard controller with takeoff/hover/command logic.

State flow:
    WARMUP_SETPOINTS -> REQUEST_OFFBOARD_ARM -> MC_TAKEOFF -> MC_HOVER

From MC_HOVER:
    fresh command [vx, vy, 1] -> MC_COMMAND

From MC_COMMAND:
    command timeout -> MC_HOVER

From any state:
    land_topic -> LANDING
"""

from __future__ import annotations

import math
from enum import IntEnum
from typing import Optional

import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, VehicleControlMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Float32MultiArray


class FlightState(IntEnum):
    WARMUP_SETPOINTS = 0
    REQUEST_OFFBOARD_ARM = 1
    MC_TAKEOFF = 2
    MC_HOVER = 3
    MC_COMMAND = 4
    LANDING = 5


class X500Offboard(Node):
    """PX4 offboard controller for multicopter X500 fleets."""

    def __init__(self) -> None:
        super().__init__("x500_offboard")

        instance = int(self.declare_parameter("instance", 0).value)
        self.fmu_prefix = "" if instance == 0 else f"/px4_{instance}"
        self.target_system = instance + 1
        self.target_component = 1

        self.setpoint_rate_hz = float(
            self.declare_parameter("setpoint_rate_hz", 20.0).value
        )
        self.offboard_warmup_cycles = int(
            self.declare_parameter("offboard_warmup_cycles", 20).value
        )
        self.rearm_interval_s = float(
            self.declare_parameter("rearm_interval_s", 2.0).value
        )

        self.takeoff_height_m = float(
            self.declare_parameter("takeoff_height_m", 15.0).value
        )
        self.takeoff_reached_tol_m = float(
            self.declare_parameter("takeoff_reached_tol_m", 0.6).value
        )
        self.hover_yaw_rad = float(self.declare_parameter("hover_yaw_rad", 0.0).value)

        self.command_topic = str(
            self.declare_parameter("command_topic", "command").value
        )
        self.cmd_timeout_s = float(self.declare_parameter("cmd_timeout_s", 3.0).value)
        self.command_speed_mps = float(
            self.declare_parameter("command_speed_mps", 3.0).value
        )
        self.max_speed_mps = float(self.declare_parameter("max_speed_mps", 5.0).value)

        self.land_topic = str(
            self.declare_parameter("land_topic", "land_request").value
        )

        self.state = FlightState.WARMUP_SETPOINTS
        self.warmup_counter = 0
        self._last_arm_request_us = 0
        self._last_offboard_request_us = 0
        self._status_log_counter = 0
        self._targets_initialized = False

        self.takeoff_x = 0.0
        self.takeoff_y = 0.0
        self.takeoff_z = -abs(self.takeoff_height_m)

        self.hover_x = 0.0
        self.hover_y = 0.0
        self.hover_z = self.takeoff_z
        self.hover_yaw = self.hover_yaw_rad

        self.cmd_dir_x: Optional[float] = None
        self.cmd_dir_y: Optional[float] = None
        self.cmd_stamp_us: Optional[int] = None
        self.cmd_active = False

        self.pending_land = False
        self.landing_sent = False

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vehicle_control_mode = VehicleControlMode()

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, self._topic("/fmu/in/offboard_control_mode"), px4_qos
        )
        self.trajectory_sp_pub = self.create_publisher(
            TrajectorySetpoint, self._topic("/fmu/in/trajectory_setpoint"), px4_qos
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, self._topic("/fmu/in/vehicle_command"), px4_qos
        )

        self.create_subscription(
            VehicleLocalPosition,
            self._topic("/fmu/out/vehicle_local_position"),
            self._local_pos_cb,
            px4_qos,
        )
        self.create_subscription(
            VehicleStatus,
            self._topic("/fmu/out/vehicle_status_v1"),
            self._status_cb,
            px4_qos,
        )
        self.create_subscription(
            VehicleControlMode,
            self._topic("/fmu/out/vehicle_control_mode"),
            self._ctrl_mode_cb,
            px4_qos,
        )
        self.create_subscription(
            Float32MultiArray, self.command_topic, self._command_cb, 10
        )
        self.create_subscription(Empty, self.land_topic, self._land_cb, 10)

        period_s = 1.0 / max(self.setpoint_rate_hz, 1.0)
        self.create_timer(period_s, self._timer_cb)

        self.get_logger().info(
            f"X500 offboard started - instance={instance} "
            f"fmu_prefix='{self.fmu_prefix}' target_system={self.target_system}"
        )
        self.get_logger().info(
            f"command_topic='{self.command_topic}' cmd_timeout_s={self.cmd_timeout_s:.1f} "
            f"command_speed_mps={self.command_speed_mps:.2f} max_speed_mps={self.max_speed_mps:.2f}"
        )

    def _topic(self, suffix: str) -> str:
        prefix = self.fmu_prefix.strip()
        if prefix and not prefix.startswith("/"):
            prefix = "/" + prefix
        return f"{prefix}{suffix}"

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _set_state(self, new_state: FlightState) -> None:
        if self.state == new_state:
            return
        self.get_logger().info(f"State: {self.state.name} -> {new_state.name}")
        self.state = new_state

        if new_state == FlightState.MC_HOVER:
            self._latch_hover_target()
        elif new_state == FlightState.LANDING:
            self.landing_sent = False

    def _initialize_targets_if_needed(self) -> None:
        if self._targets_initialized:
            return

        pos = self.vehicle_local_position
        xy_valid = bool(getattr(pos, "xy_valid", False))

        if xy_valid:
            self.takeoff_x = float(pos.x)
            self.takeoff_y = float(pos.y)
        else:
            self.takeoff_x = 0.0
            self.takeoff_y = 0.0

        self.takeoff_z = -abs(self.takeoff_height_m)
        self.hover_x = self.takeoff_x
        self.hover_y = self.takeoff_y
        self.hover_z = self.takeoff_z
        self.hover_yaw = self.hover_yaw_rad
        self._targets_initialized = True

        source = "vehicle_local_position" if xy_valid else "fallback origin"
        self.get_logger().info(
            f"Takeoff reference initialized from {source}: "
            f"x={self.takeoff_x:.2f} y={self.takeoff_y:.2f} z={self.takeoff_z:.2f}"
        )

    def _latch_hover_target(self) -> None:
        pos = self.vehicle_local_position
        if bool(getattr(pos, "xy_valid", False)):
            self.hover_x = float(pos.x)
            self.hover_y = float(pos.y)
        if bool(getattr(pos, "z_valid", False)):
            self.hover_z = float(pos.z)
        if math.isfinite(float(pos.heading)):
            self.hover_yaw = float(pos.heading)

        self.get_logger().info(
            f"Hover target latched: x={self.hover_x:.2f} y={self.hover_y:.2f} "
            f"z={self.hover_z:.2f} yaw={self.hover_yaw:.2f}"
        )

    def _is_offboard(self) -> bool:
        # return self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        return self.vehicle_control_mode.flag_control_offboard_enabled

    def _is_armed(self) -> bool:
        # return self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        return self.vehicle_control_mode.flag_armed

    def _takeoff_reached(self) -> bool:
        if not bool(getattr(self.vehicle_local_position, "z_valid", False)):
            return False
        return (
            abs(float(self.vehicle_local_position.z) - self.takeoff_z)
            <= self.takeoff_reached_tol_m
        )

    def _cmd_is_fresh(self, now_us: int) -> bool:
        if self.cmd_stamp_us is None:
            return False
        timeout_us = int(max(self.cmd_timeout_s, 0.0) * 1e6)
        return (now_us - self.cmd_stamp_us) < timeout_us

    def _heartbeat(self, position_ctrl: bool, velocity_ctrl: bool) -> None:
        msg = OffboardControlMode()
        msg.position = position_ctrl
        msg.velocity = velocity_ctrl
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self.offboard_mode_pub.publish(msg)

    def _pub_position_setpoint(self, x: float, y: float, z: float, yaw: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity = [float("nan")] * 3
        msg.acceleration = [float("nan")] * 3
        msg.jerk = [float("nan")] * 3
        msg.yaw = yaw
        msg.yawspeed = 0.0
        msg.timestamp = self._now_us()
        self.trajectory_sp_pub.publish(msg)

    def _pub_velocity_setpoint(self, vx: float, vy: float, yaw: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = [float("nan")] * 3
        msg.velocity = [vx, vy, 0.0]
        msg.acceleration = [float("nan")] * 3
        msg.jerk = [float("nan")] * 3
        msg.yaw = yaw
        msg.yawspeed = 0.0
        msg.timestamp = self._now_us()
        self.trajectory_sp_pub.publish(msg)

    def _pub_vehicle_cmd(self, command: int, **params: float) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(params.get("param1", 0.0))
        msg.param2 = float(params.get("param2", 0.0))
        msg.param3 = float(params.get("param3", 0.0))
        msg.param4 = float(params.get("param4", 0.0))
        msg.param5 = float(params.get("param5", 0.0))
        msg.param6 = float(params.get("param6", 0.0))
        msg.param7 = float(params.get("param7", 0.0))
        msg.target_system = self.target_system
        msg.target_component = self.target_component
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self.vehicle_command_pub.publish(msg)

    def _engage_offboard(self) -> None:
        self._pub_vehicle_cmd(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )

    def _arm(self) -> None:
        self._pub_vehicle_cmd(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )

    def _cmd_land(self) -> None:
        self._pub_vehicle_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def _maybe_request_arm_and_offboard(self, now_us: int) -> None:
        min_interval_us = int(max(self.rearm_interval_s, 0.1) * 1e6)

        if now_us - self._last_arm_request_us >= min_interval_us:
            self._arm()
            self._last_arm_request_us = now_us

        if now_us - self._last_offboard_request_us >= min_interval_us:
            self._engage_offboard()
            self._last_offboard_request_us = now_us

    def _log_status_throttled(self) -> None:
        status_log_interval = max(int(self.setpoint_rate_hz * 5.0), 1)
        self._status_log_counter += 1
        if self._status_log_counter < status_log_interval:
            return
        self._status_log_counter = 0
        self.get_logger().info(
            f"vehicle status: "
            f"(armed={self._is_armed()}) "
            f"(offboard={self._is_offboard()})"
        )

    def _local_pos_cb(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg

    def _status_cb(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg
    
    def _ctrl_mode_cb(self, msg: VehicleControlMode) -> None:
        self.vehicle_control_mode = msg

    def _land_cb(self, _: Empty) -> None:
        self.pending_land = True

    def _command_cb(self, msg: Float32MultiArray) -> None:
        """Accept compatibility format: [vx, vy, 1]."""
        if len(msg.data) < 3:
            self.get_logger().warn(
                "Invalid command format. Expected [vx, vy, 1], "
                f"received: {list(msg.data)}"
            )
            return

        cmd_x = float(msg.data[0])
        cmd_y = float(msg.data[1])
        raw_mode = float(msg.data[2])

        if not (
            math.isfinite(cmd_x) and math.isfinite(cmd_y) and math.isfinite(raw_mode)
        ):
            self.get_logger().warn(f"Invalid command values: {list(msg.data)}")
            return

        if not raw_mode.is_integer() or int(raw_mode) != 1:
            self.get_logger().warn(
                "Unsupported command mode for X500 node. "
                f"Expected mode=1, received mode={raw_mode}"
            )
            return

        norm = math.hypot(cmd_x, cmd_y)
        if norm <= 1e-6:
            self.get_logger().warn(
                f"Invalid velocity direction (near-zero norm): {list(msg.data)}"
            )
            return

        self.cmd_dir_x = cmd_x / norm
        self.cmd_dir_y = cmd_y / norm
        self.cmd_stamp_us = self._now_us()
        if not self.cmd_active:
            self.get_logger().info("Command override enabled")
        self.cmd_active = True

    def _timer_cb(self) -> None:
        self._initialize_targets_if_needed()
        self._log_status_throttled()

        if self.pending_land and self.state != FlightState.LANDING:
            self.pending_land = False
            self._set_state(FlightState.LANDING)

        now_us = self._now_us()

        if self.state == FlightState.WARMUP_SETPOINTS:
            self._heartbeat(position_ctrl=True, velocity_ctrl=False)
            self._pub_position_setpoint(
                self.takeoff_x,
                self.takeoff_y,
                self.takeoff_z,
                self.hover_yaw_rad,
            )
            self.warmup_counter += 1
            if self.warmup_counter >= self.offboard_warmup_cycles:
                self._set_state(FlightState.REQUEST_OFFBOARD_ARM)
            return

        if self.state == FlightState.REQUEST_OFFBOARD_ARM:
            self._heartbeat(position_ctrl=True, velocity_ctrl=False)
            self._pub_position_setpoint(
                self.takeoff_x,
                self.takeoff_y,
                self.takeoff_z,
                self.hover_yaw_rad,
            )
            self._maybe_request_arm_and_offboard(now_us)

            if self._is_armed() and self._is_offboard():
                self._set_state(FlightState.MC_TAKEOFF)
            return

        if self.state == FlightState.MC_TAKEOFF:
            self._heartbeat(position_ctrl=True, velocity_ctrl=False)
            self._pub_position_setpoint(
                self.takeoff_x,
                self.takeoff_y,
                self.takeoff_z,
                self.hover_yaw_rad,
            )
            self._maybe_request_arm_and_offboard(now_us)

            if self._takeoff_reached():
                self._set_state(FlightState.MC_HOVER)
            return

        if self.state == FlightState.MC_HOVER:
            self._heartbeat(position_ctrl=True, velocity_ctrl=False)
            self._pub_position_setpoint(
                self.hover_x,
                self.hover_y,
                self.hover_z,
                self.hover_yaw,
            )
            self._maybe_request_arm_and_offboard(now_us)

            if (
                self._cmd_is_fresh(now_us)
                and self.cmd_dir_x is not None
                and self.cmd_dir_y is not None
            ):
                self._set_state(FlightState.MC_COMMAND)
            return

        if self.state == FlightState.MC_COMMAND:
            cmd_fresh = self._cmd_is_fresh(now_us)
            if not cmd_fresh or self.cmd_dir_x is None or self.cmd_dir_y is None:
                if self.cmd_active:
                    self.get_logger().info("Command timeout - fallback to hover")
                    self.cmd_active = False
                self._set_state(FlightState.MC_HOVER)
                return

            speed = max(self.command_speed_mps, 0.0)
            if self.max_speed_mps > 0.0:
                speed = min(speed, self.max_speed_mps)
            vx = self.cmd_dir_x * speed
            vy = self.cmd_dir_y * speed
            yaw = math.atan2(vy, vx) if speed > 1e-6 else self.hover_yaw

            self._heartbeat(position_ctrl=False, velocity_ctrl=True)
            self._pub_velocity_setpoint(vx, vy, yaw)
            self._maybe_request_arm_and_offboard(now_us)
            return

        if self.state == FlightState.LANDING:
            if not self.landing_sent:
                self.get_logger().info("Land request received - sending NAV_LAND")
                self._cmd_land()
                self.landing_sent = True
            return


def main(args=None) -> None:
    node = None
    try:
        rclpy.init(args=args)
        node = X500Offboard()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
