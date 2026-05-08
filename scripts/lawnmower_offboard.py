#!/usr/bin/env python3
"""
Lawnmower Offboard Velocity
===========================
Arms a PX4 rover, switches to offboard mode, and streams velocity setpoints
based on a commanded direction or velocity.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleStatus


class LawnmowerOffboard(Node):
    def __init__(self) -> None:
        super().__init__("lawnmower_offboard")

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
        self.command_topic = str(
            self.declare_parameter("command_topic", "command").value
        )
        self.cmd_timeout_s = float(self.declare_parameter("cmd_timeout_s", 3.0).value)
        self.max_speed_mps = float(self.declare_parameter("max_speed_mps", 0.0).value)
        if self.max_speed_mps <= 0.0:
            self.max_speed_mps = 0.0

        self._warmup_counter = 0
        self._last_arm_request_us = 0
        self._last_offboard_request_us = 0
        self._status_log_counter = 0
        self.cmd_vx: Optional[float] = None
        self.cmd_vy: Optional[float] = None
        self.cmd_yaw: Optional[float] = None
        self.cmd_omega: Optional[float] = None
        self._cmd_active = False
        self.cmd_stamp_us: Optional[int] = None
        self.vehicle_status = VehicleStatus()

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
            VehicleStatus,
            self._topic("/fmu/out/vehicle_status_v1"),
            self._status_cb,
            px4_qos,
        )
        self.create_subscription(
            Float32MultiArray, self.command_topic, self._command_cb, 10
        )

        period_s = 1.0 / max(self.setpoint_rate_hz, 1.0)
        self.create_timer(period_s, self._timer_cb)

        self.get_logger().info(
            f"Lawnmower offboard started — instance={instance} "
            f"fmu_prefix='{self.fmu_prefix}' target_system={self.target_system}"
        )
        self.get_logger().info(
            "Command override enabled: "
            f"command_topic='{self.command_topic}' "
            f"cmd_timeout_s={self.cmd_timeout_s:.1f} "
            f"max_speed_mps={self.max_speed_mps:.2f}"
        )

    def _topic(self, suffix: str) -> str:
        prefix = self.fmu_prefix.strip()
        if prefix and not prefix.startswith("/"):
            prefix = "/" + prefix
        return f"{prefix}{suffix}"

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _cmd_is_fresh(self, now_us: int) -> bool:
        if self.cmd_stamp_us is None:
            return False
        timeout_us = int(max(self.cmd_timeout_s, 0.0) * 1e6)
        return (now_us - self.cmd_stamp_us) < timeout_us

    def _status_cb(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    def _command_cb(self, msg: Float32MultiArray) -> None:
        """
        Expected format:
        - [vx, vy, yaw, omega] => velocity (m/s) + yaw (rad) + yaw rate (rad/s)
        """
        if len(msg.data) < 4:
            self.get_logger().warn(f"Invalid command: '{msg.data}'")
            return

        cmd_vx = float(msg.data[0])
        cmd_vy = float(msg.data[1])
        cmd_yaw = float(msg.data[2])
        cmd_omega = float(msg.data[3])
        if not (
            math.isfinite(cmd_vx)
            and math.isfinite(cmd_vy)
            and math.isfinite(cmd_yaw)
            and math.isfinite(cmd_omega)
        ):
            self.get_logger().warn(f"Invalid command values: '{msg.data}'")
            return
        self.cmd_vx = cmd_vx
        self.cmd_vy = cmd_vy
        self.cmd_yaw = cmd_yaw
        self.cmd_omega = cmd_omega
        self.cmd_stamp_us = self._now_us()
        if not self._cmd_active:
            self._cmd_active = True
        self.get_logger().info(
            "Command override enabled: "
            f"cmd=({self.cmd_vx:.3f}, {self.cmd_vy:.3f}, "
            f"{self.cmd_yaw:.3f}, {self.cmd_omega:.3f})"
        )

    def _heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self.offboard_mode_pub.publish(msg)

    def _pub_stop_setpoint(self) -> None:
        msg = TrajectorySetpoint()
        msg.position = [float("nan")] * 3
        msg.velocity = [0.0, 0.0, 0.0]
        msg.acceleration = [float("nan")] * 3
        msg.jerk = [float("nan")] * 3
        msg.yaw = self.cmd_yaw if self.cmd_yaw is not None else float("nan")
        msg.yawspeed = 0.0
        msg.timestamp = self._now_us()
        self.trajectory_sp_pub.publish(msg)

    def _pub_velocity_setpoint(
        self, vx: float, vy: float, yaw: float, omega: float
    ) -> None:
        msg = TrajectorySetpoint()
        msg.position = [float("nan")] * 3
        msg.velocity = [vx, vy, 0.0]
        msg.acceleration = [float("nan")] * 3
        msg.jerk = [float("nan")] * 3
        msg.yaw = yaw
        msg.yawspeed = omega
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
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )

    def _arm(self) -> None:
        self._pub_vehicle_cmd(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )

    def _timer_cb(self) -> None:
        self._heartbeat()

        now_us = self._now_us()
        cmd_fresh = self._cmd_is_fresh(now_us)
        if (
            cmd_fresh
            and self.cmd_vx is not None
            and self.cmd_vy is not None
            and self.cmd_yaw is not None
            and self.cmd_omega is not None
        ):
            vx = float(self.cmd_vx)
            vy = float(self.cmd_vy)
            yaw = float(self.cmd_yaw)
            omega = float(self.cmd_omega)
            # if self.max_speed_mps > 1e-6:
            #     speed = math.hypot(vx, vy)
            #     if speed > self.max_speed_mps:
            #         scale = self.max_speed_mps / max(speed, 1e-6)
            #         vx *= scale
            #         vy *= scale
            self._pub_velocity_setpoint(vx, vy, yaw, omega)
        else:
            if self._cmd_active:
                self.get_logger().info("Command timeout -> stopping")
                self._cmd_active = False
            self._pub_stop_setpoint()

        # Throttled status log: every 5 s (rate_hz * 5 ticks).
        status_log_interval = max(int(self.setpoint_rate_hz * 5.0), 1)
        self._status_log_counter += 1
        if self._status_log_counter >= status_log_interval:
            self._status_log_counter = 0
            self.get_logger().info(
                f"vehicle status: arming_state={self.vehicle_status.arming_state}"
                f" (armed={self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED})"
                f" nav_state={self.vehicle_status.nav_state}"
                f" (offboard={self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD})"
            )

        if self._warmup_counter < self.offboard_warmup_cycles:
            self._warmup_counter += 1
            return

        min_interval_us = int(max(self.rearm_interval_s, 0.1) * 1e6)

        # Arm first, then request OFFBOARD — PX4 may reject mode switch on a
        # disarmed vehicle, so arm command goes out one message ahead.
        if now_us - self._last_arm_request_us >= min_interval_us:
            self._arm()
            self._last_arm_request_us = now_us

        if now_us - self._last_offboard_request_us >= min_interval_us:
            self._engage_offboard()
            self._last_offboard_request_us = now_us


def main(args=None) -> None:
    node = None
    try:
        rclpy.init(args=args)
        node = LawnmowerOffboard()
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
