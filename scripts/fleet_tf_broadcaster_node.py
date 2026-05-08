#!/usr/bin/env python3

import math
from typing import Dict, List, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleGlobalPosition, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def normalize_suffix(suffix: str) -> str:
    if not suffix:
        return ""
    return suffix if suffix.startswith("/") else f"/{suffix}"


def compose_topic(topic_prefix: str, robot_id: int, topic_suffix: str) -> str:
    suffix = normalize_suffix(topic_suffix)
    prefix = topic_prefix[1:] if topic_prefix.startswith("/") else topic_prefix
    return f"/{prefix}{robot_id}{suffix}"


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def ned_to_enu_xyz(
    north: float, east: float, down: float
) -> Tuple[float, float, float]:
    return (east, north, -down)


def ned_yaw_to_enu_yaw(yaw_ned: float) -> float:
    return (math.pi / 2.0) - yaw_ned


def geodetic_to_enu(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_alt_m: float,
) -> Tuple[float, float, float]:
    earth_radius_m = 6378137.0
    deg_to_rad = math.pi / 180.0

    d_lat = (lat_deg - ref_lat_deg) * deg_to_rad
    d_lon = (lon_deg - ref_lon_deg) * deg_to_rad
    lat_ref_rad = ref_lat_deg * deg_to_rad

    north = d_lat * earth_radius_m
    east = d_lon * earth_radius_m * math.cos(lat_ref_rad)
    up = alt_m - ref_alt_m
    return (east, north, up)


class FleetTfBroadcasterNode(Node):
    def __init__(self) -> None:
        super().__init__("fleet_tf_broadcaster")

        self.topic_prefix = str(self.declare_parameter("topic_prefix", "px4_").value)
        self.robot_ids = [
            int(v) for v in self.declare_parameter("robot_ids", [1, 2]).value
        ]
        self.anchor_robot_id = int(self.declare_parameter("anchor_robot_id", 1).value)
        self.map_frame = str(self.declare_parameter("map_frame", "map").value)
        self.odom_frame_prefix = str(
            self.declare_parameter("odom_frame_prefix", "odom_").value
        )
        self.base_link_frame_prefix = str(
            self.declare_parameter("base_link_frame_prefix", "base_link_").value
        )
        self.global_position_topic_name = str(
            self.declare_parameter(
                "global_position_topic_name", "/fmu/out/vehicle_global_position"
            ).value
        )
        self.odom_topic_name = str(
            self.declare_parameter("odom_topic_name", "/fmu/out/vehicle_odometry").value
        )
        self.init_sample_count = int(
            self.declare_parameter("init_sample_count", 20).value
        )
        self.max_eph = float(self.declare_parameter("max_eph", 10.0).value)
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 30.0).value
        )

        if not self.robot_ids:
            raise ValueError("robot_ids must not be empty")
        self.robot_ids = list(dict.fromkeys(self.robot_ids))
        if self.anchor_robot_id not in self.robot_ids:
            raise ValueError("anchor_robot_id must be in robot_ids")
        if self.init_sample_count <= 0:
            raise ValueError("init_sample_count must be > 0")
        if self.max_eph < 0.0:
            raise ValueError("max_eph must be >= 0")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")

        self.global_samples: Dict[int, List[Tuple[float, float, float]]] = {
            rid: [] for rid in self.robot_ids
        }
        self.local_pose_samples: Dict[int, List[Tuple[float, float]]] = {
            rid: [] for rid in self.robot_ids
        }
        self.latest_odom: Dict[int, VehicleOdometry] = {}
        self.map_to_odom: Dict[int, Tuple[float, float, float]] = {}

        self.initialized = False
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.global_subs = []
        self.odom_subs = []

        for robot_id in self.robot_ids:
            global_topic = compose_topic(
                self.topic_prefix, robot_id, self.global_position_topic_name
            )
            odom_topic = compose_topic(
                self.topic_prefix, robot_id, self.odom_topic_name
            )

            self.global_subs.append(
                self.create_subscription(
                    VehicleGlobalPosition,
                    global_topic,
                    lambda msg, rid=robot_id: self._global_cb(rid, msg),
                    qos_profile_sensor_data,
                )
            )
            self.odom_subs.append(
                self.create_subscription(
                    VehicleOdometry,
                    odom_topic,
                    lambda msg, rid=robot_id: self._odom_cb(rid, msg),
                    qos_profile_sensor_data,
                )
            )

            self.get_logger().info(
                f"Robot {robot_id}: global='{global_topic}', odom='{odom_topic}'"
            )

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)
        self.get_logger().info(
            f"fleet_tf_broadcaster started with anchor_robot_id={self.anchor_robot_id}, map_frame='{self.map_frame}'"
        )

    def _global_cb(self, robot_id: int, msg: VehicleGlobalPosition) -> None:
        if self.initialized:
            return
        if not msg.lat_lon_valid:
            return
        if msg.eph > self.max_eph:
            return

        samples = self.global_samples[robot_id]
        if len(samples) < self.init_sample_count:
            samples.append((float(msg.lat), float(msg.lon), float(msg.alt)))

    def _odom_cb(self, robot_id: int, msg: VehicleOdometry) -> None:
        self.latest_odom[robot_id] = msg

        if self.initialized:
            return

        samples = self.local_pose_samples[robot_id]
        if len(samples) < self.init_sample_count:
            samples.append(
                (
                    float(msg.position[0]),
                    float(msg.position[1]),
                )
            )

    def _extract_yaw_ned(self, msg: VehicleOdometry) -> float:
        qw = float(msg.q[0])
        qx = float(msg.q[1])
        qy = float(msg.q[2])
        qz = float(msg.q[3])
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def _all_init_data_ready(self) -> bool:
        for robot_id in self.robot_ids:
            if len(self.global_samples[robot_id]) < self.init_sample_count:
                return False
            if len(self.local_pose_samples[robot_id]) < self.init_sample_count:
                return False
        return True

    def _avg_global(self, robot_id: int) -> Tuple[float, float, float]:
        samples = self.global_samples[robot_id]
        lat = sum(v[0] for v in samples) / len(samples)
        lon = sum(v[1] for v in samples) / len(samples)
        alt = sum(v[2] for v in samples) / len(samples)
        return (lat, lon, alt)

    def _avg_local_pose(self, robot_id: int) -> Tuple[float, float]:
        samples = self.local_pose_samples[robot_id]
        x = sum(v[0] for v in samples) / len(samples)
        y = sum(v[1] for v in samples) / len(samples)
        return (x, y)

    def _try_initialize_map_to_odom(self) -> None:
        if self.initialized:
            return
        if not self._all_init_data_ready():
            return

        anchor_lat, anchor_lon, anchor_alt = self._avg_global(self.anchor_robot_id)

        for robot_id in self.robot_ids:
            lat, lon, alt = self._avg_global(robot_id)
            map_x, map_y, _ = geodetic_to_enu(
                lat,
                lon,
                alt,
                anchor_lat,
                anchor_lon,
                anchor_alt,
            )

            odom_x_n, odom_y_e = self._avg_local_pose(robot_id)
            odom_x_e, odom_y_n, _ = ned_to_enu_xyz(odom_x_n, odom_y_e, 0.0)
            tx = map_x - odom_x_e
            ty = map_y - odom_y_n

            self.map_to_odom[robot_id] = (tx, ty, 0.0)

        static_transforms: List[TransformStamped] = []
        stamp = self.get_clock().now().to_msg()
        for robot_id in self.robot_ids:
            tx, ty, tyaw = self.map_to_odom[robot_id]
            qw, qx, qy, qz = yaw_to_quaternion(tyaw)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.map_frame
            tf_msg.child_frame_id = f"{self.odom_frame_prefix}{robot_id}"
            tf_msg.transform.translation.x = float(tx)
            tf_msg.transform.translation.y = float(ty)
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.w = float(qw)
            tf_msg.transform.rotation.x = float(qx)
            tf_msg.transform.rotation.y = float(qy)
            tf_msg.transform.rotation.z = float(qz)
            static_transforms.append(tf_msg)
            self.get_logger().info(
                f"Static TF {self.map_frame} -> {tf_msg.child_frame_id}: x={tx:.2f}, y={ty:.2f}, yaw={tyaw:.3f}"
            )

        self.static_tf_broadcaster.sendTransform(static_transforms)
        self.initialized = True
        self.get_logger().info(
            "Initialized static map->odom transforms from averaged GPS and odometry samples"
        )

    def _publish_dynamic_odom_to_base(self) -> None:
        if not self.initialized:
            return

        stamp = self.get_clock().now().to_msg()
        for robot_id in self.robot_ids:
            msg = self.latest_odom.get(robot_id)
            if msg is None:
                continue

            x_e, y_n, z_u = ned_to_enu_xyz(
                float(msg.position[0]),
                float(msg.position[1]),
                float(msg.position[2]),
            )

            yaw_ned = self._extract_yaw_ned(msg)
            yaw_enu = ned_yaw_to_enu_yaw(yaw_ned)
            qw, qx, qy, qz = yaw_to_quaternion(yaw_enu)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = f"{self.odom_frame_prefix}{robot_id}"
            tf_msg.child_frame_id = f"{self.base_link_frame_prefix}{robot_id}"
            tf_msg.transform.translation.x = float(x_e)
            tf_msg.transform.translation.y = float(y_n)
            tf_msg.transform.translation.z = float(z_u)
            tf_msg.transform.rotation.w = float(qw)
            tf_msg.transform.rotation.x = float(qx)
            tf_msg.transform.rotation.y = float(qy)
            tf_msg.transform.rotation.z = float(qz)
            self.tf_broadcaster.sendTransform(tf_msg)

    def _on_timer(self) -> None:
        self._try_initialize_map_to_odom()
        self._publish_dynamic_odom_to_base()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FleetTfBroadcasterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
