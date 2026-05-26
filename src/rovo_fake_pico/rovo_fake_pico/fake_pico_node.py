#!/usr/bin/env python3
"""rovo_fake_pico: UDP -> Twist bridge emulating the on-robot Pico controller.

Listens on UDP for the ASCII packet format produced by gcs_cmd_vel.py:

    LC=<n> FWD=<i> TURN=<i> GEAR=<i> AUTO=1 ESTOP=<0|1> MODE=<VEL|XY>
    LRPM=<i> RRPM=<i> G22=0 G26=0 G27=0 CC=0

VEL mode: LRPM / RRPM are wheel target RPMs (after gearbox). They are
converted back to wheel linear velocities via the drive diameter and
gearbox ratio, then combined into Twist(linear.x, angular.z) using
differential-drive kinematics with the configured wheel base.

XY mode: FWD/TURN are scaled integers in [-800, 800] / [-500, 500]
matching gcs_cmd_vel.py's encoding; converted back to Twist by undoing
the same scaling.

Output topic: /cmd_vel_gcs (geometry_msgs/Twist) — matches the URDF
DiffDrive / TrackedVehicle plugin <topic> in rovo_gz_description.
"""

import math
import socket
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


DEFAULT_PORT = 8887
DEFAULT_DRIVE_DIAMETER_M = 0.35
DEFAULT_GEARBOX_RATIO = 16.0
DEFAULT_WHEEL_BASE_M = 1.05
DEFAULT_MAX_RPM = 3200
RECV_BUFSIZE = 1024


def _parse_packet(payload: str) -> dict:
    out = {}
    for token in payload.strip().split():
        if "=" not in token:
            continue
        k, v = token.split("=", 1)
        out[k] = v
    return out


class FakePicoNode(Node):

    def __init__(self):
        super().__init__("rovo_fake_pico")

        self.declare_parameter("listen_ip", "0.0.0.0")
        self.declare_parameter("listen_port", DEFAULT_PORT)
        self.declare_parameter("drive_diameter_m", DEFAULT_DRIVE_DIAMETER_M)
        self.declare_parameter("gearbox_ratio", DEFAULT_GEARBOX_RATIO)
        self.declare_parameter("wheel_base_m", DEFAULT_WHEEL_BASE_M)
        self.declare_parameter("max_rpm", DEFAULT_MAX_RPM)
        self.declare_parameter("output_topic", "/cmd_vel_gcs")
        self.declare_parameter("publish_rate_hz", 100.0)

        self.listen_ip = self.get_parameter("listen_ip").value
        self.listen_port = int(self.get_parameter("listen_port").value)
        self.drive_diameter_m = float(self.get_parameter("drive_diameter_m").value)
        self.gearbox_ratio = float(self.get_parameter("gearbox_ratio").value)
        self.wheel_base_m = float(self.get_parameter("wheel_base_m").value)
        self.max_rpm = int(self.get_parameter("max_rpm").value)
        self.output_topic = self.get_parameter("output_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.wheel_circumference = self.drive_diameter_m * math.pi

        self._lock = threading.Lock()
        self._last_twist = Twist()
        self._estop = False
        self._last_lc = -1
        self._rx_count = 0
        self._stop = threading.Event()

        self.pub = self.create_publisher(Twist, self.output_topic, 10)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.listen_ip, self.listen_port))
        self._sock.settimeout(0.2)

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.create_timer(1.0 / self.publish_rate_hz, self._publish_timer)
        self.create_timer(2.0, self._diag_timer)

        self.get_logger().info(
            f"rovo_fake_pico listening on udp://{self.listen_ip}:{self.listen_port} "
            f"-> publishing Twist on {self.output_topic} @ {self.publish_rate_hz} Hz"
        )

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(RECV_BUFSIZE)
            except socket.timeout:
                continue
            except OSError:
                break
            self._handle_packet(data.decode("utf-8", errors="replace"))

    def _handle_packet(self, payload: str) -> None:
        fields = _parse_packet(payload)
        try:
            mode = fields.get("MODE", "VEL").upper()
            estop = int(fields.get("ESTOP", "0")) != 0
            if estop:
                with self._lock:
                    self._last_twist = Twist()
                    self._estop = True
                    self._last_lc = int(fields.get("LC", self._last_lc))
                    self._rx_count += 1
                return

            if mode == "VEL":
                lrpm = float(fields.get("LRPM", "0"))
                rrpm = float(fields.get("RRPM", "0"))
                v_left = (lrpm / self.gearbox_ratio) * self.wheel_circumference / 60.0
                v_right = (rrpm / self.gearbox_ratio) * self.wheel_circumference / 60.0
                linear_x = (v_left + v_right) / 2.0
                angular_z = (v_right - v_left) / self.wheel_base_m
            else:
                fwd = float(fields.get("FWD", "0"))
                turn = float(fields.get("TURN", "0"))
                avg_rpm = fwd * (self.max_rpm / 800.0)
                diff_rpm = turn * (self.max_rpm / 500.0)
                lrpm = avg_rpm + diff_rpm
                rrpm = avg_rpm - diff_rpm
                v_left = (lrpm / self.gearbox_ratio) * self.wheel_circumference / 60.0
                v_right = (rrpm / self.gearbox_ratio) * self.wheel_circumference / 60.0
                linear_x = (v_left + v_right) / 2.0
                angular_z = (v_right - v_left) / self.wheel_base_m

            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z

            with self._lock:
                self._last_twist = twist
                self._estop = False
                self._last_lc = int(fields.get("LC", self._last_lc))
                self._rx_count += 1
        except (ValueError, KeyError) as exc:
            self.get_logger().warning(f"bad packet: {payload!r} ({exc})")

    def _publish_timer(self) -> None:
        with self._lock:
            twist = Twist()
            twist.linear.x = self._last_twist.linear.x
            twist.angular.z = self._last_twist.angular.z
        self.pub.publish(twist)

    def _diag_timer(self) -> None:
        with self._lock:
            self.get_logger().info(
                f"rx={self._rx_count} last_LC={self._last_lc} "
                f"v={self._last_twist.linear.x:.3f} w={self._last_twist.angular.z:.3f} "
                f"estop={self._estop}"
            )

    def shutdown(self) -> None:
        self._stop.set()
        try:
            self._sock.close()
        except OSError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = FakePicoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
