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

Output topic: /cmd_vel_gcs (geometry_msgs/TwistStamped) — matches the URDF
DiffDrive / TrackedVehicle plugin <topic> in rovo_gz_description.
"""

import math
import socket
import threading
from time import monotonic

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String


DEFAULT_PORT = 8887
DEFAULT_ANDROID_PORT = 8888
DEFAULT_DRIVE_DIAMETER_M = 0.35
DEFAULT_GEARBOX_RATIO = 16.0
DEFAULT_WHEEL_BASE_M = 1.05
DEFAULT_MAX_RPM = 3200
DEFAULT_RPM_MULTIPLIER = 1.0
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
        self.declare_parameter("android_port", DEFAULT_ANDROID_PORT)
        self.declare_parameter("drive_diameter_m", DEFAULT_DRIVE_DIAMETER_M)
        self.declare_parameter("gearbox_ratio", DEFAULT_GEARBOX_RATIO)
        self.declare_parameter("wheel_base_m", DEFAULT_WHEEL_BASE_M)
        self.declare_parameter("max_rpm", DEFAULT_MAX_RPM)
        self.declare_parameter("rpm_multiplier", DEFAULT_RPM_MULTIPLIER)
        self.declare_parameter("output_topic", "/cmd_vel_gcs")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("status_topic", "rovo_can_state")
        self.declare_parameter("status_rate_hz", 50.0)
        self.declare_parameter("output_frame_id", "base_link")
        self.declare_parameter("udp_timeout_s", 0.5)

        self.listen_ip = self.get_parameter("listen_ip").value
        self.listen_port = int(self.get_parameter("listen_port").value)
        self.android_port = int(self.get_parameter("android_port").value)
        self.drive_diameter_m = float(self.get_parameter("drive_diameter_m").value)
        self.gearbox_ratio = float(self.get_parameter("gearbox_ratio").value)
        self.wheel_base_m = float(self.get_parameter("wheel_base_m").value)
        self.max_rpm = int(self.get_parameter("max_rpm").value)
        self.rpm_multiplier = float(self.get_parameter("rpm_multiplier").value)
        self.output_topic = self.get_parameter("output_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.status_topic = self.get_parameter("status_topic").value
        self.status_rate_hz = float(self.get_parameter("status_rate_hz").value)
        self.output_frame_id = self.get_parameter("output_frame_id").value
        self.udp_timeout_s = float(self.get_parameter("udp_timeout_s").value)

        self.wheel_circumference = self.drive_diameter_m * math.pi

        self._lock = threading.Lock()
        self._last_twist = Twist()
        self._estop = False
        self._last_lc = -1
        self._rx_count = 0
        self._last_rx_time = 0.0
        self._stop = threading.Event()

        self.pub = self.create_publisher(TwistStamped, self.output_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self._alive_counter = 0

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.listen_ip, self.listen_port))
        self._sock.settimeout(0.2)

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self._android_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._android_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._android_sock.bind((self.listen_ip, self.android_port))
        self._android_sock.settimeout(0.2)

        self._android_rx_thread = threading.Thread(target=self._android_rx_loop, daemon=True)
        self._android_rx_thread.start()

        self.create_timer(1.0 / self.publish_rate_hz, self._publish_timer)
        self.create_timer(1.0 / self.status_rate_hz, self._status_timer)
        self.create_timer(2.0, self._diag_timer)

        self.get_logger().info(
            f"rovo_fake_pico listening on udp://{self.listen_ip}:{self.listen_port} (gcs) "
            f"and udp://{self.listen_ip}:{self.android_port} (android teleop) "
            f"-> publishing Twist on {self.output_topic} @ {self.publish_rate_hz} Hz "
            f"(rpm_multiplier={self.rpm_multiplier})"
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

    def _android_rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._android_sock.recvfrom(RECV_BUFSIZE)
            except socket.timeout:
                continue
            except OSError:
                break
            self._handle_android_packet(data.decode("utf-8", errors="replace"))

    def _handle_android_packet(self, payload: str) -> None:
        """Parse Flutter picoStringFormat: FWD/TURN in [-1000,1000] (float*1000)."""
        fields = _parse_packet(payload)
        try:
            estop = int(fields.get("ESTOP", "0")) != 0
            if estop:
                with self._lock:
                    self._last_twist = Twist()
                    self._estop = True
                    self._last_lc = int(fields.get("LC", self._last_lc))
                    self._rx_count += 1
                    self._last_rx_time = monotonic()
                return

            fwd = float(fields.get("FWD", "0"))
            turn = float(fields.get("TURN", "0"))
            # FWD/TURN are (velocity * 1000); normalise to [-1, 1]
            fwd_n = fwd / 1000.0
            turn_n = turn / 1000.0
            max_wheel_ms = (self.max_rpm / self.gearbox_ratio) * self.wheel_circumference / 60.0
            # positive turn_n → turn right → left faster, right slower
            v_left = (fwd_n + turn_n) * max_wheel_ms
            v_right = (fwd_n - turn_n) * max_wheel_ms
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
                self._last_rx_time = monotonic()
        except (ValueError, KeyError) as exc:
            self.get_logger().warning(f"bad android packet: {payload!r} ({exc})")

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
                    self._last_rx_time = monotonic()
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
                self._last_rx_time = monotonic()
        except (ValueError, KeyError) as exc:
            self.get_logger().warning(f"bad packet: {payload!r} ({exc})")

    def _publish_timer(self) -> None:
        # Skip publishing if no UDP packet ever received, or last packet is
        # stale. Avoids fighting external publishers (e.g. teleop_twist_keyboard
        # publishing directly on /cmd_vel_gcs when gcs_cmd_vel.py is not in
        # the pipeline).
        with self._lock:
            last_rx = self._last_rx_time
            lin_x = self._last_twist.linear.x
            ang_z = self._last_twist.angular.z
        if last_rx == 0.0:
            return
        if (monotonic() - last_rx) > self.udp_timeout_s:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.output_frame_id
        # rpm_multiplier MUST stay 1.0 for Nav2. fake_pico reconstructs the
        # exact (v, w) that gcs_cmd_vel encoded — same drive_diameter / gearbox
        # / wheel_base constants, so the UDP round-trip is loss-free. Any
        # multiplier != 1.0 injects open-loop gain between Nav2 and the plant:
        # the robot moves faster than commanded, overshoots every lookahead
        # point, and Nav2's closed loop oscillates (erratic motion). Sim
        # "slowness" is real-time-factor / Nav2 vel limits — fix it there, not
        # by scaling cmd_vel. Kept as a param only for deliberate experiments.
        msg.twist.linear.x = lin_x * self.rpm_multiplier
        msg.twist.angular.z = ang_z
        self.pub.publish(msg)

    def _status_timer(self) -> None:
        """Fake EC_RPDO1 feedback so gcs_cmd_vel.py FSM reaches OPERATIONAL.
        MainState=2 (Active), UnlockState=2 (Active), AliveCounter increments."""
        self._alive_counter = (self._alive_counter + 1) % 256
        msg = String()
        msg.data = f"MainState=2,UnlockState=2,AliveCounter={self._alive_counter}"
        self.status_pub.publish(msg)

    def _diag_timer(self) -> None:
        with self._lock:
            rx = self._rx_count
            last_lc = self._last_lc
            lin = self._last_twist.linear.x
            ang = self._last_twist.angular.z
            est = self._estop
        # Stay quiet while no UDP traffic (teleop-direct mode). Log only
        # when packets are actually arriving.
        if rx == 0:
            return
        self.get_logger().info(
            f"rx={rx} last_LC={last_lc} v={lin:.3f} w={ang:.3f} estop={est}"
        )

    def shutdown(self) -> None:
        self._stop.set()
        for sock in (self._sock, self._android_sock):
            try:
                sock.close()
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
