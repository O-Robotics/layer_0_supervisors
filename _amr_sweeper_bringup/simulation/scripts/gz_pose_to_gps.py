#!/usr/bin/env python3

import math
import random

import rclpy
from gps_msgs.msg import GPSFix
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker

ORIGIN_LAT = 43.2557
ORIGIN_LON = -79.8711
ORIGIN_ALT = 100.0
A = 6378137.0
E2 = 0.00669437999014


def enu_to_lla(x, y, z):
    lat0 = math.radians(ORIGIN_LAT)
    lon0 = math.radians(ORIGIN_LON)
    alt0 = ORIGIN_ALT
    sl = math.sin(lat0)
    cl = math.cos(lat0)
    sn = math.sin(lon0)
    cn = math.cos(lon0)
    n0 = A / math.sqrt(1.0 - E2 * sl * sl)
    x0 = (n0 + alt0) * cl * cn
    y0 = (n0 + alt0) * cl * sn
    z0 = (n0 * (1 - E2) + alt0) * sl
    dx = -sn * x - sl * cn * y + cl * cn * z
    dy = cn * x - sl * sn * y + cl * sn * z
    dz = cl * y + sl * z
    xp = x0 + dx
    yp = y0 + dy
    zp = z0 + dz
    p = math.sqrt(xp * xp + yp * yp)
    lat = math.atan2(zp, p * (1.0 - E2))
    for _ in range(5):
        s = math.sin(lat)
        n = A / math.sqrt(1.0 - E2 * s * s)
        lat = math.atan2(zp + E2 * n * s, p)
    s = math.sin(lat)
    n = A / math.sqrt(1.0 - E2 * s * s)
    alt = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(math.atan2(yp, xp)), alt


def _is_base_link(frame_id):
    tail = frame_id.rsplit("::", 1)[-1].rsplit("/", 1)[-1]
    return tail == "base_link"


class GzPoseToGps(Node):
    def __init__(self):
        super().__init__("gz_pose_to_gps")

        self.declare_parameter("world_name", "amr_sweeper_test")
        self.declare_parameter("noise_h", 0.5)
        self.declare_parameter("noise_v", 0.3)
        self.declare_parameter("spike_at_s", -1.0)
        self.declare_parameter("spike_duration_s", 8.0)
        self.declare_parameter("spike_dx_m", 60.0)
        self.declare_parameter("spike_dy_m", 0.0)
        self.declare_parameter("outage_at_s", -1.0)
        self.declare_parameter("outage_duration_s", 25.0)
        self.declare_parameter("spike2_at_s", -1.0)
        self.declare_parameter("spike2_duration_s", 6.0)
        self.declare_parameter("spike2_dx_m", -60.0)
        self.declare_parameter("spike2_dy_m", 0.0)
        self.declare_parameter("navsat_topic", "/amr_sweeper/gnss/navsat")
        self.declare_parameter("gpsfix_topic", "/amr_sweeper/gnss/fix")
        self.declare_parameter("odometry_topic", "/amr_sweeper/gnss/odometry")
        self.declare_parameter("status_marker_topic", "/amr_sweeper/gnss/status_marker")
        self.declare_parameter("frame_id", "gnss_link")

        world = self.get_parameter("world_name").get_parameter_value().string_value
        navsat_topic = self.get_parameter("navsat_topic").get_parameter_value().string_value
        gpsfix_topic = self.get_parameter("gpsfix_topic").get_parameter_value().string_value
        odometry_topic = self.get_parameter("odometry_topic").get_parameter_value().string_value
        status_marker_topic = self.get_parameter("status_marker_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self.pub_navsat = self.create_publisher(NavSatFix, navsat_topic, 10)
        self.pub_fix = self.create_publisher(GPSFix, gpsfix_topic, 10)
        self.pub_odom = self.create_publisher(Odometry, odometry_topic, 10)
        self.pub_marker = self.create_publisher(Marker, status_marker_topic, 10)
        self.sub = self.create_subscription(
            TFMessage, f"/world/{world}/pose/info", self.pose_cb, 10)

        self.body_frame_id = None
        self.ref_published = False
        self.start_ns = None
        self._last_status = ""

        self.get_logger().info(
            f"GPS publisher ready (world={world}, navsat={navsat_topic}, gpsfix={gpsfix_topic})")

    def _elapsed(self):
        if self.start_ns is None:
            return 0.0
        return (self.get_clock().now().nanoseconds - self.start_ns) * 1e-9

    def _window(self, at_param, dur_param):
        at = self.get_parameter(at_param).get_parameter_value().double_value
        if at < 0.0:
            return False
        dur = self.get_parameter(dur_param).get_parameter_value().double_value
        elapsed = self._elapsed()
        return at <= elapsed < (at + dur)

    def _outage_active(self):
        return self._window("outage_at_s", "outage_duration_s")

    def _spike_active(self):
        return self._window("spike_at_s", "spike_duration_s")

    def _spike2_active(self):
        return self._window("spike2_at_s", "spike2_duration_s")

    def _find_body(self, msg):
        if self.body_frame_id is not None:
            for tf in msg.transforms:
                if tf.child_frame_id == self.body_frame_id:
                    translation = tf.transform.translation
                    if 0.05 < translation.z < 0.8:
                        return translation
        for tf in msg.transforms:
            if _is_base_link(tf.child_frame_id):
                translation = tf.transform.translation
                if 0.05 < translation.z < 0.8:
                    self.body_frame_id = tf.child_frame_id
                    return translation
        best = None
        best_mag = -1.0
        best_frame_id = None
        for tf in msg.transforms:
            translation = tf.transform.translation
            if not (0.05 < translation.z < 0.8):
                continue
            magnitude = translation.x * translation.x + translation.y * translation.y
            if magnitude > best_mag:
                best_mag = magnitude
                best = translation
                best_frame_id = tf.child_frame_id
        if best_frame_id is not None:
            self.body_frame_id = best_frame_id
        return best

    def _publish_status(self, label: str, red: float, green: float, blue: float):
        if label == self._last_status:
            return
        self._last_status = label
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "odom"
        marker.ns = "gps_status"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = -18.0
        marker.pose.position.y = 28.0
        marker.pose.position.z = 3.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 2.5
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        marker.text = label
        self.pub_marker.publish(marker)

    def pose_cb(self, msg):
        best = self._find_body(msg)
        if best is None:
            return

        if self.start_ns is None:
            self.start_ns = self.get_clock().now().nanoseconds

        noise_h = self.get_parameter("noise_h").get_parameter_value().double_value
        noise_v = self.get_parameter("noise_v").get_parameter_value().double_value

        outage = self._outage_active()
        spike1 = self._spike_active()
        spike2 = self._spike2_active()

        if outage:
            self._publish_status("  GPS OUTAGE  ", 0.95, 0.45, 0.0)
            if not hasattr(self, "_outage_logged"):
                self.get_logger().warn("GPS OUTAGE ACTIVE: no fixes being published")
                self._outage_logged = True
            return
        if hasattr(self, "_outage_logged"):
            del self._outage_logged
            self.get_logger().info("GPS outage ended, fixes resuming")

        if spike1:
            dx = self.get_parameter("spike_dx_m").get_parameter_value().double_value
            dy = self.get_parameter("spike_dy_m").get_parameter_value().double_value
            self._publish_status("  GPS SPIKE +60 m  ", 0.95, 0.1, 0.1)
            if not hasattr(self, "_spike1_logged"):
                self.get_logger().warn(f"GPS SPIKE 1 ACTIVE: +{dx:.0f} m East")
                self._spike1_logged = True
        elif hasattr(self, "_spike1_logged"):
            del self._spike1_logged
            self.get_logger().info("GPS spike 1 ended")
            dx, dy = 0.0, 0.0
        elif spike2:
            dx = self.get_parameter("spike2_dx_m").get_parameter_value().double_value
            dy = self.get_parameter("spike2_dy_m").get_parameter_value().double_value
            self._publish_status("  GPS SPIKE -60 m  ", 0.95, 0.1, 0.1)
            if not hasattr(self, "_spike2_logged"):
                self.get_logger().warn(f"GPS SPIKE 2 ACTIVE: {dx:.0f} m East")
                self._spike2_logged = True
        elif hasattr(self, "_spike2_logged"):
            del self._spike2_logged
            self.get_logger().info("GPS spike 2 ended")
            dx, dy = 0.0, 0.0
        else:
            self._publish_status("  GPS OK  ", 0.1, 0.9, 0.1)
            dx, dy = 0.0, 0.0

        x = best.x + random.gauss(0, noise_h) + dx
        y = best.y + random.gauss(0, noise_h) + dy
        z = best.z if not self.ref_published else best.z + random.gauss(0, noise_v)

        now = self.get_clock().now().to_msg()
        lat, lon, alt = enu_to_lla(x, y, z)
        covariance = [
            noise_h ** 2, 0.0, 0.0,
            0.0, noise_h ** 2, 0.0,
            0.0, 0.0, noise_v ** 2,
        ]

        navsat = NavSatFix()
        navsat.header.stamp = now
        navsat.header.frame_id = self.frame_id
        navsat.status.status = NavSatStatus.STATUS_FIX
        navsat.status.service = NavSatStatus.SERVICE_GPS
        navsat.latitude = lat
        navsat.longitude = lon
        navsat.altitude = alt
        navsat.position_covariance = covariance
        navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_navsat.publish(navsat)

        gpsfix = GPSFix()
        gpsfix.header.stamp = now
        gpsfix.header.frame_id = self.frame_id
        gpsfix.status.header.stamp = now
        gpsfix.status.header.frame_id = self.frame_id
        gpsfix.status.status = 0
        gpsfix.status.motion_source = 1
        gpsfix.status.position_source = 1
        gpsfix.status.satellites_used = 10
        gpsfix.status.satellites_visible = 12
        gpsfix.latitude = lat
        gpsfix.longitude = lon
        gpsfix.altitude = alt
        gpsfix.hdop = max(0.7, noise_h / 0.5)
        gpsfix.vdop = max(0.9, noise_v / 0.3)
        gpsfix.err = max(noise_h, noise_v) * 2.0
        gpsfix.err_horz = noise_h * 2.0
        gpsfix.err_vert = noise_v * 2.0
        gpsfix.position_covariance = covariance
        gpsfix.position_covariance_type = GPSFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_fix.publish(gpsfix)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        variance = noise_h ** 2
        odom.pose.covariance[0] = variance
        odom.pose.covariance[7] = variance
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 1e6
        self.pub_odom.publish(odom)

        self.ref_published = True


def main():
    rclpy.init()
    rclpy.spin(GzPoseToGps())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
