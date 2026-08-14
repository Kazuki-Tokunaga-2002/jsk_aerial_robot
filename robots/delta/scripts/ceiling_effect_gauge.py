#!/usr/bin/env python3

import math
import threading

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class CeilingEffectGauge:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.input_namespace = rospy.get_param("~input_namespace", "/delta/debug/ceiling_effect").rstrip("/")
        self.distance_topic = rospy.get_param("~distance_topic", self.input_namespace + "/d_i")
        self.dbar_topic = rospy.get_param("~dbar_topic", self.input_namespace + "/dbar_i")
        self.theta_topic = rospy.get_param("~theta_topic", self.input_namespace + "/theta_i")
        self.ratio_topic = rospy.get_param("~ratio_topic", self.input_namespace + "/k_i")
        self.scale_topic = rospy.get_param("~scale_topic", self.input_namespace + "/1_k_i")
        self.marker_topic = rospy.get_param("~marker_topic", "/delta/ceiling_effect_gauge")
        self.ceiling_height = rospy.get_param("~ceiling_height", 2.73)
        self.origin_x = rospy.get_param("~origin_x", -1.0)
        self.origin_y = rospy.get_param("~origin_y", 1.7)
        self.origin_z = rospy.get_param("~origin_z", 1.2)
        self.spacing = rospy.get_param("~spacing", 0.55)
        self.radius = rospy.get_param("~radius", 0.18)
        self.text_size = rospy.get_param("~text_size", 0.065)
        self.theta_max = rospy.get_param("~theta_max", 0.6981317008)
        self.k_max = rospy.get_param("~k_max", 1.3)
        self.rate_hz = max(0.1, rospy.get_param("~rate", 10.0))

        self.latest = {
            "distance": None,
            "dbar": None,
            "theta": None,
            "ratio": None,
            "scale": None,
        }
        self.lock = threading.Lock()

        self.pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)
        self.subs = [
            rospy.Subscriber(self.distance_topic, Float32MultiArray, self.make_callback("distance"), queue_size=1),
            rospy.Subscriber(self.dbar_topic, Float32MultiArray, self.make_callback("dbar"), queue_size=1),
            rospy.Subscriber(self.theta_topic, Float32MultiArray, self.make_callback("theta"), queue_size=1),
            rospy.Subscriber(self.ratio_topic, Float32MultiArray, self.make_callback("ratio"), queue_size=1),
            rospy.Subscriber(self.scale_topic, Float32MultiArray, self.make_callback("scale"), queue_size=1),
        ]
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.publish)

        rospy.loginfo("Publishing ceiling effect gauges on %s from %s/*", self.marker_topic, self.input_namespace)

    def make_callback(self, field):
        def callback(msg):
            with self.lock:
                self.latest[field] = list(msg.data)
        return callback

    def publish(self, _event):
        with self.lock:
            data = {key: None if value is None else list(value) for key, value in self.latest.items()}

        marker_array = MarkerArray()
        marker_array.markers.append(self.delete_all_marker())

        if any(value is None for value in data.values()):
            marker_array.markers.append(self.text_marker(1, self.origin_x, self.origin_y, self.origin_z,
                                                         "waiting for\nceiling_effect", 0.08))
            self.pub.publish(marker_array)
            return

        rotor_count = min(len(value) for value in data.values())
        if rotor_count == 0:
            return

        marker_id = 1
        for i in range(rotor_count):
            distance = data["distance"][i]
            dbar = data["dbar"][i]
            theta = data["theta"][i]
            k = data["ratio"][i]
            scale = data["scale"][i]

            cx = self.origin_x + i * self.spacing
            cy = self.origin_y
            cz = self.origin_z

            d_ratio = self.normalize(distance, 0.0, self.ceiling_height)
            theta_ratio = self.normalize(theta, 0.0, self.theta_max)
            k_ratio = self.normalize(k, 1.0, self.k_max)

            marker_array.markers.append(self.ring_marker(marker_id, cx, cy, cz, self.radius, (0.35, 0.35, 0.35, 0.45)))
            marker_id += 1
            marker_array.markers.append(self.arc_marker(marker_id, cx, cy, cz, self.radius, d_ratio,
                                                        (0.20, 0.70, 1.00, 0.95)))
            marker_id += 1
            marker_array.markers.append(self.arc_marker(marker_id, cx, cy, cz, self.radius * 0.75, theta_ratio,
                                                        (1.00, 0.70, 0.20, 0.95)))
            marker_id += 1
            marker_array.markers.append(self.arc_marker(marker_id, cx, cy, cz, self.radius * 0.50, k_ratio,
                                                        (0.35, 1.00, 0.45, 0.95)))
            marker_id += 1

            text = "rotor{}\nd={:.2f}m\nd/R={:.2f}\ntheta={:.1f}deg\nk={:.3f}\n1/k={:.3f}".format(
                i + 1, distance, dbar, math.degrees(theta), k, scale)
            marker_array.markers.append(self.text_marker(marker_id, cx, cy - self.radius * 1.65, cz, text,
                                                         self.text_size))
            marker_id += 1

        self.pub.publish(marker_array)

    def delete_all_marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.DELETEALL
        return marker

    def ring_marker(self, marker_id, cx, cy, cz, radius, color):
        marker = self.base_line_marker(marker_id, "ceiling_effect_ring", color)
        marker.points = self.circle_points(cx, cy, cz, radius, 0.0, 2.0 * math.pi, 96)
        return marker

    def arc_marker(self, marker_id, cx, cy, cz, radius, ratio, color):
        marker = self.base_line_marker(marker_id, "ceiling_effect_arc", color)
        end_angle = -0.5 * math.pi + 2.0 * math.pi * ratio
        marker.points = self.circle_points(cx, cy, cz, radius, -0.5 * math.pi, end_angle, 64)
        return marker

    def text_marker(self, marker_id, x, y, z, text, size):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "ceiling_effect_text"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.z = size
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = text
        return marker

    def base_line_marker(self, marker_id, namespace, color):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.012
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        return marker

    @staticmethod
    def circle_points(cx, cy, cz, radius, start_angle, end_angle, segments):
        points = []
        if end_angle < start_angle:
            end_angle = start_angle
        for i in range(segments + 1):
            ratio = float(i) / float(segments)
            angle = start_angle + (end_angle - start_angle) * ratio
            point = Point()
            point.x = cx + radius * math.cos(angle)
            point.y = cy + radius * math.sin(angle)
            point.z = cz
            points.append(point)
        return points

    @staticmethod
    def normalize(value, min_value, max_value):
        if max_value <= min_value:
            return 0.0
        return max(0.0, min(1.0, (value - min_value) / (max_value - min_value)))


def main():
    rospy.init_node("delta_ceiling_effect_gauge")
    CeilingEffectGauge()
    rospy.spin()


if __name__ == "__main__":
    main()
