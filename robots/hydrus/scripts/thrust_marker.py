#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from spinal.msg import FourAxisCommand
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class ThrustMarkerNode:
    def __init__(self):
        # Arrow length: [m/N]
        self.scale = rospy.get_param("~scale", 0.05)

        # Color mapping range [N]
        self.thrust_min = rospy.get_param("~thrust_min", 4.0)
        self.thrust_max = rospy.get_param("~thrust_max", 12.0)

        self.frame_prefix = rospy.get_param(
            "~frame_prefix",
            "hydrus/thrust"
        )

        self.marker_topic = rospy.get_param(
            "~marker_topic",
            "thrust_markers"
        )

        # Initial k_i values
        self.thrust_ratio = [1.0, 1.0, 1.0, 1.0]

        self.marker_pub = rospy.Publisher(
            self.marker_topic,
            MarkerArray,
            queue_size=1
        )

        self.command_sub = rospy.Subscriber(
            "/hydrus/four_axes/command",
            FourAxisCommand,
            self.command_callback,
            queue_size=1
        )

        self.thrust_ratio_sub = rospy.Subscriber(
            "/hydrus/ceiling_effect/thrust_ratio",
            Float32MultiArray,
            self.thrust_ratio_callback,
            queue_size=1
        )

    def thrust_ratio_callback(self, msg):
        if len(msg.data) < 4:
            rospy.logwarn_throttle(
                1.0,
                "thrust_ratio has fewer than 4 elements."
            )
            return

        self.thrust_ratio = list(msg.data[:4])

    def command_callback(self, msg):
        if len(msg.base_thrust) < 4:
            rospy.logwarn_throttle(
                1.0,
                "base_thrust has fewer than 4 elements."
            )
            return

        marker_array = MarkerArray()
        stamp = rospy.Time.now()

        for i in range(4):
            # Displayed thrust:
            # T_i = k_i * lambda_i
            thrust_for_visualization = (
                msg.base_thrust[i] * self.thrust_ratio[i]
            )

            # Normalize thrust for color mapping
            if self.thrust_max > self.thrust_min:
                ratio = (
                    (thrust_for_visualization - self.thrust_min)
                    / (self.thrust_max - self.thrust_min)
                )
            else:
                ratio = 0.0

            ratio = max(0.0, min(1.0, ratio))

            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = "{}{}".format(
                self.frame_prefix,
                i + 1
            )

            marker.ns = "rotor_thrust"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            # In the local thrust frame, +z is the thrust direction.
            start = Point(0.0, 0.0, 0.0)
            end = Point(
                0.0,
                0.0,
                self.scale * thrust_for_visualization
            )
            marker.points = [start, end]

            # Arrow thickness
            marker.scale.x = 0.025
            marker.scale.y = 0.050
            marker.scale.z = 0.080

            # Small thrust: light blue
            # Large thrust: dark blue
            # Small thrust: blue
            # Large thrust: red
            marker.color.r = ratio
            marker.color.g = 0.15
            marker.color.b = 1.0 - ratio
            marker.color.a = 0.95

            marker.lifetime = rospy.Duration(0.2)

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


if __name__ == "__main__":
    rospy.init_node("thrust_marker")
    ThrustMarkerNode()
    rospy.spin()