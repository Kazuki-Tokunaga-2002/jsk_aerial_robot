#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker


def make_ceiling_marker(frame_id, height, size_x, size_y, thickness, alpha):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "delta_ceiling"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = rospy.get_param("~center_x", 0.0)
    marker.pose.position.y = rospy.get_param("~center_y", 0.0)
    marker.pose.position.z = height
    marker.pose.orientation.w = 1.0
    marker.scale.x = size_x
    marker.scale.y = size_y
    marker.scale.z = thickness
    marker.color.r = 0.25
    marker.color.g = 0.85
    marker.color.b = 1.0
    marker.color.a = alpha
    return marker


def main():
    rospy.init_node("delta_ceiling_marker_publisher")

    frame_id = rospy.get_param("~frame_id", "world")
    topic = rospy.get_param("~topic", "/delta/ceiling_marker")
    height = rospy.get_param("~ceiling_height", 2.73)
    size_x = rospy.get_param("~size_x", 5.0)
    size_y = rospy.get_param("~size_y", 5.0)
    thickness = rospy.get_param("~thickness", 0.01)
    alpha = rospy.get_param("~alpha", 0.25)
    rate_hz = max(0.1, rospy.get_param("~rate", 1.0))

    pub = rospy.Publisher(topic, Marker, queue_size=1, latch=True)
    rate = rospy.Rate(rate_hz)

    rospy.loginfo("Publishing ceiling marker on %s at z=%.3f in frame %s", topic, height, frame_id)
    while not rospy.is_shutdown():
        pub.publish(make_ceiling_marker(frame_id, height, size_x, size_y, thickness, alpha))
        rate.sleep()


if __name__ == "__main__":
    main()
