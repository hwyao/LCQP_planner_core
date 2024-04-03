#!/usr/bin/env python

import rospy
import tf.transformations

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, \
    InteractiveMarkerControl, Marker
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

marker_pose = PoseStamped()
pose_pub = None
# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.9, 0.9], [-0.6, 0.6], [0.05, 0.9]]


def publisher_callback(msg, link_name):
    marker_pose.header.frame_id = link_name
    marker_pose.header.stamp = rospy.Time(0)
    pose_pub.publish(marker_pose)


def process_feedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        marker_pose.pose.position.x = max([min([feedback.pose.position.x,
                                          position_limits[0][1]]),
                                          position_limits[0][0]])
        marker_pose.pose.position.y = max([min([feedback.pose.position.y,
                                          position_limits[1][1]]),
                                          position_limits[1][0]])
        marker_pose.pose.position.z = max([min([feedback.pose.position.z,
                                          position_limits[2][1]]),
                                          position_limits[2][0]])
        marker_pose.pose.orientation = feedback.pose.orientation
    server.applyChanges()


def wait_for_initial_pose():
    marker_pose.pose.orientation.x = 0
    marker_pose.pose.orientation.y = 0
    marker_pose.pose.orientation.z = 0
    marker_pose.pose.orientation.w = 1
    value = [0.29, 0.27, 0.54]
    marker_pose.pose.position.x = value[0]
    marker_pose.pose.position.y = value[1]
    marker_pose.pose.position.z = value[2]


if __name__ == "__main__":
    rospy.init_node("obstacle_pose_node")
    listener = tf.TransformListener()
    link_name = rospy.get_param("~link_name")

    wait_for_initial_pose()

    pose_pub = rospy.Publisher("obstacle_pose", PoseStamped, queue_size=10)
    server = InteractiveMarkerServer("obstacle_pose_marker")
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = link_name
    int_marker.scale = 0.3
    int_marker.name = "obstacle_pose"
    int_marker.pose = marker_pose.pose
    
    box_marker = Marker()
    box_marker.type = Marker.SPHERE
    box_marker.scale.x = 0.2
    box_marker.scale.y = 0.2
    box_marker.scale.z = 0.2
    box_marker.color.r = 0
    box_marker.color.g = 0
    box_marker.color.b = 1
    box_marker.color.a = 1

    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append( box_marker )
    int_marker.controls.append( box_control )


    # run pose publisher
    rospy.Timer(rospy.Duration(0.005),
                lambda msg: publisher_callback(msg, link_name))

    # insert a box
    # control = InteractiveMarkerControl()
    # control.orientation.w = 1
    # control.orientation.x = 1
    # control.orientation.y = 0
    # control.orientation.z = 0
    # control.name = "rotate_x"
    # control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    # int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    # control = InteractiveMarkerControl()
    # control.orientation.w = 1
    # control.orientation.x = 0
    # control.orientation.y = 1
    # control.orientation.z = 0
    # control.name = "rotate_y"
    # control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    # int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    # control = InteractiveMarkerControl()
    # control.orientation.w = 1
    # control.orientation.x = 0
    # control.orientation.y = 0
    # control.orientation.z = 1
    # control.name = "rotate_z"
    # control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    # int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    server.insert(int_marker, process_feedback)
    server.applyChanges()

    rospy.spin()
