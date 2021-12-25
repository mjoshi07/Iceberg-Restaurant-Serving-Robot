#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import gazebo_msgs.msg
import tf2_ros
import tf
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander("arm")
pub = rospy.Publisher("/gazebo/set_model_state", gazebo_msgs.msg.ModelState)

def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time(0)

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise


def move_to_pickup():
    rospy.loginfo("READY TO PERFORM TASKS")
    move_group.clear_pose_targets()
    move_group.set_named_target("pickup")
    plan = move_group.plan()
    joint_waypoints = plan[1].joint_trajectory.points
    for joint in joint_waypoints:
        move_group.go(list(joint.positions))

def do_pick_and_place():
    rospy.loginfo("READY TO PERFORM TASKS")
    move_group.clear_pose_targets()
    rospy.sleep(5)
    move_group.set_named_target("drop")
    rospy.loginfo("RECIEVED TARGET POSE TO drop")
    plan = move_group.plan()
    count =0
    joint_waypoints = plan[1].joint_trajectory.points
    for joint in joint_waypoints:
        move_group.go(list(joint.positions))

def move_to_home():
    rospy.loginfo("RECIEVED TARGET POSE TO home")
    move_group.clear_pose_targets()
    move_group.set_named_target("home")
    plan = move_group.plan()
    joint_waypoints = plan[1].joint_trajectory.points
    for joint in joint_waypoints:
        move_group.go(list(joint.positions))

def get_pose_arm():

    curr_end_eff_pose = move_group.get_current_pose()
    my_pose = geometry_msgs.msg.Pose()
    my_pose.position.x = curr_end_eff_pose.pose.position.x
    my_pose.position.y = curr_end_eff_pose.pose.position.y 
    my_pose.position.z = curr_end_eff_pose.pose.position.z
    my_pose.orientation.x = 0
    my_pose.orientation.y = 0
    my_pose.orientation.z = 0
    my_pose.orientation.w = 1

    curr_end_eff_pose_top_wrist = transform_pose(my_pose, "base_link", "top_wrist")
    my_pose.position.x = curr_end_eff_pose.pose.position.x
    my_pose.position.y = curr_end_eff_pose.pose.position.y 
    my_pose.position.z = curr_end_eff_pose.pose.position.z
    my_pose.orientation.x = 0
    my_pose.orientation.y = 0
    my_pose.orientation.z = 0
    my_pose.orientation.w = 1

    return my_pose

def spawn(model_name):
    
    msg_to_pub = gazebo_msgs.msg.ModelState()
    msg_to_pub.model_name = model_name
    msg_to_pub.pose.position.x = 0
    msg_to_pub.pose.position.y = 0
    msg_to_pub.pose.position.z = 0.2
    msg_to_pub.pose.orientation.x = 0
    msg_to_pub.pose.orientation.y = 0
    msg_to_pub.pose.orientation.z = 0
    msg_to_pub.pose.orientation.w = 0
    msg_to_pub.reference_frame = 'top_wrist'
    pub.publish(msg_to_pub)