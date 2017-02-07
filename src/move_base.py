#!/usr/bin/env python

import rospy
import tf
import roslib
# Brings in the SimpleActionClient
import actionlib
from move_base_msgs.msg import *
from math import radians
# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
class MoveBase:

    def __init__(self):
        pass

    def simple_move(self,center_distance, table_width): #in meters


        publisher = rospy.Publisher("/move_base_navi/goal", MoveBaseActionGoal,queue_size=10)
        rospy.sleep(1)
        action_goal = MoveBaseActionGoal()
        action_goal.header.frame_id = 'odom'
        # Waits until the action server has started up and started
        # listening for goals.

        action_goal.goal.target_pose.header.frame_id = 'base_link'
        action_goal.goal.target_pose.pose.position.y = -table_width
        action_goal.goal.target_pose.pose.orientation.w = 1.0
        action_goal.goal.target_pose.header.stamp = rospy.Time.now()
        publisher.publish(action_goal)
        rospy.sleep(10)
        action_goal.goal.target_pose.pose.position.x = center_distance*2
        action_goal.goal.target_pose.pose.position.y = 0.0
        action_goal.goal.target_pose.pose.orientation.w = 1.0
        action_goal.goal.target_pose.header.stamp = rospy.Time.now()
        publisher.publish(action_goal)
        rospy.sleep(10)
        action_goal.goal.target_pose.pose.position.x = 0.0
        action_goal.goal.target_pose.pose.position.y = table_width
        action_goal.goal.target_pose.pose.orientation.w = 1.0
        action_goal.goal.target_pose.header.stamp = rospy.Time.now()
        publisher.publish(action_goal)
        rospy.sleep(10)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, radians(-180))
        pose= geometry_msgs.msg.Pose()
        action_goal.goal.target_pose.pose.position.x = 0.0
        action_goal.goal.target_pose.pose.position.y = 0.0
        action_goal.goal.target_pose.pose.orientation.x = quaternion[0]
        action_goal.goal.target_pose.pose.orientation.y = quaternion[1]
        action_goal.goal.target_pose.pose.orientation.z = quaternion[2]
        action_goal.goal.target_pose.pose.orientation.w = quaternion[3]
        action_goal.goal.target_pose.header.stamp = rospy.Time.now()
        publisher.publish(action_goal)
        print "published"

if __name__ == '__main__':
        try:
            # Initializes a rospy node so that the SimpleActionClient can
            # publish and subscribe over ROS.
            move_base = MoveBase()
            move_base.simple_move()
        except rospy.ROSInterruptException:
            print "failed"