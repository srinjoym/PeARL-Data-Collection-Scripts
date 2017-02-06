#!/usr/bin/env python

import rospy
import roslib
# Brings in the SimpleActionClient
import actionlib
from move_base_msgs.msg import *

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.

def simple_move():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.

    rospy.init_node('simple_move')

    # client = actionlib.SimpleActionClient("move_base_navi", MoveBaseAction)

    publisher = rospy.Publisher("/move_base_navi/goal", MoveBaseActionGoal,queue_size=10)
    rospy.sleep(1)
    action_goal = MoveBaseActionGoal()
    action_goal.header.frame_id = 'odom'
    # Waits until the action server has started up and started
    # listening for goals.

    action_goal.goal.target_pose.header.frame_id = 'base_link'
    action_goal.goal.target_pose.pose.position.y = -1.0
    action_goal.goal.target_pose.pose.orientation.w = 1.0
    action_goal.goal.target_pose.header.stamp = rospy.Time.now()
    publisher.publish(action_goal)
    rospy.sleep(15)
    action_goal.goal.target_pose.pose.position.x = 1.0
    action_goal.goal.target_pose.pose.position.y = 0.0
    action_goal.goal.target_pose.pose.orientation.w = 1.0
    action_goal.goal.target_pose.header.stamp = rospy.Time.now()
    publisher.publish(action_goal)
    rospy.sleep(15)
    action_goal.goal.target_pose.pose.position.x = 0.0
    action_goal.goal.target_pose.pose.position.y = 1.0
    action_goal.goal.target_pose.pose.orientation.w = 1.0
    action_goal.goal.target_pose.header.stamp = rospy.Time.now()
    publisher.publish(action_goal)
    print "published"

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        simple_move()
    except rospy.ROSInterruptException:
        print "failed"