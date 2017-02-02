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

    client = actionlib.SimpleActionClient("move_base_navi", MoveBaseAction)

    # publisher = rospy.Publisher('move_base_navi/goal', MoveBaseActionGoal)

    goal = MoveBaseGoal()
    # Waits until the action server has started up and started
    # listening for goals.

    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.pose.position.x = 1.0
    goal.target_pose.pose.orientation.w = 1.0
    goal.target_pose.header.stamp = rospy.Time.now()
    print "created goal"
    
    client.wait_for_server(rospy.Duration(5))
    print "finished waiting for server"
    # Sends the goal to the action server.
    client.send_goal(goal)
    print "sent goal"
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    print "finished waiing for result"
    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        simple_move()
    except rospy.ROSInterruptException:
        print "failed"