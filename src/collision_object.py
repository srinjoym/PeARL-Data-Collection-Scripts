#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from moveit_msgs.msg import *
import moveit_msgs.srv
from geometry_msgs.msg import *
import std_msgs.msg
import wpi_jaco_msgs.msg
import wpi_jaco_msgs.srv
import time
import operator
from vector_msgs.msg import JacoCartesianVelocityCmd, LinearActuatorCmd, GripperCmd, GripperStat
import requests
import tf2_ros
import tf2_geometry_msgs
import tf
from shape_msgs.msg import *
from ar_track_alvar_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from std_msgs.msg import *
from math import pi, floor, ceil, fabs, sin, cos, radians,degrees
import numpy

def publish_collision_object(scene,goal_position):

    
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = "base_link"
    # table_pose.pose.position.x = 0.6+0.8;
    table_pose.pose.position.x = 1.25+.3; #distance + half width CHANGE THIS TO MATCH HOW FAR TABLE IS
    table_pose.pose.position.y = 0;
    table_pose.pose.position.z = .4; # half of height
    table_pose.pose.orientation.x = 0;
    table_pose.pose.orientation.y = 0;
    table_pose.pose.orientation.z = 0;
    table_pose.pose.orientation.w = 1;

    table_scale = [0.6,1.23,.8] # check if height is too low
    
    leg_pose = geometry_msgs.msg.PoseStamped()
    leg_pose.header.frame_id = "left_ee_link"
    leg_pose.pose.position.x = 0
    leg_pose.pose.position.y = 0
    leg_pose.pose.position.z = 0.1
    leg_pose.pose.orientation.x = 0;
    leg_pose.pose.orientation.y = 0;
    leg_pose.pose.orientation.z = 0;
    leg_pose.pose.orientation.w = 1;

    leg_scale = [0.05,0.05,0.3]


    floor_pose =  geometry_msgs.msg.PoseStamped()
    floor_pose.header.frame_id = "base_link"
    floor_pose.pose.position.x = 0
    floor_pose.pose.position.y = 0
    floor_pose.pose.position.z = 0.1
    floor_pose.pose.orientation.x = 0;
    floor_pose.pose.orientation.y = 0;
    floor_pose.pose.orientation.z = 0;
    floor_pose.pose.orientation.w = 1;

    floor_scale = [5,5,0.01]

   
    scene.add_box("table",table_pose,table_scale)
    rospy.sleep(2) #if collision objects don't show try increasing these delays
    scene.attach_box("left_ee_link","leg",leg_pose,leg_scale)
    rospy.sleep(2)
    scene.add_box("floor",floor_pose,floor_scale)
    rospy.sleep(2)

  
