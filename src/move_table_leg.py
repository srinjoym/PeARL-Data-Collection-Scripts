#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
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
from ar_track_alvar_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from std_msgs.msg import *
from math import pi, floor, ceil, fabs, sin, cos, radians,degrees
import numpy
from arm_moveit import *


def get_goal_pos(marker_pos,robot_pos,goal_pos):

  diff = geometry_msgs.msg.Pose()

  diff.position.x = robot_pos.position.x - marker_pos.position.x
  diff.position.y = robot_pos.position.y - marker_pos.position.y
  diff.position.z = robot_pos.position.z - marker_pos.position.z + 0.1

  angleRobot = tf.transformations.euler_from_quaternion([robot_pos.orientation.x,robot_pos.orientation.y,robot_pos.orientation.z,robot_pos.orientation.w])
  raw_angleMarker = tf.transformations.euler_from_quaternion([marker_pos.orientation.x,marker_pos.orientation.y,marker_pos.orientation.z,marker_pos.orientation.w])
  angleGoal = tf.transformations.euler_from_quaternion([goal_pos.orientation.x,goal_pos.orientation.y,goal_pos.orientation.z,goal_pos.orientation.w])

  adj_angleMarker=[0]*3
  adj_angleMarker[0]=raw_angleMarker[0] #accounting for leg being vertical in arm
  adj_angleMarker[1]=raw_angleMarker[1]
  adj_angleMarker[2]=raw_angleMarker[2]

  diffAngle = [0]*3
  diffAngle[0]=(adj_angleMarker[0]-angleRobot[0])
  diffAngle[1]=(adj_angleMarker[1]-angleRobot[1])  
  diffAngle[2]=(adj_angleMarker[2]-angleRobot[2])

  print "angle robot"
  print [degrees(angleRobot[0]),degrees(angleRobot[1]),degrees(angleRobot[2])]

  print "angle marker"
  print [degrees(adj_angleMarker[0]),degrees(adj_angleMarker[1]),degrees(adj_angleMarker[2])]

  print "diff angle"
  print [degrees(diffAngle[0]),degrees(diffAngle[1]),degrees(diffAngle[2])]

  targetAngle = [0]*3
  targetAngle[0] = angleGoal[0]-diffAngle[0]
  targetAngle[1] = angleGoal[1]-diffAngle[1]
  targetAngle[2] = angleGoal[2]-diffAngle[2]

  print "target angle"
  print targetAngle

  targetQuaternion = tf.transformations.quaternion_from_euler(targetAngle[0],targetAngle[1],targetAngle[2])

  target = geometry_msgs.msg.Pose()

  if(diff.position.x<0.1 and diff.position.y<0.1 and diff.position.z<0.1 and diffAngle[0]<radians(20) and diffAngle[1]<radians(20) and diffAngle[2]<radians(20)):
    target.position.x = goal_pos.position.x+diff.position.x
    target.position.y =goal_pos.position.y+ diff.position.y
    target.position.z = goal_pos.position.z+diff.position.z
    target.orientation.x = targetQuaternion[0]
    target.orientation.y = targetQuaternion[1]
    target.orientation.z = targetQuaternion[2]
    target.orientation.w = targetQuaternion[3]
    print "diff:"
    print diff
    print "goal"
    print goal_pos
    print "target:"
    print target
    return target
  else:
    print "error exceeded bounds"
    return goal_pos
  


def move_hole(arm,goal_pos):

  # self.publish_point(pre_goal_pos,[0,1,0])

  if(input("continue?")==-1):
    return

  arm.group[0].set_pose_target(goal_pos)
  print "going"
  arm.group[0].go()

  
def move_leg(arm,goal_position):
  counter = 0
  print "here"

  goal_angle = tf.transformations.quaternion_from_euler(0,0,0)
  goal_pos = geometry_msgs.msg.Pose()
  goal_pos.position.x = goal_position[0]
  goal_pos.position.y = goal_position[1]
  goal_pos.position.z = goal_position[2]
  goal_pos.orientation.x =0
  goal_pos.orientation.y = 0
  goal_pos.orientation.z = 0
  goal_pos.orientation.w = 1


  print "before above hole"
  goal_pos.position.z += 0.1
  move_hole(arm,goal_pos)

  goal_pos.position.z -= 0.1
  move_hole(arm,goal_pos)

     
