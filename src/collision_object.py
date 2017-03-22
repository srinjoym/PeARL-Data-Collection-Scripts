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

class CollisionObject:

  def __init__(self, planning_frame='linear_actuator_link', default_planner="RRTConnectkConfigDefault"):
    # r = requests.get("http://10.5.5.9/gp/gpControl/command/mode?p=1")
    # Make sure the moveit service is up and running
    rospy.logwarn("Starting up")
    # rospy.init_node("tag_track")

    self.robot = moveit_commander.RobotCommander()

    self.scene = moveit_commander.PlanningSceneInterface()

    self.group = [moveit_commander.MoveGroupCommander("arm")] #change this to right_arm or left_arm

    self.planner = default_planner

    self.group[0].set_pose_reference_frame(planning_frame)

    self.continuous_joints = ['left_shoulder_pan_joint','left_wrist_1_joint','left_wrist_2_joint','left_wrist_3_joint']
    self.continuous_joints_list = [0,3,4,5] # joints that are continous

    # self.publisher = rospy.Publisher('collision_object', CollisionObject)
    # self.publisher = rospy.Publisher('planning_scene', PlanningScene)
    self.publisher = rospy.Publisher('visualization_marker_array', MarkerArray)
    self.markerArray = MarkerArray()
    rospy.sleep(2)

  
 
  def publish_collision_object(self):
    
    right_pose = geometry_msgs.msg.PoseStamped()
    right_pose.header.frame_id = "linear_actuator_link"
    right_pose.pose.position.x = 0.1;
    right_pose.pose.position.y = 0;
    right_pose.pose.position.z = 0.1;
    right_pose.pose.orientation.x = 0;
    right_pose.pose.orientation.y = 0;
    right_pose.pose.orientation.z = 0;
    right_pose.pose.orientation.w = 1;

    right_scale = [.1,.1,.1]
    
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = "base_link"
    table_pose.pose.position.x = 0.6+0.8;
    table_pose.pose.position.y = 0;
    table_pose.pose.position.z = .4;
    table_pose.pose.orientation.x = 0;
    table_pose.pose.orientation.y = 0;
    table_pose.pose.orientation.z = 0;
    table_pose.pose.orientation.w = 1;

    table_scale = [0.6,1.23,.74]
    
    leg_pose = geometry_msgs.msg.PoseStamped()
    leg_pose.header.frame_id = "linear_actuator_link"
    leg_pose.pose.position.x = 1.2;
    leg_pose.pose.position.y = 0;
    leg_pose.pose.position.z = -1;
    leg_pose.pose.orientation.x = 0;
    leg_pose.pose.orientation.y = 0;
    leg_pose.pose.orientation.z = 0;
    leg_pose.pose.orientation.w = 1;

    leg_scale = [0.05,0.05,3]
    self.scene.add_box("table_leg",right_pose,right_scale)
    rospy.sleep(5)
    self.scene.add_box("right_arm",table_pose,table_scale)
    rospy.sleep(5)
    #self.scene.add_box("table",leg_pose,leg_scale)
    #rospy.sleep(2)

def main():
  collisionObject = CollisionObject()
  counter = 0
  if (not(rospy.is_shutdown())):
    # print tagTracker.get_FK()[0].pose
    collisionObject.publish_collision_object()

    print "done" + str(counter)
    counter+=1
    rospy.sleep(1)
  ## ask if integrate object scene from code or not
  
    ##   Assigned tarPose the current Pose of the robot 
       
  
if __name__ == '__main__':
  ## First initialize moveit_commander and rospy.  
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('vector_basic_IK', anonymous=True)
  main()
  rospy.spin()
