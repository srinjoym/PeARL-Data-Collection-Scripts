#!/usr/bin/env python

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from math import pi, floor, ceil, fabs, sin, cos, radians, atan
from os import listdir
import sys
import rospy
import geometry_msgs.msg
from std_msgs.msg import *
import wpi_jaco_msgs.msg
import wpi_jaco_msgs.srv
from control_msgs.msg import *
from vector_msgs.msg import *
import time
import requests
import tf
import tf_conversions
from arm_moveit import *
from move_table_leg import *
from collision_object import *

class CaptureImages:

  def __init__(self):
    #r = requests.get("http://10.5.5.9/gp/gpControl/setting/17/2") #to put camera in picture mode
    #r = requests.get("http://10.5.5.9/gp/gpControl/setting/72/0")
    #r = requests.get("http://10.5.5.9/gp/gpControl/setting/58/0")

    self.goal_position = [1.1,0,-.1] #point we are aiming for

    topic = 'visualization_marker_array'
    self.publisher = rospy.Publisher(topic, MarkerArray)
    rospy.sleep(2)
    self.arm = ArmMoveIt()
    self.markerArray = MarkerArray()
    self.next_marker_id = 0
    self.current_execution = 1
    self.file_name=""
    if(input("Publish Collision Object? (1 for yes)")==1):
      publish_collision_object(self.arm.scene,self.goal_position)

  def publish_point(self, pose,color):
    marker = Marker()
    marker.id = self.next_marker_id
    self.next_marker_id += 1
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g= color[1]
    marker.color.b = color[2]
    marker.pose.position.x = pose.position.x
    marker.pose.position.y = pose.position.y
    marker.pose.position.z = pose.position.z
    marker.pose.orientation.w = pose.orientation.w
    marker.pose.orientation.x = pose.orientation.x
    marker.pose.orientation.y = pose.orientation.y
    marker.pose.orientation.z = pose.orientation.z
    marker.header.frame_id = "/linear_actuator_link"
    # print self.marker
    # markerArray = MarkerArray()
    self.markerArray.markers.append(marker)
    

    self.publisher.publish(self.markerArray)


  def calc_orientation(self,angle,radius,height,center,rotation,tilt_angle):
    calc_tilt_angle = atan((height-center[2]/radius))

    quaternion = tf.transformations.quaternion_from_euler(0,radians(-90)+calc_tilt_angle, -radians(angle+90))
    pose1= geometry_msgs.msg.Pose()
    pose1.orientation.x = quaternion[0]
    pose1.orientation.y = quaternion[1]
    pose1.orientation.z = quaternion[2]
    pose1.orientation.w = quaternion[3] 
    return  [pose1.orientation]           

  def calc_mov(self,angle,radius,height,center):
    rad = radians(angle)
    x = center[0]+radius*(sin(rad))
    y = 1.0*radius*(cos(rad))
    z = height
    poseTmp= geometry_msgs.msg.Pose()
    poseTmp.position.x=x
    poseTmp.position.y=y
    poseTmp.position.z=z
    
    return poseTmp.position

  def get_next_pic(self):
    # r = requests.get("http://10.5.5.9/gp/gpControl/status")
    # data = r.json()
    # return data["status"]["38"] + 1
    return 0
    

  def log(self,status,height,radius,angle,rotation,tilt_angle,real_position=None,real_orientation=None):

      if status:
        with open(self.file_name, 'a+') as f:
          f.write("\nPicture %i"%int(self.get_next_pic())+"\n Height %i Radius %i Angle %i Rotation %i Tilt %i"%(height,radius,angle,rotation,tilt_angle)+"\nPosition\n"+str(real_position)+"\nOrientation\n"+str(real_orientation)+"\n")
      else:
        with open(self.file_name, 'a+') as f:
          f.write("\nPicture %i"%int(self.get_next_pic())+"\n Height %i Radius %i Angle %i Rotation %i Tilt %i"%(height,radius,angle,rotation,tilt_angle)+"\nFailed!!\n")
    
      with open(self.file_name, 'a+') as f:
            f.write("*************************************")
      

  def execute_circle(self,jump,radius,height,center):

    tarPose = geometry_msgs.msg.Pose()

    for angle in range(-135,-46,jump): #change angle range of circle, shouldn't need to touch this
    # for angle in range(-135,-110,jump):
      for tilt_angle in range(0,1,10):
      # for tilt_angle in range(0,1,10): 
        for rotation in range(0,1,15):
           
          tarPose.position = self.calc_mov(angle,radius,height,center)
          tarPose.orientation = self.calc_orientation(angle,radius,height,center,rotation,tilt_angle)[0]

          self.arm.group[1].set_pose_target(tarPose)

          if(self.arm.group[1].plan()): #for planning visualization only no execution 
          # if(self.arm.group[1].go()): #to actually move arm
            self.publish_point(tarPose,[0,1,0])
            self.log(True,height,radius,angle,rotation,tilt_angle,self.arm.get_FK()[0].pose.position,self.arm.get_FK()[0].pose.orientation)
          else:
            self.publish_point(tarPose,[1,0,0])
            self.log(False,height,radius,angle,rotation,tilt_angle)
          # r = requests.get("http://10.5.5.9/gp/gpControl/command/shutter?p=1")
          self.current_execution+=1

    with open(self.file_name, 'a+') as f:
            f.write("\nFinished Circle\n")

  def auto_circle(self,rad_outer,rad_inner,center):
    
    with open(self.file_name, 'a+') as f:
      f.write("\nStarting Run at Picture %i\n"%self.get_next_pic())
    # x_back_limit = 0.62
    x_forward_limit = 1.2
    # y_limit = 0.3
    centerPose = geometry_msgs.msg.Pose()
    centerPose.position.x = center[0]
    centerPose.position.y = center[1]
    centerPose.position.z = center[2]
    self.publish_point(centerPose,[0,0,1] )
    #if(input("Continue")==-1):
    # return
    jump = 22 #hard coded for now
    tarPose = geometry_msgs.msg.Pose()
    base_radius = 0.7
    base_height = 0.25 #change this to make both sets of points higher

    for angle in range(0,30,15):
      height = base_radius*sin(radians(angle)) #increasing
      radius = base_radius*cos(radians(angle)) #decreasing
      self.execute_circle(jump,rad_outer-height,base_height+height,center) 

    for angle in range(0,30,15):
      height = base_radius*sin(radians(angle)) #increasing
      radius = base_radius*cos(radians(angle)) #decreasing
      self.execute_circle(jump,rad_inner-height,base_height+height,center)
    

    with open(self.file_name, 'a+') as f:
      f.write("\nFinished Run at Picture %i\n"%(self.get_next_pic()-1))

  

def main():
  
  capture_img = CaptureImages()

  ## ask if integrate object scene from code or not
  
  if not rospy.is_shutdown():
    #r = requests.get("http://10.5.5.9/gp/gpControl/command/shutter?p=1")

    ##   Assigned tarPose the current Pose of the robotlp 
    #tarPose = arm.group[0].get_current_pose().pose
    #arm.auto_circle(0.57,0.35,[1.3,0,-0.35])
    last_file = 0
    for file_name in listdir("../data"):
      if(int(file_name[0:len(file_name)-4])>last_file):
        last_file = int(file_name[0:len(file_name)-4])

    capture_img.file_name="../data/%i.txt"%(last_file+1)

    with open(capture_img.file_name, 'w+') as f:
      pass

    

    move_leg(capture_img.arm,capture_img.goal_position) #move left arm for table leg

    capture_img.auto_circle(0.7,0.5,capture_img.goal_position) # get pictures with right arm
       
  
if __name__ == '__main__':
  ## First initialize moveit_commander and rospy.
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('vector_basic_IK', anonymous=True)
  
  main()
