#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
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
from move_base import *
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from math import pi, floor, ceil, fabs, sin, cos, radians, atan
from os import listdir

class ArmMoveIt:

  def __init__(self, planning_frame='linear_actuator_link', default_planner="RRTConnectkConfigDefault"):
    # r = requests.get("http://10.5.5.9/gp/gpControl/command/mode?p=1")
    # Make sure the moveit service is up and running
    rospy.logwarn("Waiting for MoveIt! to load")
    try:
      rospy.wait_for_service('compute_ik')
    except rospy.ROSExecption, e:
      rospy.logerr("No moveit service detected. Exiting")
      exit()
    else:
      rospy.loginfo("MoveIt detected: arm planner loading")

    # self.pose = geometry_msgs.msg.PoseStamped()
    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    self.robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    self.scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute   motions on the left
    ## arm.
    self.group = [moveit_commander.MoveGroupCommander("left_arm")]

    # Set the planner
    self.planner = default_planner

    # Set the planning pose reference frame
    self.group[0].set_pose_reference_frame(planning_frame)
    # print "tolerance:"
    # print self.group[0].set_num_planning_attempts(5)
    # Set continuous joint names
    self.continuous_joints = ['left_shoulder_pan_joint','left_wrist_1_joint','left_wrist_2_joint','left_wrist_3_joint']
    # NOTE: order that moveit currently is configured
    # ['right_shoulder_pan_joint', 'right_shoulder_lift_joint', 'right_elbow_joint', 'right_wrist_1_joint', 'right_wrist_2_joint', 'right_wrist_3_joint']
    self.continuous_joints_list = [0,3,4,5,6,9,10,11] # joints that are continous
    topic = 'visualization_marker_array'
    self.publisher = rospy.Publisher(topic, MarkerArray)
    self.lin_act_controller = rospy.Publisher('/vector/linear_actuator_cmd', LinearActuatorCmd)
    rospy.Subscriber('/linear_actuator_controller/state', JointTrajectoryControllerState, self.update_lin_act_callback)
    rospy.sleep(2)
    self.markerArray = MarkerArray()
    self.move_base = MoveBase()
    self.current_execution = 1
    self.lin_act_state = control_msgs.msg.JointTrajectoryControllerState()
    self.file_name=""


  def update_lin_act_callback(self,msg):
    self.lin_act_state = msg;
  
  def get_FK(self, root = 'base_link'):

    rospy.wait_for_service('compute_fk')
    compute_fk = rospy.ServiceProxy('compute_fk', moveit_msgs.srv.GetPositionFK)

    header = std_msgs.msg.Header()
    header.frame_id = root
    header.stamp = rospy.Time.now()
    fk_link_names = ['left_ee_link']
    robot_state = self.robot.get_current_state()    
    try:
      reply=compute_fk(header,fk_link_names,robot_state)
      return reply.pose_stamped

    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def get_IK(self, newPose, root = None):
    ## from a defined newPose (geometry_msgs.msg.Pose()), retunr its correspondent joint angle(list)
    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', moveit_msgs.srv.GetPositionIK)

    wkPose = geometry_msgs.msg.PoseStamped()
    if root is None:
      wkPose.header.frame_id = 'linear_actuator_link' # name:odom
    else:
      wkPose.header.frame_id = root

    wkPose.header.stamp=rospy.Time.now()
    wkPose.pose=newPose

    msgs_request = moveit_msgs.msg.PositionIKRequest()
    msgs_request.group_name = self.group[0].get_name() # name: arm
    # msgs_request.robot_state = robot.get_current_state()
    msgs_request.pose_stamped = wkPose
    msgs_request.timeout.secs = 2
    msgs_request.avoid_collisions = False

    try:
      jointAngle=compute_ik(msgs_request)
      ans=list(jointAngle.solution.joint_state.position[1:7])
      if jointAngle.error_code.val == -31:
        print 'No IK solution'
        return None
      return ans

    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def _simplify_angle(self, angle):
    # Very simple function that makes sure the angles are between -pi and pi
    if angle > pi:
      while angle > pi:
        angle -= 2*pi
    elif angle < -pi:
      while angle < -pi:
        angle += 2*pi

    return angle

  def _simplify_joints(self, joint_dict):

    if isinstance(joint_dict, dict):
      simplified_joints = dict()
      for joint in joint_dict:
        # Pull out the name of the joint
        joint_name = '_'.join(joint.split('_')[1::])
        if joint_name in self.continuous_joints:
          simplified_joints[joint] = self._simplify_angle(joint_dict[joint])
        else:
          simplified_joints[joint] = joint_dict[joint]
    elif isinstance(joint_dict, list):
      simplified_joints = []
      for i in xrange(len(joint_dict)):
      	
        a = joint_dict[i]

        if i in self.continuous_joints_list:
          simplified_joints.append(self._simplify_angle(a))
        else:
          simplified_joints.append(a)
    return simplified_joints

#   '''Older functions - left for backwards compatibility'''

  def plan_jointTargetInput(self,target_joint):
    ## input: target joint angles (list) of the robot
    ## output: plan from current joint angles to the target one
    try:
      self.group[0].set_joint_value_target(self._simplify_joints(target_joint))
      self.group[0].set_planner_id(self.planner)
      planAns=self.group[0].plan()
      return planAns
    except:
      print 'No plan found, see the moveit terminal for the error'
      print("Unexpected error:", sys.exc_info()[0])
      return None

  
  def publish_point(self, pose,color):
    marker = Marker()
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g= color[1]
    marker.color.b = color[2]
    marker.pose = pose
    marker.header.frame_id = "/linear_actuator_link"
    # print self.marker
    #markerArray = MarkerArray()
    self.markerArray.markers.append(marker)
    print "marker array"
    print self.markerArray
    id = 0
    for m in self.markerArray.markers:
      m.id = id
      print str(id)+"\n"
      print m.pose
      id += 1
    # print self.markerArray
    self.publisher.publish(self.markerArray)


  def calc_orientation(self,angle,radius,height,center,rotation,tilt_angle):
    calc_tilt_angle = atan((height-center[2]/radius))
    print "tilt angle"
    print calc_tilt_angle
    quaternion = tf.transformations.quaternion_from_euler(radians(rotation), radians(angle+90),radians(90)+radians(tilt_angle)+calc_tilt_angle,axes='szxy')
    # ang_quaternion = tf.transformations.quaternion_from_euler(0, 0,-radians(15))

    # quaternion = tf.transformations.quaternion_multiply(ba_quaternion,tf.transformations.quaternion_inverse(ang_quaternion))

    pose= geometry_msgs.msg.Pose()
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]  
    return  pose.orientation                

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
    r = requests.get("http://10.5.5.9/gp/gpControl/status")
    data = r.json()
    return data["status"]["38"] + 1
    

  def log(self,status,height,radius,angle,rotation,tilt_angle,real_position=None,real_orientation=None):
      if status:
        with open(self.file_name, 'a+') as f:
          f.write("\nPicture %i"%int(self.get_next_pic())+"\n Height %i Radius %i Angle %i Rotation %i Tilt %i"%(height,radius,angle,rotation,tilt_angle)+"\nPosition\n"+str(real_position)+"\nOrientation\n"+str(real_orientation)+"\n")
      else:
        with open(self.file_name, 'a+') as f:
          f.write("\nPicture %i"%int(self.get_next_pic())+"\n Height %i Radius %i Angle %i Rotation %i Tilt %i"%(height,radius,angle,rotation,tilt_angle)+"\nFailed!!\n")
    
      with open(self.file_name, 'a+') as f:
            f.write("\n*************************************\n")
      

  def execute_circle(self,jump,radius,height,center):

    tarPose = geometry_msgs.msg.Pose()

    for angle in range(-135,-46,jump):
  # for angle in range(-135,-134,jump):
      for tilt_angle in range(-10,11,10):
        for rotation in range(-15,16,15):
        # for rotation in range(0,1,20):
           
          tarPose.position = self.calc_mov(angle,radius,height,center)
          tarPose.orientation = self.calc_orientation(angle,radius,height,center,rotation,tilt_angle)
          print "Rotation ",rotation
          print "Angle: ", angle
          print "Radius: ", radius
          print "Height: ", height
          print "center: ", center
          print '\n The target coordinate is: %s \n' %tarPose
          jointTarg = self.get_IK(tarPose)
          planTraj = self.plan_jointTargetInput(jointTarg)
          if(planTraj!=None):
            self.publish_point(tarPose,[0,1,0])
            print "going to angle " + str(angle)   
            self.group[0].execute(planTraj)
<<<<<<< HEAD
            self.log(True,height,radius,angle,rotation,tilt_angle,self.get_FK()[0].pose.position,self.get_FK()[0].pose.orientation)
          else:
            self.publish_point(tarPose,[1,0,0])
            self.log(False,height,radius,angle,rotation,tilt_angle)            
          # r = requests.get("http://10.5.5.9/gp/gpControl/command/shutter?p=1")
=======
            with open('output.txt', 'a+') as f:
              f.write("\nExecution %f"%int(self.current_execution)+"\n Height %f Radius %f Angle %f Rotation %f Tilt %f"%(height,radius,angle,rotation,tilt_angle)+"\nPosition\n"+str(self.get_FK()[0].pose.position)+"\nOrientation\n"+str(self.get_FK()[0].pose.orientation)+"\n")
          
          else:
            self.publish_point(tarPose,[1,0,0])
            with open('output.txt', 'a+') as f:
              f.write("\nExecution %f"%int(self.current_execution)+"\n Height %f Radius %f Angle %f Rotation %f Tilt %f"%(height,radius,angle,rotation,tilt_angle)+"\nFailed!!\n")
          r = requests.get("http://10.5.5.9/gp/gpControl/command/shutter?p=1")
>>>>>>> 7bb42322ad615074af1c37aac152f97569ab5bb2
          self.current_execution+=1

    with open(self.file_name, 'a+') as f:
            f.write("\nFinished Circle\n")
    return id

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
    #	return
    jump = 22 #hard coded for now
    tarPose = geometry_msgs.msg.Pose()
    
<<<<<<< HEAD
    # self.execute_circle(jump,rad_outer,-0.45,center)
    # self.execute_circle(jump,rad_inner,-0.45,center)
    # self.execute_circle(jump,rad_outer,-0.35,center)
    # self.execute_circle(jump,rad_inner,-0.35,center)
    # self.execute_circle(jump,rad_outer,-0.25,center)
=======
    #self.execute_circle(jump,rad_outer,-0.45,center)
    # self.execute_circle(jump,rad_inner,-0.45,center)
    self.execute_circle(jump,rad_outer,-0.25,center)
    self.execute_circle(jump,rad_inner,-0.25,center)
    self.execute_circle(jump,rad_outer,-0.1,center)
>>>>>>> 7bb42322ad615074af1c37aac152f97569ab5bb2
    self.execute_circle(jump,rad_inner,-0.1,center)
    # self.move_lin_act(self.lin_act_state-0.1)
    # self.execute_circle(jump,rad_outer,-0.3,center)
    # self.execute_circle(jump,rad_inner,-0.3,center)

    

    # self.execute_circle(jump,rad_outer,-0.3,center)
    # self.execute_circle(jump,rad_inner,-0.3,center)
    # self.execute_circle(jump,rad_outer,-0.3,center)
    # self.execute_circle(jump,rad_inner,-0.3,center)
    

    # self.move_base.simple_move(center,1)
    # self.execute_circle(jump,rad_outer,center)
    # self.execute_circle(jump,rad_inner,center)

    with open(self.file_name, 'a+') as f:
      f.write("\nFinished Run at Picture %i\n"%(self.get_next_pic()-1))

  def move_lin_act(self,diff):
    rospy.sleep(2)
    current_state = self.lin_act_state.actual.positions[0]
    desired_state = LinearActuatorCmd()

    desired_state.desired_position_m = current_state+diff

    self.lin_act_controller.publish(desired_state)

def main():
  arm = ArmMoveIt()

  tarPose = geometry_msgs.msg.Pose()

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

    arm.file_name="../data/%i.txt"%(last_file+1)

    with open(arm.file_name, 'w+') as f:
      pass

    arm.auto_circle(0.75,0.62,[1.1,0,-0.35])
       
  
if __name__ == '__main__':
  ## First initialize moveit_commander and rospy.
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('vector_basic_IK', anonymous=True)
  
  main()

