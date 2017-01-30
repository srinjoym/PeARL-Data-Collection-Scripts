#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
import std_msgs.msg
import wpi_jaco_msgs.msg
import wpi_jaco_msgs.srv
import time
from math import pi, floor, ceil, fabs, sin, cos, radians

class ArmMoveIt:

  def __init__(self, planning_frame='base_link', default_planner="RRTConnectkConfigDefault"):

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
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    self.group = [moveit_commander.MoveGroupCommander("arm")]

    # Set the planner
    self.planner = default_planner

    # Set the planning pose reference frame
    self.group[0].set_pose_reference_frame(planning_frame)

    # Set continuous joint names
    self.continuous_joints = ['shoulder_pan_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
    # NOTE: order that moveit currently is configured
    # ['right_shoulder_pan_joint', 'right_shoulder_lift_joint', 'right_elbow_joint', 'right_wrist_1_joint', 'right_wrist_2_joint', 'right_wrist_3_joint']
    self.continuous_joints_list = [0,3,4,5] # joints that are continous

  def get_IK(self, newPose, root = None):
    ## from a defined newPose (geometry_msgs.msg.Pose()), retunr its correspondent joint angle(list)
    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', moveit_msgs.srv.GetPositionIK)

    wkPose = geometry_msgs.msg.PoseStamped()
    if root is None:
      wkPose.header.frame_id = self.group[0].get_planning_frame() # name:odom
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

  
  def plan_targetInput(self, target, joint_flag):
    '''Generic target planner that what type is specified'''
    try:
      if (joint_flag):
        self.group[0].set_joint_value_target(self._simplify_joints(target))
      else:
        self.group[0].set_pose_target(target)

      self.group[0].set_planner_id(self.planner)
      planAns=self.group[0].plan()
      return planAns
    except:
      print 'No plan found, see the moveit terminal for the error'
      print("Unexpected error:", sys.exc_info()[0])
      return None


  def set_robot_state_pose(self, traj):
    '''Gets the current robot state pose and sets it to the joint pose'''
    cur_robot_state = self.robot.get_current_state()
    last_point = traj.points[-1].positions
    # convert the joints to array
    joints = [x for x in cur_robot_state.joint_state.position]
    for i in xrange(len(traj.joint_names)):
      # Find index of joint
      joint_name = traj.joint_names[i]
      idx = cur_robot_state.joint_state.name.index(joint_name)
      joints[idx] = last_point[i]

    # Set full joint tuple now
    cur_robot_state.joint_state.position = joints

    return cur_robot_state


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
    # Helper function to convert a dictionary of joint values
    print "entered"
    print joint_dict
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
    print simplified_joints
    return simplified_joints

#   '''Older functions - left for backwards compatibility'''

  def plan_jointTargetInput(self,target_joint):
    ## input: target joint angles (list) of the robot
    ## output: plan from current joint angles to the target one
    try:
      print target_joint
      print self._simplify_joints(target_joint)
      self.group[0].set_joint_value_target(self._simplify_joints(target_joint))
      self.group[0].set_planner_id(self.planner)
      planAns=self.group[0].plan()
      return planAns
    except:
      print 'No plan found, see the moveit terminal for the error'
      print("Unexpected error:", sys.exc_info()[0])
      return None

  def plan_poseTargetInput(self,target_pose):
    ## input: tart pose (geometry_msgs.msg.Pose())
    ## output: plan from current  pose to the target one
    try:
      self.group[0].set_pose_target(target_pose)
      self.group[0].set_planner_id(self.planner)
      planAns=self.group[0].plan()
      return planAns
    except:
      print 'No plan found, see the moveit terminal for the error'
      print("Unexpected error:", sys.exc_info()[0])
      return None
 

  
def ask_position(arm,tarPose):
  #Ask the user the values of the target position
   while True:
    try:   
      inputPosition=input(""" \n Target position coord. (format: x,y,z or write -1 to take the robot current position ): """)      
    
      if inputPosition == -1:
        inputPosition=tarPose.position  
        return inputPosition
      
    except (ValueError,IOError,NameError):
      print("\n Please, enter the coordinate in the following format: x,y,z ")
      continue
    else:          
      if len(list(inputPosition)) == 3:
        poseTmp= geometry_msgs.msg.Pose()
        poseTmp.position.x=inputPosition[0]
        poseTmp.position.y=inputPosition[1]
        poseTmp.position.z=inputPosition[2]
        return poseTmp.position
      else:
        print("\n Please, enter the coordinate in the following format: x,y,z ")
        continue
      
        
def ask_orientation(arm,tarPose):
  # Ask the user the values of the target quaternion
  while True:
    try:   
      inputQuat=input(""" \n Target quaternion coordi. (format: qx,qy,qz,qw or write -1 to take the robot current quaternion ):""")
      
      if inputQuat == -1:
        inputQuat=arm.group[0].get_current_pose().pose.orientation                   
        return  inputQuat
        
    except (ValueError,IOError,NameError):
      print("\n Please, enter the coordinate in the following format: qx,qy,qz,qw ")
      continue
    else:
      if len(list(inputQuat)) == 4:
        poseTmp= geometry_msgs.msg.Pose()
        poseTmp.orientation.x=inputQuat[0]
        poseTmp.orientation.y=inputQuat[1]
        poseTmp.orientation.z=inputQuat[2]
        poseTmp.orientation.w=inputQuat[3]
        return poseTmp.orientation    
      else:
        print("\n Please, enter the coordinate in the following format: qx,qy,qz,qw ")

def ask_angle():
  return input("Angle?")

def calc_orientation(angle):
  poseTmp= geometry_msgs.msg.Pose()
  poseTmp.orientation.x=0
  poseTmp.orientation.y=0
  poseTmp.orientation.z = -radians(angle+90)
  poseTmp.orientation.w= 1
  # inputQuat=arm.group[0].get_current_pose().pose.orientation                   
  return  poseTmp.orientation                
  
#   return poseTmp.position
# def calculate_mov(angle):
#   radius = 0.175
#   rad = radians(angle)
#   x = 1.175 + radius*(sin(rad))
#   y = 0.2+ radius*(cos(rad))
#   z = 1.2
#   poseTmp= geometry_msgs.msg.Pose()
#   poseTmp.position.x=x
#   poseTmp.position.y=y
#   poseTmp.position.z=z
  
#   return poseTmp.position
def auto_circle(num_points,arm):
  jump = (35/num_points)/2
  tarPose = geometry_msgs.msg.Pose()
  for angle in range(-75,-40,jump):
      tarPose.position = calc_mov(angle)
      tarPose.orientation = calc_orientation(angle)
      jointTarg = arm.get_IK(tarPose)
      planTraj = arm.plan_jointTargetInput(jointTarg)
      if(planTraj!=None):
        arm.group[0].execute(planTraj)
        print "going to angle " + str(angle)
        time.sleep(5)

  for angle in range(-130,-105,jump):
      tarPose.position = calc_mov(angle)
      tarPose.orientation = calc_orientation(angle)
      jointTarg = arm.get_IK(tarPose)
      planTraj = arm.plan_jointTargetInput(jointTarg)
      if(planTraj!=None):
        arm.group[0].execute(planTraj)
        print "going to angle " + str(angle)
        time.sleep(5)


def calc_mov(angle):
  radius = 0.5
  rad = radians(angle)
  x = radius*(sin(rad))+1.3
  y = radius*(cos(rad))
  z = 1.2
  poseTmp= geometry_msgs.msg.Pose()
  poseTmp.position.x=x
  poseTmp.position.y=y
  poseTmp.position.z=z
  
  return poseTmp.position

def main():
  arm = ArmMoveIt()

  tarPose = geometry_msgs.msg.Pose()

  ## ask if integrate object scene from code or not
  
  while not rospy.is_shutdown():
    
    ##   Assigned tarPose the current Pose of the robot 
    tarPose = arm.group[0].get_current_pose().pose
  
    auto_circle(4,arm)
    ## ask input from user (COMMENT IF NOT USE AND WANT TO ASSIGN MANUAL VALUE IN CODE)    
    # angle = ask_angle()
    # tarPose.position = calc_mov(angle) 
    # tarPose.orientation = calc_orientation(angle)    
                  
    print '\n The target coordinate is: %s \n' %tarPose     
    
    ## IK for target position  
    # jointTarg = arm.get_IK(tarPose)
    # print 'IK calculation step:DONE' 
    # print jointTarg

    # ## planning with joint target from IK 
    # planTraj =  arm.plan_jointTargetInput(jointTarg)
    # print 'Planning step with target joint angles:DONE' 
    # if(planTraj != None):
    #    print 'Execution of the plan' 
    #    arm.group[0].execute(planTraj)
    ## planning with pose target
    # print 'Planning step with target pose'   
    # planTraj = arm.plan_poseTargetInput(tarPose)
      
    ## execution of the movement   
   
  
if __name__ == '__main__':
  ## First initialize moveit_commander and rospy.
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('vector_basic_IK', anonymous=True)
  
  main()

