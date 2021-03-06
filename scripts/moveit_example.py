#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
## END_SUB_TUTORIAL

from std_msgs.msg import String

def move_group_python_interface_tutorial():

  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("arm")
  group.set_goal_tolerance(0.1);
  group.set_planner_id("SBLkConfigDefault")
  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot joint state"
  print robot.get_current_state()
  print "============"

  print "============ Printing robot end-effector pose"
  print group.get_current_pose()
  print "============"
  ps  = group.get_current_pose()


  print "============ Generating plan 1"
  ps.pose.position.x= 0.104167691349
  ps.pose.position.y= 0.374989376334
  ps.pose.position.z= 0.066092524209

  ps.pose.orientation.x= -0.531954657323
  ps.pose.orientation.y= 0.684182690672
  ps.pose.orientation.z= 0.297510390091
  ps.pose.orientation.w= 0.400506998846



  #
  # move the end effector 0.05 m in the x direction (in the 'world' frame).

  group.set_joint_value_target(ps,None,True)

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group
  ## to actually move the robot
  plan1 = group.plan()

  rospy.sleep(5)
  group.go(wait=True)


  ps.pose.position.x= -0.0250002736296
  ps.pose.position.y= -1.27738778012e-05
  ps.pose.position.z= 0.600999087285

  ps.pose.orientation.x= 2.1157133798e-05
  ps.pose.orientation.y= -4.69467052624e-05
  ps.pose.orientation.z= 0.707069008992
  ps.pose.orientation.w= 0.707144549488

  group.set_joint_value_target(ps,None,True)

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group
  ## to actually move the robot
  plan1 = group.plan()

  rospy.sleep(5)
  group.go(wait=True)


  ps.pose.position.x=0.284533829625
  ps.pose.position.y= 0.095949568364
  ps.pose.position.z= 0.0320462807192

  ps.pose.orientation.x= 0.115536326545
  ps.pose.orientation.y= 0.963074963091
  ps.pose.orientation.z= 0.104567923768
  ps.pose.orientation.w= 0.219553005069




  group.set_joint_value_target(ps,None,True)

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group
  ## to actually move the robot
  plan1 = group.plan()

  rospy.sleep(5)
  group.go(wait=True)


  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
