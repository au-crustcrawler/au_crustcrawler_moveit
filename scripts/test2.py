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

def cmd_from_copy_paste(group,x,y,z,rpy):
  p = geometry_msgs.msg.Pose()
  p.position.x = x
  p.position.y = y
  p.position.z = z
  p.orientation.x = 1

  group.set_rpy_target(rpy,"wrist_temp")

  group.set_joint_value_target(p,"wrist_temp",True)


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
  #group.set_planner_id("SBLkConfigDefault")
  group.set_goal_tolerance(0.01);
  group.set_planner_id("SBLkConfigDefault")
  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot joint state"
  #print robot.get_current_state()
  print "============"

  print "============ Printing robot end-effector pose"
  #print group.get_current_pose()
  print "============"
  ps  = group.get_current_pose()


  print "============ Generating plan 1"
  #
  # move the end effector 0.05 m in the x direction (in the 'world' frame).
  ##

  cmd_from_copy_paste(group, 0.0930027332332, 0.0255666028557, 0.353285402227,[1.5707963172378656, -0.2418662666507342, 0.26827474033541865] )

  plan1 = group.plan()

  rospy.sleep(5)
  group.go(wait=True)

  cmd_from_copy_paste(group, 0.188432617112, 0.0279251403231, 0.15642730647,[1.5707963164889822, -0.4502407290730125, 0.1471261210910023])
  plan1 = group.plan()

  rospy.sleep(5)
  group.go(wait=True)

  cmd_from_copy_paste(group, 0.188432617112, 0.0279251403231, 0.15642730647,[1.5707963164889822,-0.4502407290730125, -0.3])
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
