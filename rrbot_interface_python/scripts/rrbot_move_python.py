#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs

import tf

from tf.transformations import *
from geometry_msgs.msg import Quaternion

from math import pi


class MoveGroupPythonInterfaceTutorial:
    def __init__(self):
        rospy.init_node("rrbot_move_python", anonymous=True)

        self.robot=moveit_commander.RobotCommander()
        self.arm_group=moveit_commander.move_group.MoveGroupCommander("manipulator")
        self.gripper_group=moveit_commander.move_group.MoveGroupCommander("gripper")
        self.arm_group.set_named_target("stand_by")
        self.plan = self.arm_group.go(wait=True)
        rospy.sleep(1)

    def go_to_joint_state(self):
        group=self.arm_group

        joint_goal=group.get_current_joint_values()
        joint_goal[0]= 0
        joint_goal[1]= -pi/4
        joint_goal[2]= 0
        joint_goal[3]= -pi/4
        joint_goal[4]= 0
        joint_goal[5]= pi/3

        group.go(joint_goal, wait=True)

        group.stop()

        current_joints=group.get_current_joint_values()
        print(current_joints)

    def move_to_standby(self):
        group=self.arm_group

        group.set_named_target("stand_by")
        group.go(wait=True)
        group.stop()
    
    def move_to_pose_goal(self):
        group=self.arm_group

        pose_goal=geometry_msgs.msg.Pose()
        current_pose=group.get_current_pose().pose
        pose_goal=current_pose

        pose_goal.position.x+=0.1
        pose_goal.position.y+=0.3
        pose_goal.position.z-=0.3

        group.set_pose_target(pose_goal)
        plan=group.go(wait=True)

        group.stop()
        group.clear_pose_targets()

        current_pose=group.get_current_pose().pose
        print(current_pose)
    
    def move_orientation(self):
        group=self.arm_group

        pose_goal=group.get_current_pose().pose
        q_orig=[pose_goal.orientation.w, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z]
        
        q_rot=quaternion_from_euler(1.5707, 0, 0)

        q_new=quaternion_multiply(q_orig, q_rot)
        

        pose_goal.orientation=Quaternion(q_new[0],q_new[1],q_new[2],q_new[3])

        group.set_pose_target(pose_goal)
        plan=group.go(wait=True)

        group.stop()
        group.clear_pose_targets()
    
    def move_gripper(self):
        group=self.gripper_group

        group.set_named_target("open")
        group.go(wait=True)
        group.stop()

        group.set_named_target("close")
        group.go(wait=True)
        group.stop()

    

def main(args):
    try:
        moveitExample=MoveGroupPythonInterfaceTutorial()
        
        moveitExample.go_to_joint_state()

        moveitExample.move_to_standby()

        moveitExample.move_to_pose_goal()

        moveitExample.move_to_standby()

        moveitExample.move_orientation()

        moveitExample.move_to_standby()

        moveitExample.move_gripper()
    except KeyboardInterrupt:
        print("Shutting down")
    
    moveit_commander.roscpp_initializer.roscpp_shutdown()


if __name__ == '__main__':
    main(sys.argv)