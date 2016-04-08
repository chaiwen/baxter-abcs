#!/usr/bin/env python

import rospy
import moveit_commander

rospy.init_node('move_arm', anonymous=False)

left_arm_group = moveit_commander.MoveGroupCommander('left_arm')
right_arm_group = moveit_commander.MoveGroupCommander('right_arm')

end_pose = right_arm_group.get_random_pose()

print end_pose

right_arm_group.set_start_state_to_current_state()
right_arm_group.set_pose_target(end_pose)

plan = right_arm_group.plan()
right_arm_group.execute(plan)



