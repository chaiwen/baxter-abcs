#!/usr/bin/env python

import rospy
import moveit_commander

from std_msgs.msg import String

rospy.init_node('move_arm', anonymous=False)

left_arm_group = moveit_commander.MoveGroupCommander('left_arm')
right_arm_group = moveit_commander.MoveGroupCommander('right_arm')

# curr_pose = left_arm_group.get_current_pose()
end_pose = right_arm_group.get_random_pose()

def set_pose(data):
    xyz = data.data[1:-1].split(", ")

    end_pose.pose.position.x = float(xyz[0])
    end_pose.pose.position.y = float(xyz[1])
    end_pose.pose.position.z = float(xyz[2])

    end_pose.pose.orientation.x = 0.285631307871
    end_pose.pose.orientation.y = 0.374640777506
    end_pose.pose.orientation.z = 0.0634652064499
    end_pose.pose.orientation.w = 0.87979043605

    print end_pose

    right_arm_group.set_start_state_to_current_state()
    right_arm_group.set_pose_target(end_pose)

    plan = right_arm_group.plan()
    right_arm_group.execute(plan)

rospy.Subscriber("move_arm", String, set_pose)

rospy.spin()

#end_pose.pose.position.x = 1.04361591749
#end_pose.pose.position.y = -0.403503688118
#end_pose.pose.position.z = 0.992427517419

