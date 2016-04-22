#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def send_command():
    pub = rospy.Publisher('speech_command', String, queue_size=10)
    rospy.init_node('speech_command', anonymous=True)
    block_letters = "A"
    #block_letters = "CAB"
    rospy.loginfo(block_letters)
    pub.publish(block_letters)

if __name__ == '__main__':
    try:
        send_command()
    except rospy.ROSInterruptException:
        pass
