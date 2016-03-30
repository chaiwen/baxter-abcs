#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def send_command():
    pub = rospy.Publisher('speech_command', String, queue_size=10)
    rospy.init_node('speech_command', anonymous=True)
    hello_str = "Hello"
    rospy.loginfo(hello_str)
    pub.publish(hello_str)

if __name__ == '__main__':
    try:
        send_command()
    except rospy.ROSInterruptException:
        pass
