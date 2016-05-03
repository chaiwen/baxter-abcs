#!/usr/bin/env python
import roslib
import sys
import rospy
import math

import tf
import geometry_msgs.msg

import baxter_interface

from vision.srv import *
from std_msgs.msg import String

rospy.init_node('control', anonymous=False)
# create instance of Limb class
limb = baxter_interface.Limb('right')

angles = {}
blocks = []
init_placement = (5,5,100)


# client to get XYZ from letter of block alphabet
def get_xyz_client(abc):
    rospy.wait_for_service('get_xyz_from_abc')
    try:
        get_xyz_from_abc = rospy.ServiceProxy('get_xyz_from_abc', GetXYZFromABC)
        resp1 = get_xyz_from_abc(abc)
        return resp1.x, resp1.y, resp1.z
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


# this is where we will separate the thing to be spelled into a list of locations, to be picked up in that order
def callback(data):
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#    wave()
		separated = list(data.data)
		print separated
		for l in separated:
				tup = get_letter_position(l) 
				#print 'callback tuple: '
				#print tup
				blocks.append(tup) 

		# print "the final blocks in order" 
		# print blocks

		for block in blocks:
			print block
			place_block(block)
			

# 3 item tuple in format ( x, y, z)
def get_letter_position(letter):
		print 'Requesting position of: ' + letter + '\n'
		x, y, z = get_xyz_client(letter)
		return (x,y,z) 


def place_block(block_tuple):
		# (x, y, z) 
		#
    move_pub = rospy.Publisher("move_arm", String, queue_size=10)
    rospy.loginfo(str(block_tuple) + "\n")
    move_pub.publish(str(block_tuple))
    
    print "yea baxter is moving the block with initial location: x=" + str(block_tuple[0]) + " y=" + str(block_tuple[1]) + " z=" + str(block_tuple[2])

def wave():
    # store first and second wave position
    wave_1 = {'right_s0': -0.459, 'right_s1': -0.202, 'right_e0': 1.807, 'right_e1': 1.714, 'right_w0': -0.906, 'right_w1': -1.545, 'right_w2': -0.276}
    wave_2 = {'right_s0': -0.395, 'right_s1': -0.202, 'right_e0': 1.831, 'right_e1': 1.981, 'right_w0': -1.979, 'right_w1': -1.100, 'right_w2': -0.448}

    for _move in range(3):
        limb.move_to_joint_positions(wave_1)
        limb.move_to_joint_positions(wave_2)


def control():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    # get current joint angles
    angles = limb.joint_angles()
    angles['right_s0']=0.0
    angles['right_s1']=0.0
    angles['right_e0']=0.0
    angles['right_e1']=0.0
    angles['right_w0']=0.0
    angles['right_w1']=0.0
    angles['right_w2']=0.0

    limb.move_to_joint_positions(angles)

    rospy.Subscriber("speech_command", String, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    
    x,y,z = get_letter_position('b')
    print x, y, z
    
    listener = tf.TransformListener()
    listener.waitForTransform("/kinect_mount_optical_frame", "/world", rospy.Time(), rospy.Duration(4.0))

    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform("/world", "/kinect_mount_optical_frame", now, rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform("/world", "/kinect_mount_optical_frame", now)
            
            print trans
            print rot


            
            
        except:
            print "hello"

#control() # this will subscribe and wave robot arm?

#abc = "BLOCK A!"
#print "Requesting position of %s"%(abc)
#print "%s = %d %d %d"%(abc, x, y, z)
