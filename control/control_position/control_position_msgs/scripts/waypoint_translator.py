#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from control_position_msgs.msg import ControllerReference
from math import *

pub = rospy.Publisher('ref_pose', PoseStamped, queue_size=1)

def callback(data):
    pose = PoseStamped()
    pose.header = data.header
    pose.pose.position.x = data.position.x
    pose.pose.position.y = data.position.y
    pose.pose.position.z = data.position.z
    pose.pose.orientation.x = 0 
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = sin(0.5*data.yaw)
    pose.pose.orientation.w = cos(0.5*data.yaw)
    pub.publish(pose)
    
def translator():

    rospy.init_node('translator', anonymous=True)

    rospy.Subscriber('reference', ControllerReference, callback)

    rospy.spin()

if __name__ == '__main__':
    translator()
