#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import numpy as np
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension


class TopicTf(object):
    def __init__(self):
        self._line_sub = rospy.Subscriber('line_data', Int32MultiArray, self.line_callback, queue_size=1)
        self._steering_pub = rospy.Publisher('steering', Float32, queue_size=1)


    def line_callback(self, line_msg):
        data = line_msg.data
        print data
        steering = Float32();
        steering.data = float(0 * 180./3.14)
        self._steering_pub.publish(steering)

if __name__ == '__main__':
    rospy.init_node('topictf')
    topictf = TopicTf()
    rospy.spin()
