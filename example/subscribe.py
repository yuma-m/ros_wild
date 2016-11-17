# -*- coding: utf-8 -*-

import rospy
from rosgraph_msgs.msg import Log
from std_msgs.msg import String

from ros_wild import Subscriber


def callback_log(msg):
    u""" callback for /rosout and /rosout_agg """
    print(msg.msg)


def callback_string(msg):
    u""" callback for some string topic """
    print(msg.data)


def main():
    rospy.init_node("example_node")
    sub = Subscriber(r"/.+")
    sub.register_callback(Log, callback_log)
    sub.register_callback(String, callback_string)
    rospy.spin()

if __name__ == '__main__':
    main()
