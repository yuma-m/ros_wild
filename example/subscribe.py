# -*- coding: utf-8 -*-

import rospy

from ros_wild import Subscriber


def callback(msg):
    print(msg.msg)


def main():
    rospy.init_node("example_node")
    sub = Subscriber(r"/rosout.*", callback)
    rospy.spin()

if __name__ == '__main__':
    main()
