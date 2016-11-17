# -*- coding: utf-8 -*-

import rospy
from ros_wild import Publisher
from rosgraph_msgs.msg import Log


def main():
    rospy.init_node("test")
    pub = Publisher(".+", queue_size=1)
    pub.publish(Log(msg="this is test message"))

if __name__ == '__main__':
    main()
