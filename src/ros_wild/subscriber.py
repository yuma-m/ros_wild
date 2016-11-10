# -*- coding: utf-8 -*-

import re

import rospy
import rostopic

from .topic import Topic


class Subscriber(Topic):
    u""" Wildcard subscriber class """

    def __init__(self, name_regex, data_class, *args, **kwargs):
        u""" Constructor of wildcard subscriber

        :param str name_regex: Regular expression of topic name
        """
        super(Subscriber, self).__init__(rospy.Subscriber, name_regex, data_class, *args, **kwargs)

    def __repr__(self):
        return "<ros_wild.Subscriber: {}>".format(self._name_regex)

    @property
    def subscribed_topics(self):
        return self._topics.keys()
