# -*- coding: utf-8 -*-

import re

import rospy
import rostopic


class Subscriber(object):
    def __init__(self, name_regex, data_class, *args, **kwargs):
        u""" Constructor of wildcard subscriber

        :param str name_regex: Regular expression of topic name
        """
        self._subscribers = {}

        topics = rostopic.find_by_type(data_class._type)
        for topic in topics:
            if re.match(name_regex, topic):
                subscriber = rospy.Subscriber(topic, data_class, *args, **kwargs)
                self._subscribers[topic] = subscriber

    @property
    def subscribed_topics(self):
        return self._subscribers.keys()
