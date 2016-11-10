# -*- coding: utf-8 -*-

import rospy

from .topic import Topic


class Publisher(Topic):
    u""" Wildcard publisher class """

    def __init__(self, name_regex, data_class, *args, **kwargs):
        u""" Constructor of wildcard publisher

        :param str name_regex: Regular expression of topic name
        """
        super(Publisher, self).__init__(rospy.Publisher, name_regex, data_class, *args, **kwargs)

    def __repr__(self):
        return "<ros_wild.Publisher: {}>".format(self._name_regex)

    @property
    def published_topics(self):
        return self._topics.keys()

    def publish(self, msg):
        for publisher in self._topics.values():
            publisher.publish(msg)
