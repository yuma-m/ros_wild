# -*- coding: utf-8 -*-

import rospy

from .topic import Topic


class Subscriber(Topic):
    u""" Wildcard subscriber class """

    def __init__(self, name_regex, *args, **kwargs):
        u""" Constructor of wildcard subscriber

        :param str name_regex: Regular expression of topic name
        """
        super(Subscriber, self).__init__(rospy.Subscriber, name_regex, *args, **kwargs)

    def __repr__(self):
        return "<ros_wild.Subscriber: {}>".format(self._name_regex)

    @property
    def subscribed_topics(self):
        return sorted(self._topics.keys())


class EchoSubscriber(Subscriber):
    u""" Wildcard subscriber to echo topics """
    def __init__(self, name_regex, *args, **kwargs):
        u""" Constructor of wildcard subscriber

        :param str name_regex: Regular expression of topic name
        """
        super(EchoSubscriber, self).__init__(name_regex, *args, **kwargs)

    def _call_pub_or_sub(self, topic):
        self._kwargs["callback_args"] = topic
        super(EchoSubscriber, self)._call_pub_or_sub(topic)
