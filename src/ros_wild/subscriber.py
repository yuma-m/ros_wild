# -*- coding: utf-8 -*-

import rospy

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


class EchoSubscriber(Subscriber):
    u""" Wildcard subscriber to echo topics """
    def __init__(self, name_regex, data_class, *args, **kwargs):
        u""" Constructor of wildcard subscriber

        :param str name_regex: Regular expression of topic name
        """
        super(EchoSubscriber, self).__init__(name_regex, data_class, *args, **kwargs)

    def _call_pub_or_sub(self, topic):
        pub_or_sub = self._pub_or_sub(topic, self._data_class, *self._args, callback_args=topic, **self._kwargs)
        self._topics[topic] = pub_or_sub
