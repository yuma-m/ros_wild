# -*- coding: utf-8 -*-

import rospy
import rostopic

from .topic import Topic


class Subscriber(Topic):
    u""" Wildcard subscriber class

    :type _topics: dict[str, rospy.Subscriber]
    """

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

    def register_callback(self, data_class, callback, callback_args=None):
        for topic, subscriber in self._topics.items():
            if rostopic.get_topic_class(topic)[0] == data_class:
                subscriber.impl.add_callback(callback, callback_args)
                subscriber.callback = callback
                subscriber.callback_args = callback_args

    @property
    def topics_with_callback(self):
        topics = []
        for topic, subscriber in self._topics.items():
            if subscriber.callback is not None:
                topics.append(topic)
        return topics

    @property
    def topics_without_callback(self):
        topics = []
        for topic, subscriber in self._topics.items():
            if subscriber.callback is None:
                topics.append(topic)
        return topics


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
