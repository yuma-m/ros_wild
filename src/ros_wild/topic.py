# -*- coding: utf-8 -*-

import re

import rospy
import rostopic


class Topic(object):
    u""" Super class for wildcard subscriber / publisher

    :type _topics: dict[str, (rospy.Subscriber|rospy.Publisher)]
    """

    def __init__(self, pub_or_sub, name_regex, data_class, *args, **kwargs):
        u""" Constructor of wildcard subscriber

        :param str name_regex: Regular expression of topic name
        """
        self._pub_or_sub = pub_or_sub
        self._name_regex = name_regex
        self._data_class = data_class
        self._args = args
        self._kwargs = kwargs
        self._topics = {}

        topics = rostopic.find_by_type(data_class._type)
        for topic in topics:
            if re.match(name_regex, topic):
                pub_or_sub = self._pub_or_sub(topic, data_class, *args, **kwargs)
                self._topics[topic] = pub_or_sub

    def reload_topics(self):
        topics = rostopic.find_by_type(self._data_class._type)
        for topic in topics:
            if topic not in self._topics and re.match(self._name_regex, topic):
                pub_or_sub = self._pub_or_sub(topic, self._data_class, *self._args, **self._kwargs)
                self._topics[topic] = pub_or_sub

    def unregister(self, topic_name):
        u""" unregister from a topic

        :param str topic_name: a topic to unregister from
        """
        if topic_name in self._topics:
            topic = self._topics.pop(topic_name)
            topic.unregister()
        else:
            rospy.logwarn("topic {} is not in subscribed topics".format(topic_name))

    def unregister_all(self):
        u""" unregister from all subscribed / published topics """
        for topic in self._topics.values():
            topic.unregister()
        self._topics.clear()