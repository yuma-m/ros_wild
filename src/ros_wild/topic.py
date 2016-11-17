# -*- coding: utf-8 -*-

import re
import socket

import rosgraph
import rospy
import rostopic
from rostopic import ROSTopicIOException


class Topic(object):
    u""" Super class for wildcard subscriber / publisher

    :type _topics: dict[str, (rospy.Subscriber|rospy.Publisher)]
    """

    def __init__(self, pub_or_sub, name_regex, *args, **kwargs):
        u""" Constructor of wildcard subscriber

        :param str name_regex: Regular expression of topic name
        """
        self._pub_or_sub = pub_or_sub
        self._name_regex = name_regex
        self._args = args
        self._kwargs = kwargs
        self._topics = {}

        topics = self._get_all_topics()
        for topic in topics:
            if re.match(name_regex, topic):
                self._call_pub_or_sub(topic)

    def reload_topics(self):
        topics = self._get_all_topics()
        for topic in topics:
            if topic not in self._topics and re.match(self._name_regex, topic):
                self._call_pub_or_sub(topic)

    def _get_all_topics(self):
        master = rosgraph.Master('/rostopic')
        try:
            pubs, subs, _ = master.getSystemState()
        except socket.error:
            raise ROSTopicIOException("Unable to communicate with master!")
        else:
            topics = list(set([t for t, _ in pubs] + [t for t, _ in subs]))
            return sorted(topics)

    def _call_pub_or_sub(self, topic):
        data_class = rostopic.get_topic_class(topic)[0]
        pub_or_sub = self._pub_or_sub(topic, data_class, *self._args, **self._kwargs)
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
