# -*- coding: utf-8 -*-

import rospy
import rostopic

from .topic import Topic


class Publisher(Topic):
    u""" Wildcard publisher class

    :type _topics: dict[str, rospy.Publisher]
    """

    def __init__(self, name_regex, *args, **kwargs):
        u""" Constructor of wildcard publisher

        :param str name_regex: Regular expression of topic name
        """
        super(Publisher, self).__init__(rospy.Publisher, name_regex, *args, **kwargs)

    def __repr__(self):
        return "<ros_wild.Publisher: {}>".format(self._name_regex)

    @property
    def published_topics(self):
        return sorted(self._topics.keys())

    def publish(self, msg):
        for topic, publisher in self._topics.items():
            if isinstance(msg, rostopic.get_topic_class(topic)[0]):
                publisher.publish(msg)
