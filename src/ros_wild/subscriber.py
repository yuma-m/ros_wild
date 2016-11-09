# -*- coding: utf-8 -*-

import re

import rospy
import rostopic


class Subscriber(object):
    u""" Wildcard subscriber

    :type _subscribers: dict[str, rospy.Subscriber]
    """

    def __init__(self, name_regex, data_class, *args, **kwargs):
        u""" Constructor of wildcard subscriber

        :param str name_regex: Regular expression of topic name
        """
        self._name_regex = name_regex
        self._subscribers = {}

        topics = rostopic.find_by_type(data_class._type)
        for topic in topics:
            if re.match(name_regex, topic):
                subscriber = rospy.Subscriber(topic, data_class, *args, **kwargs)
                self._subscribers[topic] = subscriber

    def __repr__(self):
        return "<ros_wild.Subscriber: {}>".format(self._name_regex)

    @property
    def subscribed_topics(self):
        return self._subscribers.keys()

    def reload_topics(self):
        raise NotImplementedError()

    def unregister(self, topic_name):
        u""" unsubscribe from a topic

        :param str topic_name: A topic to unsubscribe from
        """
        if topic_name in self._subscribers:
            subscriber = self._subscribers.pop(topic_name)
            subscriber.unregister()
        else:
            rospy.logwarn("topic {} is not in subscribed topics".format(topic_name))

    def unregister_all(self):
        u""" unsubscribe from all subscribed topics """
        for subscriber in self._subscribers.values():
            subscriber.unregister()
        self._subscribers.clear()
