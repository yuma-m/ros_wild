# -*- coding: utf-8 -*-

from nose.tools import assert_true
from rosgraph_msgs.msg import Log
from std_msgs.msg import Empty

from ros_wild import Subscriber


def callback(msg):
    pass


def test_create_subscriber():
    subscriber = Subscriber(".+")
    topics = subscriber.subscribed_topics
    assert_true("/rosout" in topics)
    assert_true("/rosout_agg" in topics)


def test_no_callback():
    subscriber = Subscriber("/rosout.*")
    subscriber.register_callback(Empty, callback)
    topics = subscriber.topics_without_callback
    assert_true("/rosout" in topics)
    assert_true("/rosout_agg" in topics)


def test_register_callback():
    subscriber = Subscriber("/rosout.*")
    subscriber.register_callback(Log, callback)
    topics = subscriber.topics_with_callback
    assert_true("/rosout" in topics)
    assert_true("/rosout_agg" in topics)
