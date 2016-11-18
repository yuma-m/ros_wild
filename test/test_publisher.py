# -*- coding: utf-8 -*-

from nose.tools import assert_true

from ros_wild import Publisher


def test_create_publisher():
    publisher = Publisher(".+", queue_size=1)
    topics = publisher.published_topics
    assert_true("/rosout" in topics)
    assert_true("/rosout_agg" in topics)
