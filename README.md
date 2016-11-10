# ros_wild

Wrapper of ROS Subscriber to subscribe to multiple topics.

## Usage

Example scripts are placed in example directory.

### Python

#### Wildcard Subscriber

```python
>>> from rosgraph_msgs.msg import Log
>>> from ros_wild import Subscriber

# subscribe to all Log topics
>>> sub = Subscriber(r".*", Log)
>>> sub.subscribed_topics
['/rosout', '/rosout_agg']

# unsubscribe from all topics
>>> sub.unregister_all()
>>> sub.subscribed_topics
[]

# resubscribed to topics
>>> sub.reload()
>>> sub.subscribed_topics
['/rosout', '/rosout_agg']
```

#### Wildcard Publisher

```python
>>> import rospy
>>> from rosgraph_msgs.msg import Log
>>> from ros_wild import Publisher

# publish to all Log topics
>>> rospy.init_node("test")
>>> pub = Publisher("/rosout.*", Log, queue_size=1)
>>> pub.publish(Log(msg="this is test message"))
```

## Installation

```bash
$ cd /path/to/your/catkin_ws/src/
$ git clone https://github.com/yuma-m/ros_wild.git
$ cd ../
$ catkin_make
```

## Links

- Author: [Yuma Mihira](http://yurax2.com)
- [ROS](http://www.ros.org/)
