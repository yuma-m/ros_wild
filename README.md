# ros_wild

Wrapper of ROS Subscriber to subscribe to multiple topics.

## Usage

Example scripts are placed in example directory.

### Command line

#### Echo multiple topics

```bash
$ rosrun ros_wild echo ".*_sensor" sensor_msgs/Range
---
topic: /ultrasound_sensor

header: 
  seq: 1
  stamp: 
    secs: 1478776655
    nsecs:    143582
  frame_id: ultrasound_sensor
radiation_type: 0
field_of_view: 0.569999992847
min_range: 0.20000000298
max_range: 3.0
range: 0.600000023842
---
topic: /infrared_sensor

header: 
  seq: 2
  stamp: 
    secs: 1478776656
    nsecs:    998511
  frame_id: infrared_sensor
radiation_type: 1
field_of_view: 0.00999999977648
min_range: 0.00999999977648
max_range: 1.0
range: 0.20000000298
```

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
