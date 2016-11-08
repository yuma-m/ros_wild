cmake_minimum_required(VERSION 2.8.3)
project(ros_wild)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
)

###########
## Build ##
###########


#############
## Install ##
#############


#############
## Testing ##
#############

catkin_add_nosetests(test)
