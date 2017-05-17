cmake_minimum_required(VERSION 2.8.3)
project(uchile_skills)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package()
