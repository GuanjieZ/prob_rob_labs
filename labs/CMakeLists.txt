cmake_minimum_required(VERSION 2.8.3)
project(prob_rob_labs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  opencv_apps
  roslaunch
  message_generation
  std_msgs
)

## Python setup
catkin_python_setup()

## Add the custom message
add_message_files(
  DIRECTORY ../msg
  FILES
  SatelliteState.msg
)

## Generate messages after Python setup
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare the catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Check launch files
roslaunch_add_file_check(launch/jackal_and_door.launch)

## Install directories
install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install Python scripts
catkin_install_python(PROGRAMS bin/satellite_mmt_pred
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS
  bin/ekf_tf
  bin/ekf_landmark_loc
  bin/satellite_simu
  bin/corresp_matcher
  bin/measurement_predictor
  bin/calc_error
  bin/odom_tracking
  bin/state
  bin/cond_prob
  bin/opendoor_movebot
  bin/flaky_door_opener
  bin/image_mean_feature_x
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

