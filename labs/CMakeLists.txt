cmake_minimum_required(VERSION 2.8.3)
project(prob_rob_labs)

find_package(catkin REQUIRED COMPONENTS
  opencv_apps
  roslaunch
)

catkin_package()

roslaunch_add_file_check(launch/jackal_and_door.launch)

catkin_python_setup()

install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

<<<<<<< HEAD
=======
catkin_install_python(PROGRAMS bin/calc_error
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

>>>>>>> Tuni/main
catkin_install_python(PROGRAMS bin/odom_tracking
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS bin/state
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS bin/cond_prob
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS bin/opendoor_movebot
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS bin/flaky_door_opener
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS bin/image_mean_feature_x
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
