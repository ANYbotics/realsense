cmake_minimum_required(VERSION 3.5.1)
project(any_realsense2_description)

find_package(catkin REQUIRED COMPONENTS
    )

# RealSense description
catkin_package(
    )

# Install files
install(DIRECTORY launch meshes rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()