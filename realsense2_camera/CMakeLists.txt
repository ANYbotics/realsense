cmake_minimum_required(VERSION 3.5.1)
project(any_realsense2_camera)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
        nav_msgs
        roscpp
        sensor_msgs
        std_msgs
        any_librealsense2
        any_realsense2_msgs
        nodelet
        cv_bridge
        image_transport
        tf
        ddynamic_reconfigure
        diagnostic_updater
        std_srvs
    )
    
find_package(Eigen3 REQUIRED)

if(BUILD_WITH_OPENMP)
    find_package(OpenMP)
    if(NOT OpenMP_FOUND)
        message(FATAL_ERROR "\n\n OpenMP is missing!\n\n")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endif()
endif()

if(SET_USER_BREAK_AT_STARTUP)
	message("GOT FLAG IN CmakeLists.txt")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBPDEBUG")
endif()

# find_package(realsense2 2.29.0)
# if(NOT realsense2_FOUND)
#     message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
# endif()

if (CMAKE_BUILD_TYPE EQUAL "RELEASE")
    message(STATUS "Create Release Build.")
    set(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")
else()
    message(STATUS "Create Debug Build.")
endif()

set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")


set(CMAKE_NO_SYSTEM_FROM_IMPORTED true)
include_directories(
    include
    SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
    )

# RealSense ROS Node
catkin_package(
    INCLUDE_DIRS 
        include
        ${EIGEN3_INCLUDE_DIR}
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        sensor_msgs
        std_msgs
        any_librealsense2
        any_realsense2_msgs
        nodelet
        cv_bridge
        image_transport
        ddynamic_reconfigure
        nav_msgs
        std_srvs
    )

add_library(${PROJECT_NAME}
    include/constants.h
    include/realsense_node_factory.h
    include/base_realsense_node.h
    include/t265_realsense_node.h
    src/realsense_node_factory.cpp
    src/base_realsense_node.cpp
    src/t265_realsense_node.cpp
    )

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_include_directories(${PROJECT_NAME}
  PRIVATE ${realsense_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    )

# Install nodelet library
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

# Install header files
install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

# Install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )

# Install rviz files
install(DIRECTORY rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    )

# Install xml files
install(FILES nodelet_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

##############
##   Test   ##
##############
if(CATKIN_ENABLE_TESTING)
catkin_add_gtest(test_${PROJECT_NAME}
  test/empty_test.cpp
)

target_include_directories(test_${PROJECT_NAME}
  PRIVATE
    include
  SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(test_${PROJECT_NAME}
  gtest_main
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

###################
## Code_coverage ##
###################
find_package(cmake_code_coverage QUIET)
if(cmake_code_coverage_FOUND)
  add_gtest_coverage(
    TEST_BUILD_TARGETS
      test_${PROJECT_NAME}
  )
endif(cmake_code_coverage_FOUND)
endif()