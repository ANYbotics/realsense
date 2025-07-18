cmake_minimum_required(VERSION 3.5.1)
project(any_realsense2_camera)
add_compile_options(-std=c++17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
    COMPONENTS
        acl_fault_propagation
        depth_camera_faults
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
find_package(PkgConfig REQUIRED)
pkg_check_modules(jsoncpp jsoncpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

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
    ${jsoncpp_INCLUDE_DIRS}
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
        depth_camera_faults
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
    DEPENDS
        jsoncpp
        Boost
    )

add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/constants.h
  include/${PROJECT_NAME}/realsense_node_factory.h
  include/${PROJECT_NAME}/base_realsense_node.h
  include/${PROJECT_NAME}/t265_realsense_node.h
  src/${PROJECT_NAME}/realsense_node_factory.cpp
  src/${PROJECT_NAME}/base_realsense_node.cpp
  src/${PROJECT_NAME}/t265_realsense_node.cpp
  src/${PROJECT_NAME}/realsense_self_calibration.cpp
    )

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC ${Boost_INCLUDE_DIRS}
  PRIVATE ${realsense_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${jsoncpp_LIBRARIES}
    ${Boost_LIBRARIES}
    pthread
    )

# Install nodelet library
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
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
    find_package(catkin REQUIRED
        COMPONENTS
            acl_fault_propagation
            depth_camera_faults
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
    endif()
endif()

#################
## Clang_tools ##
#################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling(
    DISABLE_CLANG_FORMAT
  )
endif(cmake_clang_tools_FOUND)
