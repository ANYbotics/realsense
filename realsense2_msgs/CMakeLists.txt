cmake_minimum_required(VERSION 3.16.3)
project(any_realsense2_msgs)

if("$ENV{ROS_VERSION}" STREQUAL "1")

set(CATKIN_PACKAGE_DEPENDENCIES
    std_msgs
)

find_package(catkin REQUIRED
    COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        message_generation
)


add_message_files(
    FILES
        IMUInfo.msg
        Extrinsics.msg
        FrameMetadataMsg.msg
        TimeOffsetsMsg.msg
        TimestampingInfoMsg.msg
)

add_service_files(
    FILES
        LoadJsonFile.srv
)
generate_messages(
    DEPENDENCIES
        std_msgs
)

catkin_package(
    CATKIN_DEPENDS
      ${CATKIN_PACKAGE_DEPENDENCIES}
      message_runtime
)

else() # ROS version 2

set(PACKAGE_DEPENDENCIES
  std_msgs
  )
  
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

foreach(PKG IN LISTS PACKAGE_DEPENDENCIES)
  find_package(${PKG} REQUIRED)
endforeach()

set(MSG_FILES
  "msg/IMUInfo.msg"
  "msg/Extrinsics.msg"
  "msg/FrameMetadataMsg.msg"
  "msg/TimeOffsetsMsg.msg"
  "msg/TimestampingInfoMsg.msg"
  "srv/LoadJsonFile.srv"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
    ${MSG_FILES}
  DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
  )

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()

endif()
