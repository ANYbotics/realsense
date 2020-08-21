cmake_minimum_required(VERSION 3.5.1)
project(any_realsense2_msgs)


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

generate_messages(
    DEPENDENCIES
        std_msgs
)

catkin_package(
    CATKIN_DEPENDS
      ${CATKIN_PACKAGE_DEPENDENCIES}
      message_runtime
)
