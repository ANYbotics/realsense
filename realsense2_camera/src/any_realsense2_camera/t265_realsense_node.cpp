#include "any_realsense2_camera/t265_realsense_node.h"

using namespace any_realsense2_msgs;
using namespace realsense2_camera;

T265RealsenseNode::T265RealsenseNode(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle, rs2::device dev,
                                     const std::string& serial_no, std::shared_ptr<StateCollector> stateCollector)
    : BaseRealSenseNode(nodeHandle, privateNodeHandle, dev, serial_no, stateCollector),
      _wo_snr(dev.first<rs2::wheel_odometer>()),
      _use_odom_in(false) {
  _monitor_options = {RS2_OPTION_ASIC_TEMPERATURE, RS2_OPTION_MOTION_MODULE_TEMPERATURE};
  initializeOdometryInput();
  handleWarning();
}

void T265RealsenseNode::initializeOdometryInput() {
  std::string calib_odom_file;
  _pnh.param("calib_odom_file", calib_odom_file, std::string(""));
  if (calib_odom_file.empty()) {
    ROS_INFO("No calib_odom_file. No input odometry accepted.");
    return;
  }
  std::ifstream calibrationFile(calib_odom_file);
  if (!calibrationFile) {
    ROS_FATAL_STREAM("calibration_odometry file not found. calib_odom_file = " << calib_odom_file);
    throw std::runtime_error("calibration_odometry file not found");
  }
  const std::string json_str((std::istreambuf_iterator<char>(calibrationFile)), std::istreambuf_iterator<char>());
  const std::vector<uint8_t> wo_calib(json_str.begin(), json_str.end());

  if (!_wo_snr.load_wheel_odometery_config(wo_calib)) {
    ROS_FATAL_STREAM("Format error in calibration_odometry file: " << calib_odom_file);
    throw std::runtime_error("Format error in calibration_odometry file");
  }
  _use_odom_in = true;
}

void T265RealsenseNode::toggleSensors(bool enabled) {
  ROS_WARN_STREAM("toggleSensors method not implemented for T265");
}

void T265RealsenseNode::publishTopics() {
  BaseRealSenseNode::publishTopics();
  setupSubscribers();
}

void T265RealsenseNode::handleWarning() {
  rs2::log_to_callback(rs2_log_severity::RS2_LOG_SEVERITY_WARN, [&](rs2_log_severity severity, rs2::log_message const& msg) noexcept {
    _T265_fault = msg.raw();
    std::array<std::string, 2> list_of_fault{"SLAM_ERROR", "Stream transfer failed, exiting"};
    auto it = std::find_if(begin(list_of_fault), end(list_of_fault),
                           [&](const std::string& s) { return _T265_fault.find(s) != std::string::npos; });
    if (it != end(list_of_fault)) {
      callback_updater.add("Warning ", this, &T265RealsenseNode::warningDiagnostic);
      callback_updater.force_update();
    }
  });
}

void T265RealsenseNode::setupSubscribers() {
  if (!_use_odom_in) return;

  std::string topic_odom_in;
  _pnh.param("topic_odom_in", topic_odom_in, DEFAULT_TOPIC_ODOM_IN);
  ROS_INFO_STREAM("Subscribing to in_odom topic: " << topic_odom_in);

  _odom_subscriber = _node_handle.subscribe(topic_odom_in, 1, &T265RealsenseNode::odom_in_callback, this);
}

void T265RealsenseNode::odom_in_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  ROS_DEBUG("Got in_odom message");
  rs2_vector velocity{-(float)(msg->twist.twist.linear.y), (float)(msg->twist.twist.linear.z), -(float)(msg->twist.twist.linear.x)};

  ROS_DEBUG_STREAM("Add odom: " << velocity.x << ", " << velocity.y << ", " << velocity.z);
  _wo_snr.send_wheel_odometry(0, 0, velocity);
}

void T265RealsenseNode::calcAndPublishStaticTransform(const stream_index_pair& stream, const rs2::stream_profile& base_profile) {
  // Transform base to stream
  tf::Quaternion quaternion_optical;
  quaternion_optical.setRPY(M_PI / 2, 0.0, -M_PI / 2);  // Pose To ROS
  float3 zero_trans{0, 0, 0};

  ros::Time transform_ts_ = ros::Time::now();

  rs2_extrinsics ex;
  try {
    ex = getAProfile(stream).get_extrinsics_to(base_profile);
  } catch (std::exception& e) {
    if (!strcmp(e.what(), "Requested extrinsics are not available!")) {
      ROS_WARN_STREAM(e.what() << " : using unity as default.");
      ex = rs2_extrinsics({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    } else {
      throw e;
    }
  }

  auto Q = rotationMatrixToQuaternion(ex.rotation);
  Q = quaternion_optical * Q * quaternion_optical.inverse();
  float3 trans{ex.translation[0], ex.translation[1], ex.translation[2]};
  if (stream == POSE) {
    Q = Q.inverse();
    publish_static_tf(transform_ts_, trans, Q, _frame_id[stream], _base_frame_id);
  } else {
    publish_static_tf(transform_ts_, trans, Q, _base_frame_id, _frame_id[stream]);
    publish_static_tf(transform_ts_, zero_trans, quaternion_optical, _frame_id[stream], _optical_frame_id[stream]);

    // Add align_depth_to if exist:
    if (_align_depth && _depth_aligned_frame_id.find(stream) != _depth_aligned_frame_id.end()) {
      publish_static_tf(transform_ts_, trans, Q, _base_frame_id, _depth_aligned_frame_id[stream]);
      publish_static_tf(transform_ts_, zero_trans, quaternion_optical, _depth_aligned_frame_id[stream], _optical_frame_id[stream]);
    }
  }
}

void T265RealsenseNode::warningDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status) {
  status.summary(diagnostic_msgs::DiagnosticStatus::WARN, _T265_fault);
}
