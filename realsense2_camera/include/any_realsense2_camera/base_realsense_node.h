// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once

#include "any_realsense2_camera/realsense_node_factory.h"
#include "any_realsense2_camera/realsense_self_calibration.h"

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <acl_fault_propagation/SensorValueSeverityConverter.hpp>
#include <acl_fault_propagation/StateCollector.hpp>

/** Custom messages and services **/
#include <any_realsense2_msgs/FrameMetadataMsg.h>
#include <any_realsense2_msgs/LoadJsonFile.h>
#include <any_realsense2_msgs/TimeOffsetsMsg.h>
#include <any_realsense2_msgs/TimestampingInfoMsg.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
/** Custom messages **/

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>

using namespace any_realsense2_msgs;

namespace realsense2_camera {
//* Frame metadata container.
struct FrameMetadata {
  //* Timestamps.
  rs2_metadata_type sensor_capture_timestamp = 0;
  rs2_metadata_type frame_processing_timestamp = 0;
  rs2_metadata_type kernel_arrival_timestamp = 0;
  rs2_metadata_type driver_arrival_timestamp = 0;

  //* Frame generation.
  rs2_metadata_type gain_level = 0;
  rs2_metadata_type auto_exposure = 0;
  rs2_metadata_type exposure_time = 0;

  //* Frame counter.
  rs2_metadata_type frame_counter = 0;
  rs2_metadata_type actual_fps = 0;

  //* Laser projector status.
  rs2_metadata_type laser_enabled = 0;
  rs2_metadata_type laser_power = 0;

  //* Physical state of the device.
  rs2_metadata_type temperature = 0;

  FrameMetadataMsg toRosMsg() const {
    FrameMetadataMsg msg;

    //* Timestamps.
    msg.sensor_capture_timestamp = sensor_capture_timestamp;
    msg.frame_processing_timestamp = frame_processing_timestamp;
    msg.kernel_arrival_timestamp = kernel_arrival_timestamp;
    msg.driver_arrival_timestamp = driver_arrival_timestamp;
    //* Frame generation.
    msg.gain_level = gain_level;
    msg.auto_exposure = auto_exposure;
    msg.exposure_time = exposure_time;
    //* Frame counter.
    msg.frame_counter = frame_counter;
    msg.actual_fps = actual_fps;
    //* Laser projector.
    msg.laser_enabled = laser_enabled;
    msg.laser_power = laser_power;
    //* Physical state.
    msg.temperature = temperature;

    return msg;
  }
};

//* Time offsets container.
struct TimeOffsets {
  //* Offset introduced by the delay between frame capture, processing and start of transmission.
  double frame_acquisition_offset = 0;

  //* Offset introduced during USB transmission.
  double wire_transmission_offset = 0;

  //* Offset due to kernel-user space transition.
  double driver_handover_offset = 0;

  TimeOffsetsMsg toRosMsg() const {
    TimeOffsetsMsg msg;

    //* Timestamps.
    msg.frame_acquisition_offset = frame_acquisition_offset;
    msg.wire_transmission_offset = wire_transmission_offset;
    msg.driver_handover_offset = driver_handover_offset;

    return msg;
  }
};

struct FrequencyDiagnostics {
  FrequencyDiagnostics(double expected_frequency, std::string name, std::string hardware_id)
      : expected_frequency_(expected_frequency),
        frequency_status_(diagnostic_updater::FrequencyStatusParam(&expected_frequency_, &expected_frequency_)),
        diagnostic_updater_(ros::NodeHandle(), ros::NodeHandle("~"), ros::this_node::getName() + "_" + name) {
    ROS_DEBUG("Expected frequency for %s = %.5f", name.c_str(), expected_frequency_);
    diagnostic_updater_.setHardwareID(hardware_id);
    diagnostic_updater_.add(frequency_status_);
  }

  void tick() { frequency_status_.tick(); }

  void update() { diagnostic_updater_.update(); }

  void set_status_callback(std::function<void(const diagnostic_updater::DiagnosticStatusWrapper& stat)> callback) {
    frequency_status_.callback = callback;
  }

  double expected_frequency_;

  struct FrequencyStatusWithCallback : public diagnostic_updater::FrequencyStatus {
    FrequencyStatusWithCallback(const diagnostic_updater::FrequencyStatusParam& params) : FrequencyStatus(params) {}
    virtual void run(diagnostic_updater::DiagnosticStatusWrapper& stat) {
      diagnostic_updater::FrequencyStatus::run(stat);
      if (callback) {
        callback(stat);
      }
    }
    std::function<void(const diagnostic_updater::DiagnosticStatusWrapper& stat)> callback = nullptr;
  } frequency_status_;

  diagnostic_updater::Updater diagnostic_updater_;
};

typedef std::pair<image_transport::Publisher, std::shared_ptr<FrequencyDiagnostics>> ImagePublisherWithFrequencyDiagnostics;

class TemperatureDiagnostics {
 public:
  TemperatureDiagnostics(std::string name, std::string serial_no);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status);

  void update(double crnt_temperaure) {
    _crnt_temp = crnt_temperaure;
    _updater.update();
  }

 private:
  double _crnt_temp;
  diagnostic_updater::Updater _updater;
};

// Start of custom ANYbotics code
class IREmitterDiagnostics {
 public:
  IREmitterDiagnostics(std::string serial_no);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status);

  void update(bool laser_enabled, long long laser_power) {
    _laser_enabled = laser_enabled;
    _laser_power = laser_power;
    _updater.update();
  }

 private:
  bool _laser_enabled;
  long long _laser_power;
  diagnostic_updater::Updater _updater;
};
// End of custom ANYbotics code

class NamedFilter {
 public:
  std::string _name;
  std::shared_ptr<rs2::filter> _filter;

 public:
  NamedFilter(std::string name, std::shared_ptr<rs2::filter> filter) : _name(name), _filter(filter) {}
};

class PipelineSyncer : public rs2::asynchronous_syncer {
 public:
  void operator()(rs2::frame f) const { invoke(std::move(f)); }
};

class SyncedImuPublisher {
 public:
  SyncedImuPublisher() { _is_enabled = false; };
  SyncedImuPublisher(ros::Publisher imu_publisher, std::size_t waiting_list_size = 1000);
  ~SyncedImuPublisher();
  void Pause();                        // Pause sending messages. All messages from now on are saved in queue.
  void Resume();                       // Send all pending messages and allow sending future messages.
  void Publish(sensor_msgs::Imu msg);  // either send or hold message.
  uint32_t getNumSubscribers() { return _publisher.getNumSubscribers(); };
  void Enable(bool is_enabled) { _is_enabled = is_enabled; };

 private:
  void PublishPendingMessages();

 private:
  std::mutex _mutex;
  ros::Publisher _publisher;
  bool _pause_mode;
  std::queue<sensor_msgs::Imu> _pending_messages;
  std::size_t _waiting_list_size;
  bool _is_enabled;
};

class BaseRealSenseNode : public InterfaceRealSenseNode {
 public:
  using StateCollector = acl::fault_propagation::StateCollector;

  BaseRealSenseNode(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle, rs2::device dev, const std::string& serial_no,
                    std::shared_ptr<StateCollector> stateCollector);

  virtual void toggleSensors(bool enabled) override;
  virtual void publishTopics() override;
  virtual void registerDynamicReconfigCb(ros::NodeHandle& nh) override;
  virtual ~BaseRealSenseNode();

 public:
  enum imu_sync_method { NONE, COPY, LINEAR_INTERPOLATION };

 protected:
  class float3 {
   public:
    float x, y, z;

   public:
    float3& operator*=(const float& factor) {
      x *= factor;
      y *= factor;
      z *= factor;
      return (*this);
    }
    float3& operator+=(const float3& other) {
      x += other.x;
      y += other.y;
      z += other.z;
      return (*this);
    }
  };

  // Timestamping methods
  //   Baseline: the default method, that ships with the realsense2 package.
  //   Fixed offset: assumes cameras have a fixed delay between acquisition and
  //                 driver availability, which is used to correct the frame stamps.
  //   Varying offsets: uses frame metadata to estimate frame acquisition (fD) and
  //                    driver transition delays (dD). Assumes fixed transmission
  //                    delay (tD) and removes all of them from frame stamps.
  enum class timestamping_method {
    baseline,
    fixed_offset,
    varying_offsets,
  };

  bool _is_running;
  // Start of custom ANYbotics code
  std::string _frame_id_prefix;
  // End of custom ANYbotics code
  std::string _base_frame_id;
  std::string _odom_frame_id;
  std::map<stream_index_pair, std::string> _frame_id;
  std::map<stream_index_pair, std::string> _optical_frame_id;
  std::map<stream_index_pair, std::string> _depth_aligned_frame_id;
  ros::NodeHandle &_node_handle, _pnh;
  bool _align_depth;
  std::vector<rs2_option> _monitor_options;

  virtual void calcAndPublishStaticTransform(const stream_index_pair& stream, const rs2::stream_profile& base_profile);
  rs2::stream_profile getAProfile(const stream_index_pair& stream);
  tf::Quaternion rotationMatrixToQuaternion(const float rotation[9]) const;
  void publish_static_tf(const ros::Time& t, const float3& trans, const tf::Quaternion& q, const std::string& from, const std::string& to);

 private:
  class CimuData {
   public:
    CimuData() : m_time(-1){};
    CimuData(const stream_index_pair type, Eigen::Vector3d data, double time) : m_type(type), m_data(data), m_time(time){};
    bool is_set() { return m_time > 0; };

   public:
    stream_index_pair m_type;
    Eigen::Vector3d m_data;
    double m_time;
  };

  static std::string getNamespaceStr();
  void getParameters();
  void setupDevice();
  void setupErrorCallback();
  void setupPublishers();
  void enable_devices();
  void setupFilters();
  void setupStreams();
  void setBaseTime(double frame_time, bool warn_no_metadata);
  cv::Mat& fix_depth_scale(const cv::Mat& from_image, cv::Mat& to_image);
  void clip_depth(rs2::depth_frame depth_frame, float clipping_dist);
  void updateStreamCalibData(const rs2::video_stream_profile& video_profile);
  void updateExtrinsicsCalibData(const rs2::video_stream_profile& left_video_profile, const rs2::video_stream_profile& right_video_profile);
  void SetBaseStream();
  void publishStaticTransforms();
  void publishDynamicTransforms();
  void publishIntrinsics();
  void runFirstFrameInitialization(rs2_stream stream_type);
  void publishPointCloud(rs2::points f, const ros::Time& t, const rs2::frameset& frameset);
  Extrinsics rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics, const std::string& frame_id) const;

  IMUInfo getImuInfo(const stream_index_pair& stream_index);
  void publishFrame(rs2::frame f, const ros::Time& t, const stream_index_pair& stream, std::map<stream_index_pair, cv::Mat>& images,
                    const std::map<stream_index_pair, ros::Publisher>& info_publishers,
                    const std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics>& image_publishers,
                    std::map<stream_index_pair, int>& seq, std::map<stream_index_pair, sensor_msgs::CameraInfo>& camera_info,
                    const std::map<stream_index_pair, std::string>& optical_frame_id, const std::map<rs2_stream, std::string>& encoding,
                    bool copy_data_from_frame = true);
  void publishFrameWithoutDiagnostics(rs2::frame f, const ros::Time& t, const stream_index_pair& stream,
                                      std::map<stream_index_pair, cv::Mat>& images,
                                      const std::map<stream_index_pair, ros::Publisher>& info_publishers,
                                      const std::map<stream_index_pair, image_transport::Publisher>& image_publishers,
                                      std::map<stream_index_pair, int>& seq,
                                      std::map<stream_index_pair, sensor_msgs::CameraInfo>& camera_info,
                                      const std::map<stream_index_pair, std::string>& optical_frame_id,
                                      const std::map<rs2_stream, std::string>& encoding, bool copy_data_from_frame = true);
  bool getEnabledProfile(const stream_index_pair& stream_index, rs2::stream_profile& profile);

  void publishAlignedDepthToOthers(rs2::frameset frames, const ros::Time& t);
  sensor_msgs::Imu CreateUnitedMessage(const CimuData accel_data, const CimuData gyro_data);

  void FillImuData_Copy(const CimuData imu_data, std::deque<sensor_msgs::Imu>& imu_msgs);
  void ImuMessage_AddDefaultValues(sensor_msgs::Imu& imu_msg);
  void FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<sensor_msgs::Imu>& imu_msgs);
  void imu_callback(rs2::frame frame);
  void imu_callback_sync(rs2::frame frame, imu_sync_method sync_method = imu_sync_method::COPY);
  void pose_callback(rs2::frame frame);
  void multiple_message_callback(rs2::frame frame, imu_sync_method sync_method);
  void frame_callback(rs2::frame frame);
  void registerDynamicOption(ros::NodeHandle& nh, rs2::options sensor, std::string& module_name);
  void readAndSetDynamicParam(ros::NodeHandle& nh1, std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec,
                              const std::string option_name, const int min_val, const int max_val, rs2::sensor sensor, int* option_value);
  void registerAutoExposureROIOptions(ros::NodeHandle& nh);
  void set_auto_exposure_roi(const std::string option_name, rs2::sensor sensor, int new_value);
  void set_sensor_auto_exposure_roi(rs2::sensor sensor);
  rs2_stream rs2_string_to_stream(std::string str);
  void startMonitoring();
  void temperature_fault_check();
  void publish_temperature();
  void publish_frequency_update();
  //* Custom methods
  void setupServices();
  bool calibration_intrinsic_health_callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool calibration_extrinsic_health_callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool restore_factory_calibration_callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool intrinsic_calibration_callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool extrinsic_calibration_callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool get_current_calibration_callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool toggleColor(bool enabled);
  bool toggleColorCb(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
  bool toggleEmitter(bool enable);
  bool toggleEmitterCb(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
  bool loadJsonFileServiceCb(any_realsense2_msgs::LoadJsonFile::Request& request, any_realsense2_msgs::LoadJsonFile::Response& response);
  void fetchFrameMetadata(const rs2::frame& frame, FrameMetadata& metadata_container);
  void publishTimestampingInformation(const ros::Time& t, const rs2::frame& frame, const FrameMetadata& metadata,
                                      const TimeOffsets& time_offsets);

  //* Custom methods
 public:
  rs2::device _dev;  // exposed to perform hardware resets.

  // LCSM calibration callbacks
  using CalibrationStateCallback = std::function<bool()>;
  void setEnterCalibrationModeCallback(CalibrationStateCallback callback) { calibrationEnterCallback_ = callback; }
  void setExitCalibrationModeCallback(CalibrationStateCallback callback) { calibrationExitCallback_ = callback; }

 private:
  // Helper functions for calibration operations
  std::string formatHealthScoreMessage(float healthScore, const std::string& operation_name, bool success, bool is_health_check = false);

  /**
   * @brief Unified calibration operation handler
   * @param cal_type The type of calibration to perform
   * @param is_health_check Whether this is a health check (true) or actual calibration (false)
   * @return Service response with success status and message
   */
  std_srvs::Trigger::Response performCalibrationOperation(CalibrationType cal_type, bool is_health_check = false);

  std::map<stream_index_pair, rs2::sensor> _sensors;
  std::map<std::string, std::function<void(rs2::frame)>> _sensors_callback;
  std::vector<std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure>> _ddynrec;

  std::string _json_file_path;
  std::string _serial_no;
  float _depth_scale_meters;
  float _clipping_distance;
  bool _allow_no_texture_points;
  bool _ordered_pc;

  double _linear_accel_cov;
  double _angular_velocity_cov;
  bool _hold_back_imu_for_frames;

  std::map<stream_index_pair, rs2_intrinsics> _stream_intrinsics;
  std::map<stream_index_pair, int> _width;
  std::map<stream_index_pair, int> _height;
  std::map<stream_index_pair, int> _fps;
  std::map<rs2_stream, int> _format;
  std::map<stream_index_pair, bool> _enable;
  std::map<rs2_stream, std::string> _stream_name;
  bool _publish_tf;
  double _tf_publish_rate;
  tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;
  tf2_ros::TransformBroadcaster _dynamic_tf_broadcaster;
  std::vector<geometry_msgs::TransformStamped> _static_tf_msgs;
  std::shared_ptr<std::thread> _tf_t;

 public:
  std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics> _image_publishers;  // exposed to access frequency diagnostics
 private:
  std::map<stream_index_pair, ros::Publisher> _imu_publishers;
  std::shared_ptr<SyncedImuPublisher> _synced_imu_publisher;
  std::map<rs2_stream, int> _image_format;
  std::map<stream_index_pair, ros::Publisher> _info_publisher;
  std::map<stream_index_pair, cv::Mat> _image;
  std::map<rs2_stream, std::string> _encoding;

  std::map<stream_index_pair, int> _seq;
  std::map<rs2_stream, int> _unit_step_size;
  std::map<stream_index_pair, sensor_msgs::CameraInfo> _camera_info;
  std::atomic_bool _is_initialized_time_base;
  double _camera_time_base;
  std::map<stream_index_pair, std::vector<rs2::stream_profile>> _enabled_profiles;

  ros::Publisher _pointcloud_publisher;
  ros::Time _ros_time_base;
  bool _sync_frames;
  bool _pointcloud;
  bool _publish_odom_tf;
  double _temperature_warning_threshold;
  double _temperature_error_threshold;

  imu_sync_method _imu_sync_method;

  //* Custom attributes
  ros::Publisher _timestamping_info_publisher;
  timestamping_method _timestamping_method;
  double _fixed_time_offset = 0.0;
  ros::ServiceServer _toggleColorService;
  ros::ServiceServer _toggleEmitterService;
  ros::ServiceServer _calibration_intrinsic_health_server;
  ros::ServiceServer _calibration_extrinsic_health_server;
  ros::ServiceServer _factory_calibration_server;
  ros::ServiceServer _intrinsic_calibration_server;
  ros::ServiceServer _extrinsic_calibration_server;
  ros::ServiceServer _get_current_calibration_server;
  ros::ServiceServer _loadJsonFileService;

  bool _disable_color_startup;
  //* Custom attributes

  std::string _filters_str;
  stream_index_pair _pointcloud_texture;
  PipelineSyncer _syncer;
  std::vector<NamedFilter> _filters;
  std::vector<rs2::sensor> _dev_sensors;
  std::map<rs2_stream, std::shared_ptr<rs2::align>> _align;

  std::map<stream_index_pair, cv::Mat> _depth_aligned_image;
  std::map<stream_index_pair, cv::Mat> _depth_scaled_image;
  std::map<rs2_stream, std::string> _depth_aligned_encoding;
  std::map<stream_index_pair, sensor_msgs::CameraInfo> _depth_aligned_camera_info;
  std::map<stream_index_pair, int> _depth_aligned_seq;
  std::map<stream_index_pair, ros::Publisher> _depth_aligned_info_publisher;
  std::map<stream_index_pair, image_transport::Publisher> _depth_aligned_image_publishers;
  std::map<stream_index_pair, ros::Publisher> _depth_to_other_extrinsics_publishers;
  std::map<stream_index_pair, rs2_extrinsics> _depth_to_other_extrinsics;
  std::map<std::string, rs2::region_of_interest> _auto_exposure_roi;
  std::map<rs2_stream, bool> _is_first_frame;
  std::map<rs2_stream, std::vector<std::function<void()>>> _video_functions_stack;

  typedef std::pair<rs2_option, std::shared_ptr<TemperatureDiagnostics>> OptionTemperatureDiag;
  std::vector<OptionTemperatureDiag> _temperature_nodes;
  std::shared_ptr<std::thread> _monitoring_t;
  mutable std::condition_variable _cv;

  stream_index_pair _base_stream;
  const std::string _namespace;

  sensor_msgs::PointCloud2 _msg_pointcloud;
  std::vector<unsigned int> _valid_pc_indices;

  // Start of custom ANYbotics code
  struct SensorThresholds {
    double minor_warning;
    double major_warning;
    double error;
  };

  void publish_ir_emitter();
  std::shared_ptr<IREmitterDiagnostics> _ir_emitter_diag;

  //! Fault propagation
  SensorThresholds _temperature_asic_thresholds;
  SensorThresholds _temperature_ir_emitter_thresholds;

  std::shared_ptr<StateCollector> _fault_state_collector;
  std::unordered_map<rs2_option, acl::fault_propagation::SensorValueSeverityConverter> _fault_sensor_value_converters;

  // LCSM calibration callbacks
  CalibrationStateCallback calibrationEnterCallback_;
  CalibrationStateCallback calibrationExitCallback_;

  // End of custom ANYbotics code

};  // end class

}  // namespace realsense2_camera
