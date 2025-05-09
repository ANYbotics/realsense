// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#include "any_realsense2_camera/realsense_self_calibration.h"

#include <json/json.h>
#include <ros/console.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iomanip>

namespace realsense2_camera {
namespace self_calibration {
rs2::calibration_table get_current_calibration_table(rs2::auto_calibrated_device& dev) {
  // Get current calibration table.
  return dev.get_calibration_table();
}

self_calibration_result self_calibration_step(const std::string& json_config, rs2::auto_calibrated_device& dev) {
  self_calibration_result result;

  // Run actual self-calibration.
  try {
    ROS_INFO("On-chip calibration step starts. Please keep sensor at a fixed position.");
    // The health_score represents the root-mean-square (RMS) error in millimeters
    // of the measured depths versus a best-fit plane over the central 256×144 ROI.
    // Lower absolute values indicate better calibration quality.
    result.new_calibration_table = dev.run_on_chip_calibration(json_config, &result.health_score);
    ROS_INFO_STREAM("On-chip calibration step finished. Health = " << result.health_score);
  } catch (const rs2::error& e) {
    ROS_ERROR_STREAM("RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what());
    throw;
  }

  return result;
}

bool validate_self_calibration(const self_calibration_result& result) {
  // Health score is the RMS error (in mm) between measured depths and a best-fit plane.
  // Health scores can be negative in some cases, which is why we use absolute values.
  // Lower absolute values indicate better calibration quality:
  // - Below OPTIMAL threshold: Excellent calibration
  // - Below USABLE threshold: Acceptable but not ideal calibration
  // - Above USABLE threshold: Poor calibration, needs attention
  const auto abs_score{std::abs(result.health_score)};
  if (abs_score < self_calibration_health_thresholds::OPTIMAL) {
    ROS_INFO("Optimal calibration results achieved. Device is already well calibrated.");
    return true;
  } else {
    if (abs_score < self_calibration_health_thresholds::USABLE) {
      ROS_WARN("Calibration results are usable but not ideal. Please repeat the calibration procedure.");
      return true;
    } else {
      // Unusable results
      ROS_ERROR("Camera requires calibration.");
      return true;
    }
  }
}

/**
 * @brief Performs self-calibration on the RealSense device
 *
 * This function executes either intrinsic or extrinsic calibration depending on the scan_type.
 * It builds a JSON configuration with calibration parameters and runs multiple calibration
 * attempts until either successful or maximum attempts are reached.
 *
 * The calibration JSON includes:
 * - speed: Controls calibration speed (fixed at 3)
 * - average step count: Number of averaging steps (fixed at 20)
 * - scan parameter: Numeric value representing the calibration type
 * - scan type: String representation of the calibration type ("intrinsic" or "extrinsic")
 * - step count: Number of calibration steps (fixed at 20)
 * - apply preset: Controls whether to apply a preset (fixed at 0)
 * - accuracy: Controls calibration accuracy (fixed at 1)
 *
 * @param dev The RealSense device to calibrate
 * @param scan_type The type of calibration to perform (INTRINSIC or EXTRINSIC)
 * @return self_calibration_result Structure containing calibration results
 */
self_calibration_result self_calibrate(rs2::auto_calibrated_device& dev, realsense2_camera::CalibrationType scan_type) {
  self_calibration_result calib_results;

  // Self calibration parameters;
  std::stringstream ss;
  // Set scan parameter based on calibration type
  int scan_parameter = static_cast<int>(scan_type);
  std::string scan_type_str = (scan_parameter == 0) ? "intrinsic" : "extrinsic";

  // Log the scan type for debugging
  ROS_INFO_STREAM("Starting " << scan_type_str << " calibration (scan parameter=" << scan_parameter << ")");

  ss << "{\n \"speed\":" << 3 << ",\n \"average step count\":" << 20 << ",\n \"scan parameter\":" << scan_parameter
     << ",\n \"step count\":" << 20 << ",\n \"apply preset\":" << 0 << ",\n \"accuracy\":" << 1 << "}";

  const std::string json{ss.str()};

  // Loop until we get a good calibration.
  bool valid_calibration{false};
  std::size_t num_calib_attempts{0u};
  while (!valid_calibration && num_calib_attempts < self_calibration_health_thresholds::MAX_NUM_UNSUCCESSFUL_ITERATIONS) {
    ROS_INFO_STREAM("Self Calibration, iteration " << num_calib_attempts << "...");

    try {
      calib_results = self_calibration_step(json, dev);
      valid_calibration = validate_self_calibration(calib_results);
    } catch (const rs2::error& e) {
      ROS_WARN("Self Calibration procedure was unsuccessful. Please point your sensor to an area with visible texture.");
      ROS_WARN("%s", e.what());
    }

    num_calib_attempts++;
  }

  calib_results.success = valid_calibration;

  return calib_results;
}

void write_calibration_to_device(const rs2::calibration_table& table, rs2::auto_calibrated_device& dev) {
  try {
    // Apply self calibration table.
    dev.set_calibration_table(table);

    // Write results to device's ROM.
    dev.write_calibration();
  } catch (const rs2::error& e) {
    ROS_ERROR_STREAM("RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what());
  }
}

void restore_factory_calibration(rs2::auto_calibrated_device& dev) {
  try {
    // Restore factory calibration.
    dev.reset_to_factory_calibration();
  } catch (const rs2::error& e) {
    ROS_ERROR_STREAM("RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what());
  }
}

std::string getCurrentDateAsString() {
  // Helper function to get the current timestamp in a usable format for filenames.
  std::time_t now{std::time(nullptr)};
  std::tm* currentTime{std::localtime(&now)};

  std::ostringstream oss;
  oss << std::put_time(currentTime, "%Y%m%d-%H%M%S");
  return oss.str();
}

std::string write_calibration_to_file(rs2::calibration_table tableRaw, rs2::device& dev, std::string fnPrefix) {
  try {
    // Compose the filename.
    auto stamp{getCurrentDateAsString()};
    std::string fnBase{"rs_calibrations/"};
    std::string filename{fnBase + fnPrefix + stamp + "_realsense_" + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) + ".json"};

    // Interpret the calibration table.
    auto table{reinterpret_cast<coefficients_table*>(tableRaw.data())};
    // Dump the calibration table to a JSON object.
    Json::Value jsonMap;
    jsonMap["baseline"] = std::to_string(table->baseline);

    auto save_float3x4 = [&](std::string name, float3x3& m) {
      jsonMap[std::string(name + ".x.x").c_str()] = std::to_string(m.x.x);
      jsonMap[std::string(name + ".x.y").c_str()] = std::to_string(m.x.y);
      jsonMap[std::string(name + ".x.z").c_str()] = std::to_string(m.x.z);

      jsonMap[std::string(name + ".y.x").c_str()] = std::to_string(m.y.x);
      jsonMap[std::string(name + ".y.y").c_str()] = std::to_string(m.y.y);
      jsonMap[std::string(name + ".y.z").c_str()] = std::to_string(m.y.z);

      jsonMap[std::string(name + ".z.x").c_str()] = std::to_string(m.z.x);
      jsonMap[std::string(name + ".z.y").c_str()] = std::to_string(m.z.y);
      jsonMap[std::string(name + ".z.z").c_str()] = std::to_string(m.z.z);
    };

    save_float3x4("intrinsic_left", table->intrinsic_left);
    save_float3x4("intrinsic_right", table->intrinsic_right);
    save_float3x4("world2left_rot", table->world2left_rot);
    save_float3x4("world2right_rot", table->world2right_rot);

    for (int i = 0; i < max_ds5_rect_resolutions; i++) {
      auto xy = resolutions_list[(ds5_rect_resolutions)i];
      int w = xy.x;
      int h = xy.y;

      jsonMap[std::string("rectified." + std::to_string(i) + ".width").c_str()] = std::to_string(w);
      jsonMap[std::string("rectified." + std::to_string(i) + ".height").c_str()] = std::to_string(h);

      jsonMap[std::string("rectified." + std::to_string(i) + ".fx").c_str()] = std::to_string(table->rect_params[i].x);
      jsonMap[std::string("rectified." + std::to_string(i) + ".fy").c_str()] = std::to_string(table->rect_params[i].y);

      jsonMap[std::string("rectified." + std::to_string(i) + ".ppx").c_str()] = std::to_string(table->rect_params[i].z);
      jsonMap[std::string("rectified." + std::to_string(i) + ".ppy").c_str()] = std::to_string(table->rect_params[i].w);
    }

    // Create the folder if it doesn't exist.
    boost::filesystem::path folder(fnBase);
    if (!boost::filesystem::exists(folder)) {
      if (!boost::filesystem::create_directory(folder)) {
        ROS_ERROR_STREAM("Failed to create the folder.");
      }
    }

    // Write the JSON object to a file.
    std::ofstream file(filename);
    if (file.is_open()) {
      file << jsonMap;
      file.close();
      ROS_INFO_STREAM("JSON file saved successfully. " << filename);
    } else {
      ROS_ERROR_STREAM("Unable to open the file " << filename);
    }

    // Return the absolute path of the saved file
    boost::filesystem::path absolute_path = boost::filesystem::absolute(filename);
    return absolute_path.string();
  } catch (const std::exception& ex) {
    ROS_ERROR_STREAM(ex.what());
    return "";
  }
}
}  // namespace self_calibration

bool run_self_calibration(rs2::device& dev, float& healthScore, bool applyCalibration, realsense2_camera::CalibrationType scan_type) {
  // Create a Pipeline - this serves as a top-level API for streaming and processing frames
  rs2::pipeline pipe;

  std::string serial{dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)};
  // Configure device and start streaming.
  rs2::config cfg;
  // Note, this is required, otherwise the system picks the first available realsense.
  cfg.enable_device(serial);
  cfg.enable_stream(RS2_STREAM_INFRARED, 256, 144, RS2_FORMAT_Y8, 90);
  cfg.enable_stream(RS2_STREAM_DEPTH, 256, 144, RS2_FORMAT_Z16, 90);
  rs2::device device{pipe.start(cfg).get_device()};

  // Get auto calibration handler.
  rs2::auto_calibrated_device calib_dev{device.as<rs2::auto_calibrated_device>()};

  // Get current calibration table.
  ROS_DEBUG("Fetching current calibration table from device...");
  const rs2::calibration_table original_calibration_table{self_calibration::get_current_calibration_table(calib_dev)};

  // Self calibration.
  const self_calibration::self_calibration_result calib_results{self_calibration::self_calibrate(calib_dev, scan_type)};

  // Log the calibration health score.
  // The health score represents the root-mean-square (RMS) error in millimeters
  // of the measured depths versus a best-fit plane over the central 256×144 ROI.
  // Lower absolute values indicate better calibration quality:
  // - Values below 0.25: Optimal calibration
  // - Values between 0.25 and 0.75: Usable calibration
  // - Values above 0.75: Poor calibration requiring attention
  // Note: Health scores can sometimes be negative, use absolute values for comparison.
  healthScore = calib_results.health_score;
  ROS_INFO("Self calibration health score: %f", healthScore);

  // Early return if we do not want to apply the new calibration to the device
  if (!applyCalibration) {
    // Restore the original calibration table.
    self_calibration::write_calibration_to_device(original_calibration_table, calib_dev);
    return calib_results.success;
  }

  if (calib_results.success) {
    // Save to file the original and the new calibrations.
    std::string original_calibration_path = self_calibration::write_calibration_to_file(original_calibration_table, calib_dev, "original_");
    std::string new_calibration_path = self_calibration::write_calibration_to_file(calib_results.new_calibration_table, calib_dev);
    // Apply the new calibration values.
    self_calibration::write_calibration_to_device(calib_results.new_calibration_table, calib_dev);
    ROS_INFO("Calibration table has been written to the device ROM.");
    return true;
  } else {
    ROS_INFO("No changes were made. Restored previous calibration.");
    // Restore the previous calibration table.
    self_calibration::write_calibration_to_device(original_calibration_table, calib_dev);
    return false;
  }
}

bool restore_factory_calibration(rs2::device& dev) {
  // Create a Pipeline - this serves as a top-level API for streaming and processing frames
  rs2::pipeline pipe;

  std::string serial{dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)};
  // Configure device and start streaming.
  rs2::config cfg;
  cfg.enable_device(serial);
  cfg.enable_stream(RS2_STREAM_INFRARED, 256, 144, RS2_FORMAT_Y8, 90);
  cfg.enable_stream(RS2_STREAM_DEPTH, 256, 144, RS2_FORMAT_Z16, 90);
  rs2::device device{pipe.start(cfg).get_device()};

  // Get auto calibration handler.
  rs2::auto_calibrated_device calib_dev{device.as<rs2::auto_calibrated_device>()};

  // Restore the factory calibration table.
  self_calibration::restore_factory_calibration(calib_dev);
  ROS_INFO("Factory calibration restored.");
  return true;
}

}  // namespace realsense2_camera
