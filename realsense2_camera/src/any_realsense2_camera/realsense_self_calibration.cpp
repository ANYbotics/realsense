// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#include "any_realsense2_camera/realsense_self_calibration.h"

#include <ros/console.h>

namespace realsense2_camera
{
    namespace self_calibration
    {
        rs2::calibration_table get_current_calibration_table(rs2::auto_calibrated_device& dev)
        {
            // Get current calibration table.
            return dev.get_calibration_table();
        }

        self_calibration_result self_calibration_step(const std::string& json_config, rs2::auto_calibrated_device& dev)
        {
            self_calibration_result result;

            // Run actual self-calibration.
            try {
                ROS_INFO("On-chip calibration step starts. Please keep sensor at a fixed position.");
                result.new_calibration_table = dev.run_on_chip_calibration(json_config, &result.health_score);
                ROS_INFO_STREAM("On-chip calibration step finished. Health = " << result.health_score);
            } catch (const rs2::error& e) {
                ROS_ERROR_STREAM("RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what()); 
                throw;
            }

            return result;
        }

        bool validate_self_calibration(const self_calibration_result& result)
        {
            const auto abs_score = std::abs(result.health_score);
            if (abs_score < self_calibration_health_thresholds::OPTIMAL)
            {
                ROS_INFO("Optimal calibration results achieved. They will be written to the device ROM.");
                return true;
            } else {
                if (abs_score < self_calibration_health_thresholds::USABLE)
                {
                    ROS_WARN("Calibration results are usable but not ideal.");
                }
                else
                {
                    // Unusable results
                    ROS_ERROR("Calibration results are unusable.");
                }
                return false;
            }
        }

        self_calibration_result self_calibrate(rs2::auto_calibrated_device& dev)
        {
            self_calibration_result calib_results;

            // Self calibration parameters;
            std::stringstream ss;
                ss << "{\n \"speed\":" << 3 <<
                    ",\n \"average step count\":" << 20 <<
                    ",\n \"scan parameter\":" << 1 <<
                    ",\n \"step count\":" << 15 <<
                    ",\n \"apply preset\":" << 0 <<
                    ",\n \"accuracy\":" << 1 <<"}";

            const std::string json = ss.str();

            // Loop until we get a good calibration.
            bool valid_calibration = false;
            std::size_t num_calib_attempts = 0;
            while(!valid_calibration && 
                    num_calib_attempts < self_calibration_health_thresholds::MAX_NUM_UNSUCCESSFUL_ITERATIONS)
            {
                ROS_INFO_STREAM("Self Calibration, iteration " << num_calib_attempts << "...");

                try {
                    calib_results = self_calibration_step(json, dev);
                    valid_calibration = validate_self_calibration(calib_results);
                } catch (const rs2::error& e) {
                    ROS_WARN("Self Calibration procedure was unsuccessful. Please point your sensor to an area with visible texture.");
                }

                num_calib_attempts++;
            }

            calib_results.success = valid_calibration;

            return calib_results;
        }

        void write_calibration_to_device(const rs2::calibration_table& table, rs2::auto_calibrated_device& dev)
        {
            try {
                // Apply self calibration table.
                dev.set_calibration_table(table);

                // Write results to device's ROM.
                dev.write_calibration();
            } catch (const rs2::error& e) {
                ROS_ERROR_STREAM("RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what());
            }
        }

        void restore_factory_calibration(rs2::auto_calibrated_device& dev)
        {
            try {
                // Restore factory calibration.
                dev.reset_to_factory_calibration();
            } catch (const rs2::error& e) {
                ROS_ERROR_STREAM("RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what());
            }
        }
    }

bool run_self_calibration(rs2::device& dev)
{
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline pipe;

    // Configure device and start streaming.
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 256, 144, RS2_FORMAT_Y8, 90);
    cfg.enable_stream(RS2_STREAM_DEPTH, 256, 144, RS2_FORMAT_Z16, 90);
    rs2::device device = pipe.start(cfg).get_device();

    // Get auto calibration handler.
    rs2::auto_calibrated_device calib_dev = device.as<rs2::auto_calibrated_device>();

    // Get current calibration table.
    ROS_DEBUG("Fetching current calibration table from device...");
    const rs2::calibration_table original_calibration_table = self_calibration::get_current_calibration_table(calib_dev);

    // Self calibration.
    const self_calibration::self_calibration_result calib_results = self_calibration::self_calibrate(calib_dev);

    if(calib_results.success)
    {
        // Apply the new calibration values.
        self_calibration::write_calibration_to_device(calib_results.new_calibration_table, calib_dev);
        ROS_INFO("Calibration table has been written to the device ROM.");
        return true;
    } else {
        // Restore factory calibration.
        self_calibration::restore_factory_calibration(calib_dev);
        ROS_INFO("Factory calibration restored.");
        return false;
    }
}

}