// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#pragma once

#include <any_librealsense2/rs.hpp>  // Include RealSense Cross Platform API
#include <map>

namespace realsense2_camera {

namespace self_calibration {
// Hardcoded types to describe the calibration table. Copied them from any_librealsense2
/* Begin - Duplicated code from `anybotics/drivers/depth_sensors/librealsense/src/types.h` */
struct int2 {
  int x, y;
};
struct float3 {
  float x, y, z;
  float& operator[](int i) { return (&x)[i]; }
};
struct float4 {
  float x, y, z, w;
  float& operator[](int i) { return (&x)[i]; }
};
struct float3x3 {
  float3 x, y, z;
  float& operator()(int i, int j) { return (&x)[j][i]; }
};  // column-major
/* End - Duplicated code from `anybotics/drivers/depth_sensors/librealsense/src/types.h` */

// Hardcoded values to describe the calibration table. Copied them from any_librealsense2.
/* Begin - Duplicated code from `anybotics/drivers/depth_sensors/librealsense/src/common/ds5-private.h` */
struct table_header {
  uint16_t version;     // major.minor. Big-endian
  uint16_t table_type;  // ctCalibration
  uint32_t table_size;  // full size including: TOC header + TOC + actual tables
  uint32_t param;       // This field content is defined ny table type
  uint32_t crc32;       // crc of all the actual table data excluding header/CRC
};
enum ds5_rect_resolutions : unsigned short {
  res_1920_1080,
  res_1280_720,
  res_640_480,
  res_848_480,
  res_640_360,
  res_424_240,
  res_320_240,
  res_480_270,
  res_1280_800,
  res_960_540,
  reserved_1,
  reserved_2,
  res_640_400,
  // Resolutions for DS5U
  res_576_576,
  res_720_720,
  res_1152_1152,
  max_ds5_rect_resolutions
};

static std::map<ds5_rect_resolutions, int2> resolutions_list = {
    {res_320_240, {320, 240}},
    {res_424_240, {424, 240}},
    {res_480_270, {480, 270}},
    {res_640_360, {640, 360}},
    {res_640_400, {640, 400}},
    {res_640_480, {640, 480}},
    {res_848_480, {848, 480}},
    {res_960_540, {960, 540}},
    {res_1280_720, {1280, 720}},
    {res_1280_800, {1280, 800}},
    {res_1920_1080, {1920, 1080}},
    // Resolutions for DS5U
    {res_576_576, {576, 576}},
    {res_720_720, {720, 720}},
    {res_1152_1152, {1152, 1152}},
};

struct coefficients_table {
  table_header header;
  float3x3 intrinsic_left;   //  left camera intrinsic data, normilized
  float3x3 intrinsic_right;  //  right camera intrinsic data, normilized
  float3x3 world2left_rot;   //  the inverse rotation of the left camera
  float3x3 world2right_rot;  //  the inverse rotation of the right camera
  float baseline;            //  the baseline between the cameras in mm units
  uint32_t brown_model;      //  Distortion model: 0 - DS distorion model, 1 - Brown model
  uint8_t reserved1[88];
  float4 rect_params[max_ds5_rect_resolutions];
  uint8_t reserved2[64];
};
/* End - Duplicated code from `anybotics/drivers/depth_sensors/librealsense/src/common/ds5-private.h` */

struct self_calibration_result {
  rs2::calibration_table new_calibration_table;
  float health_score;
  bool success;
};

struct self_calibration_health_thresholds {
  constexpr static float OPTIMAL = 0.25;
  constexpr static float USABLE = 0.75;
  constexpr static float UNUSABLE = 1.0;
  constexpr static std::size_t MAX_NUM_UNSUCCESSFUL_ITERATIONS = 10;
};

rs2::calibration_table get_current_calibration_table(rs2::auto_calibrated_device& dev);

self_calibration_result self_calibration_step(const std::string& json_config, rs2::auto_calibrated_device& dev);

bool validate_self_calibration(const self_calibration_result& result);

void restore_factory_calibration(rs2::auto_calibrated_device& dev);

self_calibration_result self_calibrate(rs2::auto_calibrated_device& dev);

void write_calibration_to_file(rs2::calibration_table tableRaw, rs2::device& dev, std::string fnPrefix = "");

void write_calibration_to_device(const rs2::calibration_table& table, rs2::auto_calibrated_device& dev);

}  // namespace self_calibration

bool run_self_calibration(rs2::device& dev, float& healthScore, bool applyCalibration);
bool restore_factory_calibration(rs2::device& dev);
}  // namespace realsense2_camera
