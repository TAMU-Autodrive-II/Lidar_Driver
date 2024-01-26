#include "cepton_publisher.h"
#include "sdk_proxy.h"
#include <iostream>
#include <chrono>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include <regex>

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using CeptonPoints = cepton_messages::msg::CeptonPointData;
using namespace std::chrono_literals;
using namespace std;

namespace cepton_ros {
inline void check_sdk_error(int re, const char *msg) {
  if (re != CEPTON_SUCCESS) {
    cout << "Error: " << re << " " << string(msg) << endl;
  }
}

static SdkProxy sdk;

// Reflectivity look up table
static const float reflectivity_LUT[256] = {
    0.000f,  0.010f,  0.020f,  0.030f,  0.040f,  0.050f,  0.060f,  0.070f,
    0.080f,  0.090f,  0.100f,  0.110f,  0.120f,  0.130f,  0.140f,  0.150f,
    0.160f,  0.170f,  0.180f,  0.190f,  0.200f,  0.210f,  0.220f,  0.230f,
    0.240f,  0.250f,  0.260f,  0.270f,  0.280f,  0.290f,  0.300f,  0.310f,
    0.320f,  0.330f,  0.340f,  0.350f,  0.360f,  0.370f,  0.380f,  0.390f,
    0.400f,  0.410f,  0.420f,  0.430f,  0.440f,  0.450f,  0.460f,  0.470f,
    0.480f,  0.490f,  0.500f,  0.510f,  0.520f,  0.530f,  0.540f,  0.550f,
    0.560f,  0.570f,  0.580f,  0.590f,  0.600f,  0.610f,  0.620f,  0.630f,
    0.640f,  0.650f,  0.660f,  0.670f,  0.680f,  0.690f,  0.700f,  0.710f,
    0.720f,  0.730f,  0.740f,  0.750f,  0.760f,  0.770f,  0.780f,  0.790f,
    0.800f,  0.810f,  0.820f,  0.830f,  0.840f,  0.850f,  0.860f,  0.870f,
    0.880f,  0.890f,  0.900f,  0.910f,  0.920f,  0.930f,  0.940f,  0.950f,
    0.960f,  0.970f,  0.980f,  0.990f,  1.000f,  1.010f,  1.020f,  1.030f,
    1.040f,  1.050f,  1.060f,  1.070f,  1.080f,  1.090f,  1.100f,  1.110f,
    1.120f,  1.130f,  1.140f,  1.150f,  1.160f,  1.170f,  1.180f,  1.190f,
    1.200f,  1.210f,  1.220f,  1.230f,  1.240f,  1.250f,  1.260f,  1.270f,
    1.307f,  1.345f,  1.384f,  1.424f,  1.466f,  1.509f,  1.553f,  1.598f,
    1.644f,  1.692f,  1.741f,  1.792f,  1.844f,  1.898f,  1.953f,  2.010f,
    2.069f,  2.129f,  2.191f,  2.254f,  2.320f,  2.388f,  2.457f,  2.529f,
    2.602f,  2.678f,  2.756f,  2.836f,  2.919f,  3.004f,  3.091f,  3.181f,
    3.274f,  3.369f,  3.467f,  3.568f,  3.672f,  3.779f,  3.889f,  4.002f,
    4.119f,  4.239f,  4.362f,  4.489f,  4.620f,  4.754f,  4.892f,  5.035f,
    5.181f,  5.332f,  5.488f,  5.647f,  5.812f,  5.981f,  6.155f,  6.334f,
    6.519f,  6.708f,  6.904f,  7.105f,  7.311f,  7.524f,  7.743f,  7.969f,
    8.201f,  8.439f,  8.685f,  8.938f,  9.198f,  9.466f,  9.741f,  10.025f,
    10.317f, 10.617f, 10.926f, 11.244f, 11.572f, 11.909f, 12.255f, 12.612f,
    12.979f, 13.357f, 13.746f, 14.146f, 14.558f, 14.982f, 15.418f, 15.866f,
    16.328f, 16.804f, 17.293f, 17.796f, 18.314f, 18.848f, 19.396f, 19.961f,
    20.542f, 21.140f, 21.755f, 22.389f, 23.040f, 23.711f, 24.401f, 25.112f,
    25.843f, 26.595f, 27.369f, 28.166f, 28.986f, 29.830f, 30.698f, 31.592f,
    32.511f, 33.458f, 34.432f, 35.434f, 36.466f, 37.527f, 38.620f, 39.744f,
    40.901f, 42.092f, 43.317f, 44.578f, 45.876f, 47.211f, 48.586f, 50.000f,
};

/**
 * @brief Internal points callback invoked by SDK. Publishes points in format
 * sensor_msgs::PointCloud2
 *
 * @param handle
 * @param start_timestamp
 * @param n_points
 * @param stride
 * @param points
 * @param node A pointer to the CeptonPublisher
 */


void ceptonFrameCallback(CeptonSensorHandle handle, int64_t start_timestamp,
                         size_t n_points, size_t stride, const uint8_t *points,
                         void *node) {
  cepton_messages::msg::CeptonPointData cpts;
  cpts.set__handle(handle);
  cpts.set__n_points(n_points);
  cpts.set__start_timestamp(start_timestamp);
  cpts.set__stride(stride);

  cpts.points.resize(n_points * stride);
  memcpy(cpts.points.data(), points, n_points * stride);

  reinterpret_cast<CeptonPublisher *>(node)->ceptonPointsPublisher->publish(
      cpts);
}
long long getChronyTimestamp() {
    FILE* pipe = popen("chronyc -n tracking 2>&1", "r");
    if (!pipe) {
        std::cerr << "Error opening pipe to chronyc command." << std::endl;
        return -1;
    }

    char buffer[128];
    std::string result = "";
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
    }

    auto pcloseStatus = pclose(pipe);
    if (pcloseStatus == -1) {
        std::cerr << "Error closing pipe to chronyc command." << std::endl;
        return -1;
    }

    // Extract last offset value using regular expression
    std::regex offsetRegex("Last offset\\s+:\\s+([+-]?\\d+\\.\\d+)\\s+seconds");
    std::smatch offsetMatch;
    if (std::regex_search(result, offsetMatch, offsetRegex)) {
        std::string offsetStr = offsetMatch[1];
        double lastOffset = std::stod(offsetStr);

        // Get current time and adjust using the last offset
        auto currentTime = std::chrono::system_clock::now();
        auto timeSinceEpoch = std::chrono::duration_cast<std::chrono::microseconds>(currentTime.time_since_epoch());
        auto adjustedTime = timeSinceEpoch + std::chrono::microseconds(static_cast<long long>(lastOffset * 1e6));

        return adjustedTime.count();
    }

    return -1;
}

void sensorFrameCallback(CeptonSensorHandle handle, int64_t start_timestamp,
                         size_t n_points, size_t stride, const uint8_t *points,
                         void *node) {
  // https://github.com/ros/common_msgs/blob/jade-devel/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h
  PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = 4;
  long long chronyTimestamp = getChronyTimestamp();

  cloud.header.stamp.sec = chronyTimestamp / (int64_t)1e6;
  cloud.header.stamp.nanosec = chronyTimestamp % (int64_t)1e6 * (int64_t)1e3;
  cloud.header.frame_id = "lidar_frame";

  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2Fields(
      8,
      // clang-format off
    "x", 1, PointField::FLOAT32, 
    "y", 1, PointField::FLOAT32, 
    "z", 1, PointField::FLOAT32,
    "intensity", 1, PointField::FLOAT32, 
    "timestamp_s", 1, PointField::INT32, 
    "timestamp_us", 1, PointField::INT32,
    "flags", 1, PointField::UINT8,
    "channel_id", 1, PointField::UINT8
      // clang-format on
  );

  // Allocate space for the points vector
  mod.resize(n_points);

  bool keepInvalid = reinterpret_cast<CeptonPublisher *>(node)->keepInvalid;
  int64_t tsCumul = start_timestamp;

  // Populate the cloud fields
  sensor_msgs::PointCloud2Iterator<float> x_iter(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y_iter(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z_iter(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> i_iter(cloud, "intensity");
  sensor_msgs::PointCloud2Iterator<int32_t> t_iter(cloud, "timestamp_s");
  sensor_msgs::PointCloud2Iterator<int32_t> t_us_iter(cloud, "timestamp_us");
  sensor_msgs::PointCloud2Iterator<uint8_t> f_iter(cloud, "flags");
  sensor_msgs::PointCloud2Iterator<uint8_t> c_iter(cloud, "channel_id");

  int kept = 0;
  for (unsigned i = 0; i < n_points; i++) {
    CeptonPoint const &p0 =
        *reinterpret_cast<CeptonPoint const *>(points + i * stride);
    tsCumul += p0.relative_timestamp;

    // Skip no-return points if keepInvalid was not set true
    if ((p0.flags & CEPTON_POINT_NO_RETURN) != 0 && !keepInvalid) continue;

    *x_iter = p0.x * 0.005;
    *y_iter = p0.y * 0.005;
    *z_iter = p0.z * 0.005;
    *i_iter = reflectivity_LUT[p0.reflectivity];
    *t_iter =
        static_cast<int32_t>(tsCumul / (int64_t)1e6);  // seconds component
    *t_us_iter =
        static_cast<int32_t>(tsCumul % (int64_t)1e6);  // microsec component
    *f_iter = p0.flags;
    *c_iter = p0.channel_id;

    // clang-format off
    ++x_iter; ++y_iter; ++z_iter; ++i_iter; ++t_iter; ++t_us_iter; ++f_iter; ++c_iter;
    // clang-format on

    kept++;
  }
  mod.resize(kept);

  // Publish points
  reinterpret_cast<CeptonPublisher *>(node)->pointsPublisher->publish(cloud);
}

void sensorInfoCallback(CeptonSensorHandle handle,
                        const struct CeptonSensor *info, void *node) {
  auto msg = cepton_messages::msg::CeptonSensorInfo();
  msg.serial_number = info->serial_number;
  msg.handle = handle;
  msg.model_name = info->model_name;
  msg.model = info->model;
  msg.part_number = info->part_number;
  msg.firmware_version = info->firmware_version;
  msg.power_up_timestamp = info->power_up_timestamp;
  msg.time_sync_offset = info->time_sync_offset;
  msg.time_sync_drift = info->time_sync_drift;
  msg.return_count = info->return_count;
  msg.channel_count = info->channel_count;
  msg.status_flags = info->status_flags;
  msg.temperature = info->temperature;

  // Publish info
  reinterpret_cast<CeptonPublisher *>(node)->infoPublisher->publish(msg);
}

CeptonPublisher::CeptonPublisher() : Node("cepton_publisher") {
  // Allowed options, with default values
  declare_parameter("capture_file", "");
  declare_parameter("capture_play_loop", false);
  declare_parameter("sensor_port", 8808);
  declare_parameter("keep_invalid", false);

  // Control published topics
  bool publishPcl2 = true;
  bool publishCeptonPoints = true;
  bool publishCeptonInfo = true;
  declare_parameter("publish_pcl2", publishPcl2);
  declare_parameter("publish_cepton_points", publishCeptonPoints);
  declare_parameter("publish_cepton_info", publishCeptonInfo);

  // Check whether publishing pcl2
  rclcpp::Parameter pPublishPcl2 = get_parameter("publish_pcl2");
  if (pPublishPcl2.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
    publishPcl2 = pPublishPcl2.as_bool();

  // Check whether publishing cepton points
  rclcpp::Parameter pPublishCeptonPoints =
      get_parameter("publish_cepton_points");
  if (pPublishCeptonPoints.get_type() !=
      rclcpp::ParameterType::PARAMETER_NOT_SET)
    publishCeptonPoints = pPublishCeptonPoints.as_bool();

  // Check whether publishing cepton info
  rclcpp::Parameter pPublishCeptonInfo = get_parameter("publish_cepton_info");
  if (pPublishCeptonInfo.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
    publishCeptonInfo = pPublishCeptonInfo.as_bool();

  // Load the SDK shared object library
  try {
    sdk.LoadModule("cepton_sdk2");
  } catch (exception &e) {
    cerr << "LoadModule received exception: " << e.what() << endl;
  }

  // Initialize sdk
  int ret = sdk.Initialize();
  check_sdk_error(ret, "CeptonInitialize");

  // Set legacy mode
  ret = sdk.EnableLegacy();
  check_sdk_error(ret, "CeptonEnableLegacyTranslation");

  // Listen for point data to publish as PointCloud2
  if (publishPcl2) {
    // Create publisher
    pointsPublisher = create_publisher<PointCloud2>("cepton_pcl2", 50);

    // Register callback
    ret = sdk.ListenFrames(CEPTON_AGGREGATION_MODE_NATURAL, sensorFrameCallback,
                           this);
    check_sdk_error(ret, "CeptonListenFrames");
  }

  // Listen for point data to publish as CeptonPointData
  if (publishCeptonPoints) {
    // Create publisher
    ceptonPointsPublisher = create_publisher<CeptonPoints>("cepton_points", 50);

    // Register callback
    ret = sdk.ListenFrames(CEPTON_AGGREGATION_MODE_NATURAL, ceptonFrameCallback,
                           this);
    check_sdk_error(ret, "CeptonListenFrames");
  }

  // Listen for info data
  if (publishCeptonInfo) {
    // Create publisher
    infoPublisher = create_publisher<cepton_messages::msg::CeptonSensorInfo>(
        "cepton_info", 10);

    // Register callback
    ret = sdk.ListenInfo(sensorInfoCallback, this);
    check_sdk_error(ret, "CeptonListenSensorInfo");
  }

  // Check whether to keep invalid points
  rclcpp::Parameter pKeepInvalid = get_parameter("keep_invalid");
  if (pKeepInvalid.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
    keepInvalid = pKeepInvalid.as_bool();

  // Load a pcap for replay, or start networking for a live sensor
  rclcpp::Parameter captureFile = get_parameter("capture_file");
  if (captureFile.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET &&
      !captureFile.as_string().empty()) {
    // If running replay, check for flags
    rclcpp::Parameter capturePlayLoop = get_parameter("capture_play_loop");
    int flag = 0;
    if (capturePlayLoop.get_type() !=
            rclcpp::ParameterType::PARAMETER_NOT_SET &&
        capturePlayLoop.as_bool()) {
      flag |= CEPTON_REPLAY_FLAG_PLAY_LOOPED;
    }
    ret = sdk.ReplayLoadPcap(captureFile.as_string().c_str(), flag,
                             &replay_handle);
    check_sdk_error(ret, "CeptonReplayLoadPcap");
  } else {
    // Start listening for UDP data on the specified port. Default port is 8808
    rclcpp::Parameter port = get_parameter("sensor_port");
    int p = 8808;
    if (port.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
      p = port.as_int();
    sdk.StartNetworkingOnPort(p);
  }
}

CeptonPublisher::~CeptonPublisher() { sdk.Deinitialize(); }
}  // namespace cepton_ros