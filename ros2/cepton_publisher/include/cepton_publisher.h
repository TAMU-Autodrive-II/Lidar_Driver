#include <dlfcn.h>

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "cepton_sdk2.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "cepton_messages/msg/cepton_sensor_info.hpp"
#include "cepton_messages/msg/cepton_point_data.hpp"

namespace cepton_ros {
/**
 * @brief Class definition for the CeptonPublisher component
 *
 */
class CeptonPublisher : public rclcpp::Node {
 public:
  CeptonPublisher();
  ~CeptonPublisher();

 private:
  CeptonReplayHandle replay_handle = nullptr;

  friend void sensorFrameCallback(CeptonSensorHandle handle,
                                  int64_t start_timestamp, size_t n_points,
                                  size_t stride, const uint8_t *points,
                                  void *node);

  /**
   * @brief Internal points callback invoked by SDK. Publishes points in format
   * CeptonPointData (see cepton_sdk2.h)
   *
   * @param handle
   * @param start_timestamp
   * @param n_points
   * @param stride
   * @param points
   * @param node
   */
  friend void ceptonFrameCallback(CeptonSensorHandle handle,
                                  int64_t start_timestamp, size_t n_points,
                                  size_t stride, const uint8_t *points,
                                  void *node);

  /**
   * @brief SDK info callback
   *
   * @param handle
   * @param info
   * @param node
   */
  friend void sensorInfoCallback(CeptonSensorHandle handle,
                                 const struct CeptonSensor *info, void *node);

  /**
   * @brief Responsible for publishing the points received from the SDK callback
   * functions. Publish points in format msg::PointCloud2
   *
   */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointsPublisher;

  /**
   * @brief Responsible for publishing the points received from the SDK callback
   * functions. Publish points in format cepton_messages::msg::CeptonPointData
   *
   */
  rclcpp::Publisher<cepton_messages::msg::CeptonPointData>::SharedPtr
      ceptonPointsPublisher;

  /**
   * @brief Responsible for publishing the sensor info packets
   *
   */
  rclcpp::Publisher<cepton_messages::msg::CeptonSensorInfo>::SharedPtr
      infoPublisher;

  const rclcpp::NodeOptions options;

  /* If set to false, do not publish no-return points */
  bool keepInvalid = false;
};
}  // namespace cepton_ros