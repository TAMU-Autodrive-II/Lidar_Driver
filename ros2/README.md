## Cepton ROS2 Modules

`cepton_messages`
* Contains ROS2 topic definitions for Cepton lidar messages. Contains `CeptonSensorInfo` and `CeptonPointData` message formats

`cepton_publisher`
* Publishes topics for different lidar messages. Current topics are: `cepton_pcl2` (`sensor_msgs::msg::PointCloud2`) and `cepton_info` (`cepton_messages::msg::CeptonSensorInfo`), and `cepton_points` (`cepton_messages::msg::CeptonPointData`)

`cepton_subscriber`
* A sample of how to receive data for Cepton ROS topics


## Building the packages
All steps are run from the `ros2` directory.
```
# Setup the ROS2 environment
source /opt/ros/foxy/setup.bash

# Build the messages, publisher, and subscriber
colcon build --ament-cmake-args " -DCMAKE_MODULE_PATH=/path/to/redist/cepton_sdk2.0.19/cmake/" --packages-select cepton_messages cepton_subscriber cepton_publisher
```

## Running
All steps are run from the `ros2` directory
```
# Terminal 1
. install/setup.bash
ros2 run cepton_publisher cepton_publisher_node
```
```
# Terminal 2
. install setup.bash
ros2 run cepton_subscriber cepton_subscriber_node
```


**Optional publisher arguments**

The publisher node can optionally be with any of the following arguments:
* `capture_file` (string): the absolute path to a pcap file for replay
* `capture_play_loop` (boolean): whether to replay the pcap in a loop, default=false
* `keep_invalid` (boolean): whether to publish no-return points, default=false
* `sensor_port` (int): the port to listen for sensor UDP data, default=8808
* `publish_pcl2` (boolean): whether to publish frames in `PointCloud2` format, default=true
* `publish_cepton_info` (boolean): whether to publish to `CeptonSensorInfo` topic, default=true
* `publish_cepton_points` (boolean): whether to publish to `CeptonPointData` topic, default=true
```
Example:
ros2 run cepton_publisher cepton_publisher_node --ros-args -p capture_file:="/path/to/pcap" -p capture_play_loop:=true
```

**Optional subscriber options**

If running the subscriber node sample, the following options are provided:
* `subscribe_pcl2` (boolean)
* `subscribe_cepton_points` (boolean)
* `subscribe_cepton_info` (boolean)
* `export_to_csv` (boolean): if true, then export `CeptonPointData` messages to a new directory `frames`, located in current working directory.

## Displaying point cloud in Rviz
After installing `rviz` and starting it with `ros2 run rviz2 rviz2`, and also starting the publisher node, do the following configuration in the RVIZ UI to show the proper point cloud:

* Under `global options`, change the `Fixed Frame` setting to read `lidar_frame`
* Select `Add Topic` and add the `/cepton_pcl2` topic
* Deselect `Autocompute Intensity` and change `Max Intensity` to 1
