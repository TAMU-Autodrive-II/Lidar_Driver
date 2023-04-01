```bash
source /opt/ros/foxy/setup.bash
  
colcon build
  --ament-cmake-args "-DCMAKE_MODULE_PATH=/path/to/cepton_sdk/cmake/" \
  --packages-select cepton_messages cepton_subscriber cepton_publisher
    
. install/setup.bash

ros2 run cepton_publisher cepton_publisher_node 
```
