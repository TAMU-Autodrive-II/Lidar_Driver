cmake_minimum_required(VERSION 3.5)
project(cepton_publisher)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

 
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# find the non-sdk dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# find the SDK
find_package(CeptonSDK2 REQUIRED)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:\$ORIGIN/..")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:\$ORIGIN/../lib")

# -------------------------------------------------------------------------------
# Compile CeptonPublisher
# -------------------------------------------------------------------------------

find_package(cepton_messages REQUIRED)
add_library(${PROJECT_NAME} SHARED src/cepton_publisher.cpp src/sdk_proxy.h)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
                          $<INSTALL_INTERFACE:${PROJECT_NAME}/include>)

include_directories(${CeptonSDK2_INCLUDE_DIR})

# Link ros dependencies
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp_components rclcpp
                          std_msgs sensor_msgs cepton_messages)

# Install
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# -------------------------------------------------------------------------------
# Other setup - Install launch file
# -------------------------------------------------------------------------------

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# -------------------------------------------------------------------
# Publisher standalone publisher node
# -------------------------------------------------------------------

add_executable(cepton_publisher_node src/main.cpp)
target_link_libraries(cepton_publisher_node cepton_publisher)
install(TARGETS cepton_publisher_node DESTINATION lib/${PROJECT_NAME})

# Copy dependencies to the node location
install(FILES ${CeptonSDK2_LIBRARY} ${CeptonSDKLegacy_LIBRARY} DESTINATION lib)

ament_package()
