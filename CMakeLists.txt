cmake_minimum_required(VERSION 3.5)
project(cepton_driver)

include(ExternalProject)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ExternalProject_Add(cepton_messages 
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ros2/cepton_messages
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_CURRENT_SOURCE_DIR}/../../install/cepton_messages
  INSTALL_COMMAND make install && mkdir -p ${CMAKE_CURRENT_SOURCE_DIR}/../../install/cepton_messages/share/colcon-core/packages/ && touch ${CMAKE_CURRENT_SOURCE_DIR}/../../install/cepton_messages/share/colcon-core/packages/cepton_messages
)
ExternalProject_Add(cepton_publisher 
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ros2/cepton_publisher
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_CURRENT_SOURCE_DIR}/../../install/cepton_publisher
  INSTALL_COMMAND make install && mkdir -p ${CMAKE_CURRENT_SOURCE_DIR}/../../install/cepton_publisher/share/colcon-core/packages/ && touch ${CMAKE_CURRENT_SOURCE_DIR}/../../install/cepton_publisher/share/colcon-core/packages/cepton_publisher
  DEPENDS cepton_messages
)
ExternalProject_Add(cepton_subscriber
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ros2/cepton_subscriber
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_CURRENT_SOURCE_DIR}/../../install/cepton_subscriber
  INSTALL_COMMAND make install && mkdir -p ${CMAKE_CURRENT_SOURCE_DIR}/../../install/cepton_subscriber/share/colcon-core/packages/ && touch ${CMAKE_CURRENT_SOURCE_DIR}/../../install/cepton_subscriber/share/colcon-core/packages/cepton_subscriber
  DEPENDS cepton_messages
)

ament_package()
