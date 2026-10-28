cmake_minimum_required(VERSION 3.8)
project(campus_autoaim)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tdt_interface REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(
  autoaim_node
  src/autoaim_node.cpp
  src/armor.cpp
  src/detector.cpp
  src/number.cpp
  src/draw.cpp
  src/pose.cpp
  src/target_selector.cpp
  src/motion_predictor.cpp
  src/aim_solver.cpp
  src/gimbal_controller.cpp
  src/outpost_aimer.cpp
)

target_include_directories(
  autoaim_node
  PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
)

target_link_libraries(
  autoaim_node
  ${OpenCV_LIBS}
)

ament_target_dependencies(
  autoaim_node
  rclcpp
  tdt_interface
  ament_index_cpp
)

install(
  TARGETS autoaim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
