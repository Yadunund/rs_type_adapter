cmake_minimum_required(VERSION 3.5)
project(rs_type_adapter_example)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(realsense2_camera REQUIRED)

add_library(rs_type_adapt_subscriber SHARED
  src/rs_type_adapt_subscriber.cpp
)
target_compile_features(rs_type_adapt_subscriber PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(rs_type_adapt_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(rs_type_adapt_subscriber
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${OpenCV_LIBS}
)
ament_target_dependencies(rs_type_adapt_subscriber
  cv_bridge
)

rclcpp_components_register_node(rs_type_adapt_subscriber
  PLUGIN "rs_type_adapt_example::RsTypeAdaptIntraSub"
  EXECUTABLE image_sub_type_adapt_intra
)

add_library(rs_subscriber SHARED
  src/rs_subscriber.cpp
)
target_compile_features(rs_subscriber PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(rs_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(rs_subscriber
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${OpenCV_LIBS}
)
ament_target_dependencies(rs_subscriber
  cv_bridge
)

rclcpp_components_register_node(rs_subscriber
  PLUGIN "rs_type_adapt_example::RsIntraSub"
  EXECUTABLE image_sub_intra
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
  )
install(
    TARGETS
      rs_type_adapt_subscriber
      rs_subscriber
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include/${PROJECT_NAME}
)
ament_export_libraries(
  rs_type_adapt_subscriber
  rs_subscriber
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
