cmake_minimum_required(VERSION 3.5)
project(rs_type_adapter_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)

add_library(rs_type_adapt_subscriber SHARED
  src/cv_mat_sensor_msgs_image_type_adapter.cpp
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
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rs_type_adapt_subscriber PRIVATE "TYPE_ADAPT_EXAMPLE_BUILDING_LIBRARY")
rclcpp_components_register_node(rs_type_adapt_subscriber
  PLUGIN "rs_type_adapt_example::RsTypeAdaptIntraSub"
  EXECUTABLE image_sub_type_adapt_intra
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
  )
install(
    TARGETS
      rs_type_adapt_subscriber
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
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
