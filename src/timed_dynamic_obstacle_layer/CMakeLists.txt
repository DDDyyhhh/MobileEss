cmake_minimum_required(VERSION 3.8)
project(timed_dynamic_obstacle_layer)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_library(timed_dynamic_obstacle_layer SHARED
  src/timed_dynamic_obstacle_layer.cpp)
target_compile_features(timed_dynamic_obstacle_layer PUBLIC cxx_std_17)
target_include_directories(timed_dynamic_obstacle_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(timed_dynamic_obstacle_layer
  geometry_msgs nav2_costmap_2d pluginlib rclcpp rclcpp_lifecycle
  sensor_msgs tf2_geometry_msgs tf2_ros tf2_sensor_msgs)

pluginlib_export_plugin_description_file(nav2_costmap_2d timed_dynamic_obstacle_layer.xml)

if(BUILD_TESTING)
  add_executable(plugin_load_test test/plugin_load_test.cpp)
  ament_target_dependencies(plugin_load_test nav2_costmap_2d pluginlib)
  add_test(NAME plugin_load_test COMMAND plugin_load_test)
endif()

install(TARGETS timed_dynamic_obstacle_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES timed_dynamic_obstacle_layer.xml DESTINATION share/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
