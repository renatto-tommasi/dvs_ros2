cmake_minimum_required(VERSION 3.8)
project(dvs_view_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dvs_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp)
find_package(OpenCV REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(histogram_2D src/dvs_histogram.cpp)
ament_target_dependencies(histogram_2D rclcpp dvs_msgs sensor_msgs cv_bridge)
target_include_directories(histogram_2D PUBLIC include)
# Link OpenCV
target_link_libraries(histogram_2D
  ${OpenCV_LIBRARIES}
)

add_executable(time_surface src/dvs_time_surface.cpp)
ament_target_dependencies(time_surface rclcpp dvs_msgs sensor_msgs cv_bridge)
target_include_directories(time_surface PUBLIC include)
# Link OpenCV
target_link_libraries(time_surface
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  histogram_2D
  time_surface
  DESTINATION lib/${PROJECT_NAME})


ament_package()
