cmake_minimum_required(VERSION 3.8)
project(dvs_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(dvs_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp)
find_package(OpenCV REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(arc_star_detector src/arc_star_detector.cpp)
ament_target_dependencies(arc_star_detector rclcpp dvs_msgs sensor_msgs cv_bridge)
target_include_directories(arc_star_detector PUBLIC include)
# Link OpenCV
target_link_libraries(arc_star_detector
  ${OpenCV_LIBRARIES}
)


add_executable(efast_detector src/efast_detector.cpp)
ament_target_dependencies(efast_detector rclcpp sensor_msgs cv_bridge)
target_include_directories(efast_detector PUBLIC include)
target_link_libraries(efast_detector ${OpenCV_LIBRARIES})

install(TARGETS
  arc_star_detector
  efast_detector
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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

ament_package()
