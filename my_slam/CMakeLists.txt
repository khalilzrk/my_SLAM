cmake_minimum_required(VERSION 3.8)
project(my_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_sim REQUIRED)

find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)  # For OpenCV integration
find_package(vision_msgs REQUIRED)  # For detection message types
# rosidl_generate_interfaces(${PROJECT_NAME}
#     "srv/OddEvenCheck.srv"
#     "srv/CameraService.srv"
#     "action/Navigate.action"
#     DEPENDENCIES 
#     sensor_msgs geometry_msgs
# )

ament_python_install_package(scripts)
install(
  DIRECTORY configs/
  DESTINATION share/${PROJECT_NAME}/configs
)
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)
install(PROGRAMS
  scripts/yoloV8.py
  scripts/LidarIMUDetection.py
  scripts/fix_scan_frame.py
  scripts/tf_connector.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

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