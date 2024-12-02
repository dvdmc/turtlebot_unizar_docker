cmake_minimum_required(VERSION 3.5)
project(tb_unizar_nav)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mocap4r2_msgs
)
foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

add_library(gt_pose_broadcaster SHARED
  src/gt_pose_broadcaster_node.cpp)
  target_include_directories(gt_pose_broadcaster PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(gt_pose_broadcaster
  "rclcpp"
  "rclcpp_components"
  "nav_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "mocap4r2_msgs"
  )

rclcpp_components_register_node(gt_pose_broadcaster PLUGIN "tb_unizar_nav::GTPoseBroadcaster" EXECUTABLE gt_pose_broadcaster_node)
install(TARGETS gt_pose_broadcaster
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

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