cmake_minimum_required(VERSION 3.8)
project(rrt_explore)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/ExplorationState.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
   ${msg_files}
   DEPENDENCIES  geometry_msgs std_msgs rclcpp std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

set(
  DEPENDENCIES
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  nav2_msgs
  rclcpp_action
  )
set(
  SOURCES
  src/rrt_node.cpp
  src/rrt.cpp
  src/mtrand.cpp
  src/functions.cpp
)



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

add_executable(rrt ${SOURCES})
ament_target_dependencies(rrt ${DEPENDENCIES})

target_include_directories(rrt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rrt PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(rrt "${cpp_typesupport_target}")


install(TARGETS rrt
DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
