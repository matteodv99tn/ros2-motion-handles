cmake_minimum_required(VERSION 3.8)
project(motion_control_handle)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")


#  ____                            _                 _
# |  _ \  ___ _ __   ___ _ __   __| | ___ _ __   ___(_) ___  ___
# | | | |/ _ \ '_ \ / _ \ '_ \ / _` |/ _ \ '_ \ / __| |/ _ \/ __|
# | |_| |  __/ |_) |  __/ | | | (_| |  __/ | | | (__| |  __/\__ \
# |____/ \___| .__/ \___|_| |_|\__,_|\___|_| |_|\___|_|\___||___/
#            |_|

find_package(Eigen3 REQUIRED)
set(ROS2_REQUIRED_COMPONENTS
    rclcpp
    visualization_msgs
    geometry_msgs
    interactive_markers
    )
foreach(COMPONENT ${ROS2_REQUIRED_COMPONENTS})
    find_package(${COMPONENT} REQUIRED)
endforeach()

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
    pose_handle
    src/pose_handle.cpp
    )
target_link_libraries(
    pose_handle
    Eigen3::Eigen
    )
target_include_directories(
    pose_handle
    PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    # $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    )
ament_target_dependencies(pose_handle ${ROS2_REQUIRED_COMPONENTS})

#  ___           _        _ _
# |_ _|_ __  ___| |_ __ _| | |
#  | || '_ \/ __| __/ _` | | |
#  | || | | \__ \ || (_| | | |
# |___|_| |_|___/\__\__,_|_|_|
#

install(
    DIRECTORY include/
    DESTINATION include
    )
install(
    DIRECTORY launch rviz
    DESTINATION share/${PROJECT_NAME}
    )
install(
    TARGETS pose_handle
    DESTINATION lib/${PROJECT_NAME}/
    )

ament_export_dependencies(${ROS2_REQUIRED_COMPONENTS})
ament_package()
