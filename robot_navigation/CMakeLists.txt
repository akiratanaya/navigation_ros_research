cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch, config, rviz directories
install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts (Nav2 and CMU)
install(
  PROGRAMS
    # Nav2 scripts
    scripts/nav2/auto_explorer.py
    scripts/nav2/initial_pose_pub.py
    # CMU Autonomy scripts
    scripts/cmu/cmu_sim_bridge.py
    scripts/cmu/lio_sam_cmu_bridge.py
    scripts/cmu/cmu_coverage_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
