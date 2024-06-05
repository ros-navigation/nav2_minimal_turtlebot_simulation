cmake_minimum_required(VERSION 3.8)
project(nav2_minimal_tb3_sim)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY configs launch worlds urdf models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
