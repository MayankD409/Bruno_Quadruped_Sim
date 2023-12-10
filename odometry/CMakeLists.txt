cmake_minimum_required(VERSION 3.8)
project(odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(nav_msgs REQUIRED)

install(
    DIRECTORY include/
    DESTINATION include
    )

install(DIRECTORY
    src
    DESTINATION share/${PROJECT_NAME}/
)

add_library(odometry_plugin SHARED
  src/odometry_plugin.cpp
)

target_include_directories(odometry_plugin PUBLIC include)

ament_target_dependencies(odometry_plugin
  "gazebo_ros"
  "nav_msgs"
  "tf2_ros"
)


ament_export_libraries(odometry_plugin)

install(TARGETS
  odometry_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
