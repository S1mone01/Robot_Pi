cmake_minimum_required(VERSION 3.8)
project(bumper_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# aggiungi qui i nuovi find_package richiesti
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(create_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # il linter che controlla i copyright viene saltato
  set(ament_cmake_copyright_FOUND TRUE)
  # cpplint viene saltato (funziona solo in un repo git)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# aggiungi qui l'eseguibile e le sue dipendenze
add_executable(bumper_pointcloud_node src/bumper_pointcloud_node.cpp)
ament_target_dependencies(bumper_pointcloud_node
  rclcpp
  sensor_msgs
  create_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  bumper_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

