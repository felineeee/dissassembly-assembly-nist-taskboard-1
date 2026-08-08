cmake_minimum_required(VERSION 3.8)
project(project_perception)

# 1. Compiler Settings & Clangd Support
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Generates compile_commands.json in the build folder for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 2. Dependencies Setup
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common features filters io kdtree search segmentation sample_consensus surface)
find_package(yaml-cpp REQUIRED)
find_package(project_interfaces REQUIRED)

set(COMMON_DEPS
  rclcpp
  pcl_conversions
  pcl_ros
  sensor_msgs
  shape_msgs
  project_interfaces
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  moveit_msgs
  geometric_shapes
  ament_index_cpp
)

set(MOVEIT_DEPS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
)

# Find all ROS 2 / MoveIt packages
foreach(package ${COMMON_DEPS} ${MOVEIT_DEPS})
  find_package(${package} REQUIRED)
endforeach()

# 3. Global Definitions
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DPCL_NO_PRECOMPILE)

# -----------------------------------------------------------------------------
# 4. Shared Library
# -----------------------------------------------------------------------------
add_library(${PROJECT_NAME}_lib SHARED
  src/collision_object_utils.cpp
  src/scene_loader.cpp
  src/pcl/filter.cpp
  src/pcl/helper.cpp
  src/pcl/registration.cpp
  src/pcl/segmentation.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_lib 
  ${PCL_LIBRARIES} 
  ${YAML_CPP_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_lib ${COMMON_DEPS} ${MOVEIT_DEPS})

# -----------------------------------------------------------------------------
# 5. Executables
# -----------------------------------------------------------------------------

# Perception Server
add_executable(perception_server src/perception_server.cpp)
target_link_libraries(perception_server ${PROJECT_NAME}_lib)
ament_target_dependencies(perception_server ${COMMON_DEPS} ${MOVEIT_DEPS})

# Scene Loader (Note: Ensure this file has a main() if you want to run it as a node)
add_executable(scene_loader src/scene_loader.cpp)
target_link_libraries(scene_loader 
  ${PROJECT_NAME}_lib
  project_interfaces::project_interfaces__rosidl_typesupport_cpp
)
ament_target_dependencies(scene_loader ${COMMON_DEPS} ${MOVEIT_DEPS})

# -----------------------------------------------------------------------------
# 6. Install & Packaging
# -----------------------------------------------------------------------------

install(TARGETS ${PROJECT_NAME}_lib perception_server scene_loader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()