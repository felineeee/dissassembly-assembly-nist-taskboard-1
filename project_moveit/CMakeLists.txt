cmake_minimum_required(VERSION 3.8)
project(project_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common features filters io kdtree search segmentation sample_consensus surface)
find_package(yaml-cpp REQUIRED)


set(COMMON_DEPS
  rclcpp
  sensor_msgs
  shape_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geometric_shapes
  ament_index_cpp
)

set(MOVEIT_DEPS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_visual_tools
  moveit_msgs
)

foreach(package ${COMMON_DEPS} ${MOVEIT_DEPS})
  find_package(${package} REQUIRED)
endforeach()

# 3. Global Definitions

# -----------------------------------------------------------------------------
# 4. Shared Library
# -----------------------------------------------------------------------------
add_library(${PROJECT_NAME}_lib SHARED
  src/planning_mgi.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_lib 
  ${YAML_CPP_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_lib ${COMMON_DEPS} ${MOVEIT_DEPS})

# -----------------------------------------------------------------------------
# 5. Executables
# -----------------------------------------------------------------------------

# Perception Server
add_executable(model_and_state_service src/model_and_state_service.cpp)
add_executable(object_tf src/object_tf.cpp)
add_executable(mgi_main src/mgi_main.cpp)

# Link libraries

# target_link_libraries(model_and_state_service ${PROJECT_NAME}_lib)
ament_target_dependencies(model_and_state_service ${COMMON_DEPS} ${MOVEIT_DEPS})
ament_target_dependencies(object_tf ${COMMON_DEPS} ${MOVEIT_DEPS})
ament_target_dependencies(mgi_main ${COMMON_DEPS} ${MOVEIT_DEPS})

target_link_libraries(model_and_state_service ${PROJECT_NAME}_lib)
target_link_libraries(object_tf ${PROJECT_NAME}_lib)
target_link_libraries(mgi_main ${PROJECT_NAME}_lib)

# -----------------------------------------------------------------------------
# 6. Install & Packaging
# -----------------------------------------------------------------------------

install(TARGETS model_and_state_service object_tf
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

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

ament_package()
