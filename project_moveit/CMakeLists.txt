cmake_minimum_required(VERSION 3.8)
project(project_moveit)

# -----------------------------------------------------------------------------
# 1. Policies & Compiler Flags
# -----------------------------------------------------------------------------
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# -----------------------------------------------------------------------------
# 2. Dependencies
# -----------------------------------------------------------------------------
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

# -----------------------------------------------------------------------------
# 3. Shared Library (PlanningMGI & Helpers)
# -----------------------------------------------------------------------------
add_library(${PROJECT_NAME}_lib SHARED
  src/planning_mgi.cpp
  src/mtc_containers.cpp
  src/task_constructor_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_lib 
  yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib ${COMMON_DEPS} ${MOVEIT_DEPS})

# -----------------------------------------------------------------------------
# 4. Executables
# -----------------------------------------------------------------------------
add_executable(model_and_state_service src/model_and_state_service.cpp)
add_executable(object_tf src/object_tf.cpp)
add_executable(mgi_main src/mgi_main.cpp)
add_executable(planning_mtc src/planning_mtc.cpp)
add_executable(task_constructor_node src/main.cpp)

# Target Dependencies
ament_target_dependencies(model_and_state_service ${COMMON_DEPS} ${MOVEIT_DEPS})
ament_target_dependencies(object_tf ${COMMON_DEPS} ${MOVEIT_DEPS})
ament_target_dependencies(mgi_main ${COMMON_DEPS} ${MOVEIT_DEPS})
ament_target_dependencies(planning_mtc ${COMMON_DEPS} ${MOVEIT_DEPS})
ament_target_dependencies(task_constructor_node ${COMMON_DEPS} ${MOVEIT_DEPS})

# Link Shared Library to Executables
target_link_libraries(model_and_state_service ${PROJECT_NAME}_lib)
target_link_libraries(object_tf ${PROJECT_NAME}_lib)
target_link_libraries(mgi_main ${PROJECT_NAME}_lib)
target_link_libraries(planning_mtc ${PROJECT_NAME}_lib)
target_link_libraries(task_constructor_node ${PROJECT_NAME}_lib)

# -----------------------------------------------------------------------------
# 5. Install Rules (Crucial: Shared Lib vs Executable Destination)
# -----------------------------------------------------------------------------

# 5a. Install Shared Library to standard 'lib'
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 5b. Install Executables to 'lib/${PROJECT_NAME}'
install(TARGETS 
  model_and_state_service 
  object_tf 
  mgi_main 
  planning_mtc
  task_constructor_node
  DESTINATION lib/${PROJECT_NAME}
)

# 5c. Install Include Directory
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
# -----------------------------------------------------------------------------
# 6. Export Package Artifacts
# -----------------------------------------------------------------------------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(${COMMON_DEPS} ${MOVEIT_DEPS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()