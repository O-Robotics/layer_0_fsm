cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_layer_0_fsm)

# -----------------------------
# Compiler / warnings
# -----------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# -----------------------------
# Interfaces (msg/srv)
# -----------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FSMState.msg"
  "msg/FSMStatus.msg"
  "srv/RequestState.srv"
  DEPENDENCIES builtin_interfaces
)

# Typesupport target for C++
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# -----------------------------
# Common library used by state nodes (and supervisor helpers)
# -----------------------------
add_library(fsm_state_common
  src/_supervisor/process_manager.cpp
  src/_supervisor/state_node_base.cpp
)

target_include_directories(fsm_state_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fsm_state_common
  yaml_cpp_vendor
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  ament_index_cpp
  rcl_interfaces
)

target_link_libraries(fsm_state_common
  ${cpp_typesupport_target}
  yaml-cpp
)


# -----------------------------
# Supervisor node executable
# -----------------------------
add_executable(supervisor_node
  src/_supervisor/supervisor_node.cpp
)

target_include_directories(supervisor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(supervisor_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  ament_index_cpp
  rcl_interfaces
)

target_link_libraries(supervisor_node
  fsm_state_common
  ${cpp_typesupport_target}
)


# -----------------------------
# State node executables
# -----------------------------
function(add_state_node target src)
  add_executable(${target} ${src} ${ARGN})
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${target}
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
    rcl_interfaces
  )
  target_link_libraries(${target}
    fsm_state_common
    ${cpp_typesupport_target}
  )
endfunction()

add_state_node(initializing_state_node src/0_initializing/initializing_state.cpp)
add_state_node(idling_state_node       src/1_idling/idling_state.cpp)
add_state_node(running_state_node      src/2_running/running_state.cpp)
add_state_node(charging_state_node     src/3_charging/charging_state.cpp)
add_state_node(fault_state_node        src/4_fault/fault_state.cpp)

# Ensure code targets build after interfaces are generated
add_dependencies(fsm_state_common ${PROJECT_NAME})
add_dependencies(supervisor_node ${PROJECT_NAME})
add_dependencies(initializing_state_node ${PROJECT_NAME})
add_dependencies(idling_state_node ${PROJECT_NAME})
add_dependencies(running_state_node ${PROJECT_NAME})
add_dependencies(charging_state_node ${PROJECT_NAME})
add_dependencies(fault_state_node ${PROJECT_NAME})


# -----------------------------
# FSM tester node executable (test utility)
# -----------------------------
add_executable(fsm_tester_node
  tests/fsm_tester_node.cpp
)

ament_target_dependencies(fsm_tester_node
  rclcpp
  std_msgs
)


# -----------------------------
# Install
# -----------------------------
install(TARGETS
  fsm_state_common
  supervisor_node
  initializing_state_node
  idling_state_node
  running_state_node
  charging_state_node
  fault_state_node
  fsm_tester_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
