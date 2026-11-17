cmake_minimum_required(VERSION 3.16)
project(robotics_lab LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(MUJOCO_ROOT "" CACHE PATH "Path to the MuJoCo installation")
if(NOT MUJOCO_ROOT AND DEFINED ENV{MUJOCO_ROOT})
  set(MUJOCO_ROOT "$ENV{MUJOCO_ROOT}")
endif()
if(NOT MUJOCO_ROOT)
  message(FATAL_ERROR
    "MuJoCo was not found. Set MUJOCO_ROOT to the MuJoCo installation directory, "
    "for example: -DMUJOCO_ROOT=$ENV{HOME}/.mujoco/mujoco-3.17.0")
endif()
if(NOT EXISTS "${MUJOCO_ROOT}/include/mujoco/mujoco.h")
  message(FATAL_ERROR "Invalid MUJOCO_ROOT: ${MUJOCO_ROOT}/include/mujoco/mujoco.h does not exist")
endif()
if(NOT EXISTS "${MUJOCO_ROOT}/lib/libmujoco.so" AND NOT EXISTS "${MUJOCO_ROOT}/lib/libmujoco.dylib")
  message(FATAL_ERROR "Invalid MUJOCO_ROOT: MuJoCo library was not found under ${MUJOCO_ROOT}/lib")
endif()
if(NOT EXISTS "${MUJOCO_ROOT}/simulate/simulate.cc")
  message(FATAL_ERROR "Invalid MUJOCO_ROOT: ${MUJOCO_ROOT}/simulate/simulate.cc does not exist")
endif()

set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT}/include")
set(MUJOCO_LIB_DIR "${MUJOCO_ROOT}/lib")

file(GLOB SIM_SRC
  src/joystick/joystick.cc
  ${MUJOCO_ROOT}/simulate/glfw_*.cc
  ${MUJOCO_ROOT}/simulate/platform_*.cc
  ${MUJOCO_ROOT}/simulate/simulate.cc
  src/lodepng/lodepng.cpp
  src/ros2_backend.cc
)

add_executable(mujoco_sim_app ${SIM_SRC} src/main.cpp)
set_target_properties(mujoco_sim_app PROPERTIES OUTPUT_NAME mujoco_sim)
target_include_directories(mujoco_sim_app PRIVATE
  ${MUJOCO_INCLUDE_DIR} ${MUJOCO_ROOT}/simulate
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lodepng
)
target_link_directories(mujoco_sim_app PRIVATE ${MUJOCO_LIB_DIR})
target_link_libraries(mujoco_sim_app Threads::Threads mujoco glfw yaml-cpp Boost::program_options fmt)
ament_target_dependencies(mujoco_sim_app ament_index_cpp rclcpp std_msgs)

add_executable(lqr_controller src/lqr_controller.cpp)
target_include_directories(lqr_controller PRIVATE include ${MUJOCO_INCLUDE_DIR})
target_link_directories(lqr_controller PRIVATE ${MUJOCO_LIB_DIR})
target_link_libraries(lqr_controller Eigen3::Eigen mujoco)
ament_target_dependencies(lqr_controller rclcpp std_msgs)

set_target_properties(mujoco_sim_app PROPERTIES
  BUILD_RPATH "${MUJOCO_LIB_DIR}"
  INSTALL_RPATH "${MUJOCO_LIB_DIR}"
)
set_target_properties(lqr_controller PROPERTIES
  BUILD_RPATH "${MUJOCO_LIB_DIR}"
  INSTALL_RPATH "${MUJOCO_LIB_DIR}"
)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)
target_include_directories(jstest PRIVATE src/joystick)

install(TARGETS mujoco_sim_app jstest lqr_controller DESTINATION lib/${PROJECT_NAME})
install(FILES config.yaml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY assets DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
