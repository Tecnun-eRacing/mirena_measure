cmake_minimum_required(VERSION 3.10)
project(mirena_consensus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mirena_common REQUIRED)

include_directories(src)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(mirena_consensus
src/mirena_consensus.cpp
src/DBMMK1/dbmmk1.cpp
src/ekf_scheduler.cpp
)

ament_target_dependencies(mirena_consensus
    rclcpp
    sensor_msgs
    mirena_common
)

install(TARGETS mirena_consensus
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
