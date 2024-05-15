cmake_minimum_required(VERSION 3.8)
project(pcl_compression)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(benchmark REQUIRED)

find_package(Draco REQUIRED)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

add_subdirectory(3rdparty/lz4)
add_subdirectory(3rdparty/zstd)

add_executable(benchmark_rosbag src/bag_reader.cpp)
ament_target_dependencies(benchmark_rosbag rclcpp rosbag2_cpp sensor_msgs)
target_link_libraries(benchmark_rosbag libzstd_static lz4_static  ${DRACO_LIBRARY})
target_link_libraries(benchmark_rosbag ${CMAKE_CURRENT_SOURCE_DIR}/codec/libpcl_codec.a)

target_include_directories(benchmark_rosbag 
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/lz4
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/zstd
    ${CMAKE_CURRENT_SOURCE_DIR}/codec
    ${DRACO_INCLUDE_DIR}
)

install(
  TARGETS benchmark_rosbag
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
