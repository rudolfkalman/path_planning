cmake_minimum_required(VERSION 3.8)
project(AutoDriveSimulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
option(MYGAME_VENDORED "Use vendored libraries" OFF)
# find dependencies
#find_package(ament_cmake REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)

if(MYGAME_VENDORED)
  add_subdirectory(vendored/SDL EXCLUDE_FROM_ALL)
else()
  find_package(SDL3 REQUIRED CONFIG REQUIRED COMPONENTS SDL3-shared)
endif()

add_executable(${PROJECT_NAME} 
  src/main.cpp
  src/robot.cpp
  src/sensor.cpp
  src/map.cpp
  src/visualize.cpp
  src/path_planning.cpp
  src/log.cpp
  src/noise.cpp
  src/filter.cpp
  src/slam.cpp
)

#add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE 
  Eigen3::Eigen
  SDL3::SDL3)

#ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs)

target_include_directories(${PROJECT_NAME} PUBLIC include)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
