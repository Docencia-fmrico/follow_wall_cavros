cmake_minimum_required(VERSION 3.5)
project(follow-wall-cavros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
    rclcpp
    std_msgs
)

add_executable(follow_line_main src/follow_line_main.cpp)
ament_target_dependencies(follow_line_main ${dependencies})



install(TARGETS
  follow_line_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})

ament_package()