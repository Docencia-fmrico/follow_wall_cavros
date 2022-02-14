cmake_minimum_required(VERSION 3.5)
project(follow_wall_cavros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    std_msgs
    geometry_msgs
    sensor_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED 
  src/follow_wall_cavros/LifeCycle.cpp
  src/follow_wall_cavros/MoveNode.cpp  
  src/follow_wall_cavros/LaserNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(follow_wall_main src/follow_wall_main.cpp)
ament_target_dependencies(follow_wall_main ${dependencies})

target_link_libraries(follow_wall_main ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  follow_wall_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()