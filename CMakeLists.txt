cmake_minimum_required(VERSION 3.8)

cmake_policy(SET CMP0074 NEW)

project(robotino2 VERSION 1.0.0 DESCRIPTION "ROS2 node of Festo Robotino2")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  set(CMAKE_CXX_FLAGS_RELEASE "-D_WIN32_WINNT=0x0601")
  set(CMAKE_CXX_FLAGS_DEBUG "-D_WIN32_WINNT=0x0601")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robotino_interfaces REQUIRED)
find_package(Boost 1.71.0 REQUIRED)

set(INCLUDE_DIR "include/${PROJECT_NAME}")

string(CONCAT ROBOTINO2_PUBLIC_HEADER "${INCLUDE_DIR}/Robotino2.hpp;"
                                      "${INCLUDE_DIR}/Robotino2Input.hpp;"
                                      "${INCLUDE_DIR}/Robotino2Output.hpp;"
                                      "${INCLUDE_DIR}/TCPPayload.hpp;"
                                      "${INCLUDE_DIR}/RobotinoKinematics.hpp;"
                                      "${INCLUDE_DIR}/utils.hpp")

add_library(robotino2_lib src/Robotino2.cpp src/Robotino2Input.cpp src/Robotino2Output.cpp src/RobotinoKinematics.cpp)

set_target_properties(robotino2_lib PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      PUBLIC_HEADER "${ROBOTINO2_PUBLIC_HEADER}"
                      DEBUG_POSTFIX d)

target_link_libraries(robotino2_lib ${Boost_LIBRARIES})

target_include_directories(robotino2_lib PUBLIC
                          $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                          ${Boost_INCLUDE_DIRS})



add_library(robotino2_node src/Robotino2Node.cpp)

set_target_properties(robotino2_node PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      PUBLIC_HEADER "${INCLUDE_DIR}/Robotino2Node.hpp"
                      DEBUG_POSTFIX d)

target_link_libraries(robotino2_node robotino2_lib)

target_include_directories(robotino2_node PUBLIC
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

ament_target_dependencies(robotino2_node rclcpp std_msgs std_srvs geometry_msgs sensor_msgs robotino_interfaces)



add_executable(robotino2_node_bin src/robotino2_node.cpp)

set_target_properties(robotino2_node_bin PROPERTIES
                      OUTPUT_NAME robotino2_node)

target_link_libraries(robotino2_node_bin robotino2_node)

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

install(TARGETS
        robotino2_lib
        robotino2_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        PUBLIC_HEADER DESTINATION ${INCLUDE_DIR})

install(TARGETS
        robotino2_node_bin
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories("include")
ament_export_libraries(robotino2_lib)

ament_package()
