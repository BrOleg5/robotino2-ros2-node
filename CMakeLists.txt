cmake_minimum_required(VERSION 3.8)

cmake_policy(SET CMP0074 NEW)

project(robotino2 VERSION 0.0.1 DESCRIPTION "ROS2 node of Festo Robotino2")

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
find_package(geometry_msgs REQUIRED)
find_package(robotino_interfaces REQUIRED)
find_package(Boost 1.71.0 REQUIRED)

string(CONCAT ROBOTINO2_PUBLIC_HEADER "include/${PROJECT_NAME}/Robotino2.hpp;"
                                      "include/${PROJECT_NAME}/Robotino2Input.hpp;"
                                      "include/${PROJECT_NAME}/Robotino2Output.hpp;"
                                      "include/${PROJECT_NAME}/TCPPayload.hpp;"
                                      "include/${PROJECT_NAME}/RobotinoKinematics.hpp;"
                                      "include/${PROJECT_NAME}/utils.hpp")

add_library(robotino2 src/Robotino2.cpp src/Robotino2Input.cpp src/Robotino2Output.cpp src/RobotinoKinematics.cpp)

set_target_properties(robotino2 PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      PUBLIC_HEADER "${ROBOTINO2_PUBLIC_HEADER}"
                      DEBUG_POSTFIX d)

target_link_libraries(robotino2 ${Boost_LIBRARIES})

target_include_directories(robotino2 PUBLIC
                          $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                          ${Boost_INCLUDE_DIRS})



add_library(robotino2_node src/Robotino2Node.cpp)

set_target_properties(robotino2_node PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      PUBLIC_HEADER "include/${PROJECT_NAME}/Robotino2Node.hpp"
                      DEBUG_POSTFIX d)

target_link_libraries(robotino2_node robotino2)

target_include_directories(robotino2_node PUBLIC
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

ament_target_dependencies(robotino2_node rclcpp std_msgs std_srvs geometry_msgs robotino_interfaces)



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
        robotino2
        robotino2_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        PUBLIC_HEADER DESTINATION include/${PROJECT_NAME})

install(TARGETS
        robotino2_node_bin
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
