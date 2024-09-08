cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 14)
project(stmotion_controller VERSION 1.0.0 LANGUAGES CXX)

# /* -------------------------------------------------------------------------- */
# /*                              Build Properties                              */
# /* -------------------------------------------------------------------------- */

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

# /* -------------------------------------------------------------------------- */
# /*                                Find Package                                */
# /* -------------------------------------------------------------------------- */

find_package(Eigen3 3.3.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package()
catkin_python_setup()
include_directories(
    include
    ${catkin_INCLUDE_DIRS})

# /* -------------------------------------------------------------------------- */
# /*                               Create Library                               */
# /* -------------------------------------------------------------------------- */

file(GLOB src_files src/*.cpp
                    src/Utils/*.cpp)

add_library(${PROJECT_NAME} SHARED ${src_files})

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    ${catkin_LIBRARIES}
    jsoncpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1)


# /* -------------------------------------------------------------------------- */
# /*                             Testing Executables                            */
# /* -------------------------------------------------------------------------- */

add_executable(${PROJECT_NAME}_controller_usage_example
  examples/cpp/controller_usage_example.cpp
)
target_link_libraries(${PROJECT_NAME}_controller_usage_example
    ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_bringup
  src/controller_node.cpp
)
target_link_libraries(${PROJECT_NAME}_bringup
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_teleoperation_bringup
  src/ros_node/controller_node_teleoperation.cpp
)
target_link_libraries(${PROJECT_NAME}_teleoperation_bringup
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_cartesian_bringup
  src/ros_node/controller_node_cartesian.cpp
)
target_link_libraries(${PROJECT_NAME}_cartesian_bringup
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(PROGRAMS examples/python/controller_usage_example.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})