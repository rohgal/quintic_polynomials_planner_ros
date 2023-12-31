cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 14)

project(quintic_polynomials_planner_ros)

find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        tf
        message_generation
        plan2control_msgs
        )

add_service_files(
  FILES
  GetPolynomials.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf message_runtime plan2control_msgs
        DEPENDS Eigen
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/quintic_polynomials_planner.cpp src/quintic_polynomials_node.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Eigen_LIBRARIES} yaml-cpp)
