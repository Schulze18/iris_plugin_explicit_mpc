cmake_minimum_required(VERSION 2.8.3)
project(iris_plugin_explicit_mpc)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  xacro
  gazebo
)
find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
    DEPENDS
        geometry_msgs
        roscpp
        rospy
        std_msgs
        xacro
        gazebo
)

include_directories(
    include)
include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(
    iris_plugin_explicit_mpc SHARED
    src/iris_plugin_explicit_mpc.cc
    )


target_link_libraries(iris_plugin_explicit_mpc ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(iris_plugin_explicit_mpc ${catkin_EXPORTED_TARGETS})

install(
    TARGETS
        iris_plugin_explicit_mpc
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

foreach(dir include)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)