cmake_minimum_required(VERSION 3.16.3)
project(rotor_tm)

set(CMAKE_C_COMPILER "gcc-10")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-10")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#message("C++ standards in this project ${CMAKE_CXX_STANDARD}")

## Specify additional locations of header files
## Your package locations should be listed before other locations


find_package(Boost EXACT 1.81.0 COMPONENTS program_options REQUIRED)
message("Boost version: ${Boost_VERSION}")

find_package(Eigen3 3.4.0 REQUIRED NO_MODULE)
message("Eigen version: ${Eigen3_VERSION}")


find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_library(lib_quadrotor_dynamic_simulator SHARED
            include/rotor_tm_sim/lib_quadrotor_dynamic_simulator.hpp
            src/rotor_tm_sim/lib_quadrotor_dynamic_simulator.cpp)

target_link_libraries(lib_quadrotor_dynamic_simulator Boost::program_options Eigen3::Eigen)


add_library(lib_ros_simulator SHARED
              include/rotor_tm_sim/lib_ros_simulator.hpp
              src/rotor_tm_sim/lib_ros_simulator.cpp)

target_link_libraries(lib_ros_simulator ${catkin_LIBRARIES} lib_quadrotor_dynamic_simulator)    

# add_library(lib_ros_quadrotor_dynamic_simulator SHARED
#              include/rotor_tm_sim/lib_quadrotor_dynamic_simulator.hpp
#              include/rotor_tm_sim/lib_ros_quadrotor_dynamic_somulator.hpp
#              src/rotor_tm_sim/lib_quadrotor_dynamic_simulator.cpp
#              src/rotor_tm_sim/lib_ros_quadrotor_dynamic_somulator.cpp)

# target_link_libraries(lib_ros_quadrotor_dynamic_simulator Boost::program_options Eigen3::Eigen)

add_executable(main src/rotor_tm_sim/main.cpp)

# target_link_libraries(main lib_quadrotor_dynamic_simulator ${catkin_LIBRARIES})

target_link_libraries(main lib_ros_simulator ${catkin_LIBRARIES})


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

###########
## Build ##
###########


## Mark cpp header files for installation
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
