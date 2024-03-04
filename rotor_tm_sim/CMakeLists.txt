cmake_minimum_required(VERSION 3.16)

# 1. configure compliers 
## commnet the two lines if you want to use your default compliers
set(CMAKE_C_COMPILER "/usr/bin/gcc-10")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-10")

enable_language(CXX)
enable_language(C)

project(rotor_tm)

# print complier information
message("C complier is ${CMAKE_C_COMPILER} and ID is ${CMAKE_C_COMPILER_ID} and version is ${CMAKE_C_COMPILER_VERSION}")
message("C++ complier is ${CMAKE_CXX_COMPILER} and ID is ${CMAKE_CXX_COMPILER_ID} and version is ${CMAKE_CXX_COMPILER_VERSION}")

# set CXX standard as 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

message("C++ standards in this project ${CMAKE_CXX_STANDARD}")

# 2. define packages for building simulators
# boost 1.71
# more boost versions can be tested later
find_package(Boost EXACT 1.71.0 COMPONENTS program_options REQUIRED)
message("Boost version: ${Boost_VERSION}")

# Eigen
find_package(Eigen3 3.4.0 REQUIRED NO_MODULE)
message("Eigen version: ${Eigen3_VERSION}")

# ros pkgs
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    rotor_tm_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)

include_directories(${CMAKE_SOURCE_DIR}/include)

# 3. libs for simulators
## 3.1 base class/lib : rigid body, cable, point mass
## (1) rigid body
add_library(lib_rigidbody SHARED
            include/rotor_tm_sim/lib_rigidbody.hpp
            src/rotor_tm_sim/lib/lib_rigidbody.cpp)

# use this options in the future that specify c++ 17 for  library            
target_compile_options(lib_rigidbody PRIVATE -std=c++17 -Wall -Wextra)

# link libs needed for building lib_rigidbody
target_link_libraries(lib_rigidbody Boost::program_options Eigen3::Eigen)


# (3)pointmass 
add_library(lib_pointmass SHARED
            include/rotor_tm_sim/lib_pointmass.hpp
            src/rotor_tm_sim/lib/lib_pointmass.cpp)

# use this options in the future that specify c++ 17 for  library
target_compile_options(lib_pointmass PRIVATE -std=c++17 -Wall -Wextra)

# link libs needed for building lib_pointmass
target_link_libraries(lib_pointmass Boost::program_options Eigen3::Eigen)

# (4) cable
add_library(lib_cable SHARED
            include/rotor_tm_sim/lib_cable.hpp
            src/rotor_tm_sim/lib/lib_cable.cpp)

target_compile_options(lib_cable PRIVATE -std=c++17 -Wall -Wextra)

target_link_libraries(lib_cable Eigen3::Eigen)


## 3.2 devired class/lib : payload, quadrotor, quadrotor + cable
## (1) quadrotor
add_library(lib_quadrotor SHARED
            include/rotor_tm_sim/lib_quadrotor.hpp
            src/rotor_tm_sim/lib/lib_quadrotor.cpp)

# use this options in the future that specify c++ 17 for  library
target_compile_options(lib_quadrotor PRIVATE -std=c++17 -Wall -Wextra)

# link libs needed for building lib_quadrotor
target_link_libraries(lib_quadrotor Boost::program_options Eigen3::Eigen)

# (3) mav + cable
add_library(lib_uav_cable SHARED
            include/rotor_tm_sim/lib_uav_cable.hpp
            src/rotor_tm_sim/lib/lib_uav_cable.cpp)

target_compile_options(lib_uav_cable PRIVATE -std=c++17 -Wall -Wextra)

target_link_libraries(lib_uav_cable lib_cable lib_quadrotor lib_rigidbody Eigen3::Eigen)

# (3) payload 
add_library(lib_payload SHARED
            include/rotor_tm_sim/lib_payload.hpp
            src/rotor_tm_sim/lib/lib_payload.cpp)

# use this options in the future that specify c++ 17 for  library
target_compile_options(lib_payload PRIVATE -std=c++17 -Wall -Wextra)

# link libs needed for building lib_payload
target_link_libraries(lib_payload lib_rigidbody lib_uav_cable Boost::program_options Eigen3::Eigen)


#  (4) cooperative
add_library(lib_cooperative SHARED
            include/rotor_tm_sim/lib_cooperative.hpp
            src/rotor_tm_sim/lib/lib_cooperative.cpp)

target_compile_options(lib_cooperative PRIVATE -std=c++17 -Wall -Wextra)

# link libs needed for building lib_payload
target_link_libraries(lib_cooperative lib_payload lib_uav_cable lib_rigidbody lib_quadrotor Boost::program_options Eigen3::Eigen)

# ## 3.3 simulation class/lib : single quadrotor + point mass; 
# # (1) single quadrotor + point mass payload
# add_library(lib_quadrotor_pointmass SHARED
#             include/rotor_tm_sim/lib_quadrotor_pointmass.hpp
#             src/rotor_tm_sim/lib/lib_quadrotor_pointmass.cpp)

# target_compile_options(lib_quadrotor_pointmass PRIVATE -std=c++17 -Wall -Wextra)

# target_link_libraries(lib_quadrotor_pointmass lib_pointmass lib_quadrotor lib_quadrotor Boost::program_options Eigen3::Eigen)

# # (2) cooperative quadrotors + payload
# add_library(lib_cooperative SHARED
#             include/rotor_tm_sim/lib_cooperative.hpp
#             src/rotor_tm_sim/lib/lib_cooperative.cpp)

# target_compile_options(lib_cooperative PRIVATE -std=c++17 -Wall -Wextra)

# target_link_libraries(lib_cooperative lib_cable Eigen3::Eigen)



# 4 ros node
# ros node for quadrotor + point mass
# add_executable(rotorTM_CXX_DronePointMass src/rotor_tm_sim/rotorTM_node_drone_pointmass.cpp)
# target_link_libraries(rotorTM_CXX_DronePointMass lib_quadrotor_pointmass ${catkin_LIBRARIES} Eigen3::Eigen Boost::program_options)


###########
## Test ##
###########

# test for rigid body class
catkin_add_gtest(rigidbody_test gtest/testRigidBodyClass.cpp)

target_link_libraries(rigidbody_test lib_rigidbody ${catkin_LIBRARIES} Eigen3::Eigen Boost::program_options)

# test for quadrotor class
catkin_add_gtest(quadrotor_test gtest/testQuadrotor.cpp)

target_link_libraries(quadrotor_test lib_quadrotor lib_rigidbody ${catkin_LIBRARIES} Eigen3::Eigen Boost::program_options)

# test for cable class
catkin_add_gtest(cable_test gtest/testCable.cpp)

target_link_libraries(cable_test lib_cable ${catkin_LIBRARIES} Eigen3::Eigen Boost::program_options)

# test for UAVcable class
catkin_add_gtest(UAVcable_test gtest/testUAVCable.cpp)

target_link_libraries(UAVcable_test lib_uav_cable ${catkin_LIBRARIES} Eigen3::Eigen Boost::program_options)


# # test for payload class
catkin_add_gtest(payload_test gtest/testPayloadClass.cpp)

target_link_libraries(payload_test lib_payload lib_rigidbody lib_uav_cable  ${catkin_LIBRARIES} Eigen3::Eigen Boost::program_options)


# test for cooperative class
catkin_add_gtest(cooperative_test gtest/testCooperative4MAVs.cpp)

target_link_libraries(cooperative_test lib_cooperative lib_payload lib_rigidbody lib_uav_cable  ${catkin_LIBRARIES} Eigen3::Eigen Boost::program_options)







# # test for quadrotor + point mass
# catkin_add_gtest(singleUAVPointmassTest gtest/testQuadrotorPoinmass.cpp)

# target_link_libraries(singleUAVPointmassTest lib_quadrotor_pointmass ${catkin_LIBRARIES} Eigen3::Eigen Boost::program_options)

# ## test for payloads in cooperative case
# catkin_add_gtest(payloadTest gtest/testPayloadClass.cpp)

# target_link_libraries(payloadTest lib_payload  lib_rigidbody ${catkin_LIBRARIES} Eigen3::Eigen)
