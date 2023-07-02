# RotorTM in CXX

## Developing envir and Dependency
1. Ubuntu 18.04/20.04
2. ROS melodic/noetic
3. Boost 1.81.0
4. Eigen 3.4.0
5. CXX 17

## Steps to run single quadrotor dynamic simulator
1. modify test_CXXSimulator.launch file by
  - changing drone mass and interia that are defined in rotor_tm_config/config/uav_params/race.yaml as
      ```xml
      mass: 0.95
      inertia: {Ixx: 0.003, Ixy: 0.0, Ixz: 0.0, Iyx: 0.0, Iyy: 0.003, Iyz: 0.0, Izx: 0.0, Izy: 0.0, Izz: 0.004}
      ```
  - deciding topics to receive input wrench 
    - ```<remap from="reference/fm_cmd" to="/fm_cmd"/>```
    - message's type is rotor_tm_msgs::FMCommand
    - Note: 
      - the input thrust (scalar, norm of thrust force) is defined at /fm_cmd/thrust, 
      - the input torque (3X1 vector, torque in body frame) is defined at /fm_cmd/moments.
  - deciding topics to pusblish drone state
    - ```<remap from="outputs/odom" to="/odom"/>```
    - message's type is nav_msgs::Odometry
2. launch the simulator by
    ```bash
     roslaunch rotor_tm test_CXXSimulator.launch
    ```
3. Note that the ros frequency is 100Hz and quadrotor simulators's step is 0.01, but they are be chosen independently.

## Explaination of Code
1. Class QuadrotorDynamicSimulator defines quadrotor dynamics and uses odeint for integration.
2. rotorTM_node.cpp provides ROS interface and more explanation can be found in comments.


## Modification log
25-06-23
1. change how to compute dEuler from bodyrate.
   1. Delete the functions quadTransMatrix and quadBodyrate2Eulerrate that compute matrix mapping dEuler 2 bodyrate and solve linear equations.
   2. develop the function matirxBodyrate2EulerRate that returns the matrix that maps bodyrate 2 dEuler directly.
2. implement gTests in catkin
3. implement tests with gootle tests.
   1. test cases are defined in the folder gtest and test must be defined in CmakeLists.
      - take testQuadrotorClass for example
      - test cases are defeind in  gtest/testQuadrotorClass.cpp
      - in CMakeLists.txt
        - create a test with the name being singleUAVTest
            ```cmake
            catkin_add_gtest(singleUAVTest gtest/testQuadrotorClass.cpp)
            ```
        - link libraries to be used in this test
            ```cmake
            target_link_libraries(singleUAVTest lib_quadrotor_dynamic_simulator ${catkin_LIBRARIES} Eigen3::Eigen Boost::program_options)
            ```
    2. steps to run tests 
       1. build all packages using ```catkin build```
       2. run test
          1. for all packages
          ```cmake
          catkin test
          ```
          2. for one package like rotor_tm 
          ```
          catkin test rotor_tm
          ```
  
