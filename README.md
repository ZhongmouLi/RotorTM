# RotorTM in CXX

## Developing envir and Dependency
1. Ubuntu 18.04
2. ROS melodic
3. Boost 1.81.0
4. Eigen 3.4.0
5. CXX 17

## Steps to run single quadrotor dynamic simulator
1. modify test_CXXSimulator.launch file by
   - inputing drone mass and interia
      <param name="drone_mass" value="1" />
      <param name="drone_Ixx" value="0.01" />
      <param name="drone_Iyy" value="0.01" />
      <param name="drone_Izz" value="0.02" />
  - deciding topics to receive input wrench 
    - ```<remap from="reference/fm_cmd" to="/fm_cmd"/>```
    - message's type is rotor_tm_msgs::FMCommand
    - thrust force in world frame and torque in body frame
  - deciding topics to pusblish drone state
    - ```<remap from="outputs/odom" to="/odom"/>```
    - message's type is nav_msgs::Odometry
2. launch simulator by
    ```bash
     roslaunch rotor_tm test_CXXSimulator.launch
    ```
3. note that the ros frequency is 100Hz and quadrotor simulators's step is 0.01, but they are be chosen independently.

## Explaination of Code
1. Class QuadrotorDynamicSimulator defines quadrotor dynamics and uses odeint for integration.
2. rotorTM_node.cpp provides ROS interface and more explanation can be found in comments.