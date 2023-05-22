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