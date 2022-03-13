# Iris 3DR - Explicit MPC Gazebo Plugin
Binary Search Tree (BST) of a Explicit MPC implementation in a Gazebo plugin with ROS interface.
All the codes developed to generate the controller and the BST are available [here](https://github.com/Schulze18/Explicit-MPC).

The plugin create three topics:

*/gazebo_client/iris_ref*: receive new reference position

*/gazebo_client/iris_state*: publish the current possition and velocity from the quadrotor

*/gazebo_client/vel_cmd*: publish the velocity applied to the rotors

## Software Versions:
ROS Kinetic  
Gazebo 7


Details about the model and controller are available in [this paper](https://ieeexplore.ieee.org/document/9480185).
