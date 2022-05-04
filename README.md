# MA1_project

This project regroup the control of the robotic arm franka panda for a push simulation with 3 different physics engine: Pybullet, Gazebo and Mujoco.
All files are launched with ROS and Python3 is the main programming language for the code.

The Pybullet simulation must first be ran to compute the command for the simulation in Gazebo and MuJoCo (by storing the joints coordinates in the data file).

Many of the code comes from other works:
Pybullet simulation is based on pandagym (https://github.com/qgallouedec/panda-gym)
Gazebo simulation is based on Gaoyuan Liu works ()
MuJoco simulation is based on Saif Sidhik and Baris Yazici works (https://github.com/justagist/mujoco_panda)

 The goal is to test the performances of Pybullet 
 and MuJoCo as a physics engine for reinforcement learning 
 applications in robotics.
 
 

## Features

In order to run this project, multiple prerequisites are needed:
- mujoco-py package from MuJoCo
- python3
- ROS Noetic

Specific at Mujoco simulation:
- pip install numpy-quaternion

Specific at Gazebo simulation:
- sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers


## Deployment

To deploy a simulation of the panda robotic arm multiple files
can be run in a terminal.

To start the push simulation with Pybullet, run:
```bash
  TO CHANGE
```

To start the push simulation with with MuJoCo, run:
```bash
  TO CHANGE
```

To start the push simulation with Gazebo, run:
```bash
  TO CHANGE
```


## Author

- [@LouiseMassager](https://github.com/LouiseMassager)



## Acknowledgements

 - [panda gym](https://github.com/qgallouedec/panda-gym)
 - [franka panda](https://github.com/vikashplus/franka_sim)
 - [mujoco panda](https://github.com/justagist/mujoco_panda)
 - [MoveRL](https://github.com/Gaoyuan-Liu/MoveRL)
