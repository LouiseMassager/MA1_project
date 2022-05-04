# push_panda

newpanda is an adaptation of pandagym to
 control the robotic arm franka panda for a push simulation 
 and extract time and end-effector position data 
 from it. Contrary to pandagym, it doens't use OpenAI/gym 
 robotic environments but only rely on PyBullet physics engine.
 The goal is to test the performances of Pybullet 
 compared to Gazebo as a physics engine for reinforcement learning 
 applications in robotics.
 
 (! project still in progress !)


## Features

In order to run this project, multiple prerequisites are needed:
- mujoco-py package from MuJoCo
- python3
- gym package from OpenAI
- ROS Noetic

Specific at Mujoco simulation:
- pip install numpy-quaternion


## Deployment

To deploy a simulation of the panda robotic arm multiple files
can be run in a terminal.

To start a demonstration of the simulation with Pybullet, run:
```bash
  cd push_panda
  python3 main.py "basic"
```

To start the throw simulation with Pybullet, run:
```bash
  cd push_panda
  python3 main.py "throw"
```

To start the push simulation with Pybullet, run:
```bash
  cd push_panda
  python3 main.py "push_online"
```
To start the push simulation with Pybullet at fixed speed, run:
```bash
  cd push_panda
  python3 main.py "push_constant_speed"
```

To start the push simulation with Pybullet and record the joint angles on a text file "text_file_name.txt", run:
```bash
  cd push_panda
  python3 main.py "push_online" "text_file_name.txt"
```

To redo the push simulation with Pybullet based on saved results on the text file "text_file_name.txt" with a period of p seconds, run:
```bash
  cd push_panda
  python3 main.py "push_offline" "text_file_name.txt" p
```

To start a demonstration of the simulation with MuJoCo, run:
```bash
  python3 mujoco_demo.py
```

## Screenshots

<img src="pybullet_demo.png" width=1000>
<img src="mujoco_demo.png" width=1000>


## Author

- [@LouiseMassager](https://github.com/LouiseMassager/MA1project_ROSMuJoCo/tree/main/new_panda)



## Acknowledgements

 - [panda gym](https://github.com/qgallouedec/panda-gym)
 - [franka panda](https://github.com/vikashplus/franka_sim)
 - [mujoco panda](https://github.com/justagist/mujoco_panda)
