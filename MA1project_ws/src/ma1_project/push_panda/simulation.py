#!/usr/bin/env python3
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, Tuple, Union

import os
import gym
import gym.spaces
import gym.utils.seeding
import gym_robotics
import numpy as np



class Simulation(ABC):
    """Abstract class for Pybullet and MuJoCo simulations
    """

    @abstractmethod
    def save_info(self,f)-> None:
        """Save info in text file f

        Args:
            f (text file open): The text file.
        """

    @abstractmethod
    def display_info(self)-> None:
        """Display robot informations (end-effector position, joints angles, joints velocities)
        """

    @abstractmethod
    def random_mvt(self)-> None:
        """Create a certain movement of the robot for demonstration."""
	
	
    @abstractmethod
    def set_robot_position(self, objective_position: np.array, objective_direction: np.array):
        """Set robot end-effector position

        Args:
            objective_position (np.array) : The wanted position of the end-effector
            objective_direction (np.array) : The wanted direction of the end-effector
        """
        
    @abstractmethod
    def go_in_front_of_box(self)-> None:
        """Set end-effector position in front of box."""
        
        
    @abstractmethod
    def push_box(self)-> None:
        """Push box."""
               
    @abstractmethod
    def throw_box(self)-> None:
        """Throw box by pushing it forward."""
        
