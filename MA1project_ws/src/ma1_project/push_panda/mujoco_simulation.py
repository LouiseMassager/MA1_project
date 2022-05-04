import time
import os
import numpy as np

from mujoco_py import load_model_from_path, MjSim, MjViewer
from mujoco_panda import MujocoRobot
from simulation import Simulation

import time


class MujocoSimulation(Simulation):
	def __init__(
		self,
		panda: MujocoRobot = MujocoRobot(os.getcwd()+"/robot_model/model_mujoco/franka_panda.xml",True,''),

	) -> None:
		
		self.panda=panda
		
        

	def display_info(self)-> None:
	"""Display in the terminal multiple info: ee velocity and pose, joints positions and velocities
	"""
		print("ee velocity : "+str(self.panda.ee_velocity()))
		print("ee position and orientation : "+str(self.panda.ee_pose()))
		print("joints velocities :"+str(self.panda.joint_velocities()))
		print("joints position :"+str(self.panda.joint_positions()))

	def random_mvt(self)-> None:
	"""Apply a random movement to verify that the robot can be simulated. Only gravity is at work, no commands is sent to the actuators.
	"""
		self.panda.step()
		self.panda.render()

		t=0
		while True:
			for i in range(500):  
	    			self.panda.step()		#moving forward in the simulation
	    			self.panda.render()		#updating the view of the simple_box
			self.display_info()
			t += 1
			if t > 15 or os.getenv('TESTING') is not None:   #stop the simulation after 15*500 steps ~15s
	    			break
		time.sleep(10)

		
	def redo_simulation_from_saved_data(self,saveddatafile:str,period:float=None):
		f= open("datafiles/"+saveddatafile,'r')
		while f.readline().strip()=="_":
			start=time.process_time()
			for joint_i in [0,1,2,3,4,5,6,7,8]:
				self.panda.hard_set_joint_positions([float(f.readline().strip())], [joint_i])
			delay=time.process_time()-start
			if delay<period/2:
				self.panda.render()
			time.sleep(period-float(delay))
			self.panda.step()
			self.panda.render()
		f.close()
        
        
	def save_info(self,f)-> None:
        	pass
        	
    	def set_robot_position(self, objective_position: np.array, objective_direction: np.array):
        	pass
        	
    	def go_in_front_of_box(self)-> None:
        	pass
        
    	def push_box(self)-> None:
        	pass
               
	def throw_box(self)-> None:
        	pass
        
