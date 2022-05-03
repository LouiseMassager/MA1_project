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
		timeposarray: np.array =[],
		box_size:float=0.04,
		box_position:np.array=np.array([0.4, 0.1, 0.04/2]),
		box_objective_position:np.array=np.array([0.7, 0.1, 0.04/2]),
		ee_objective_direction:np.array=np.array([1.0,0.0,0.0,0.0]),
		text_file_name:str="data_mujoco.txt",
	) -> None:
		
		self.panda=panda
		self.timeposarray=timeposarray

		self.box_position=box_position
		self.box_objective_position=box_objective_position
		self.ee_objective_direction=ee_objective_direction
		self.box_size=box_size
		self.ee_thickness=0.01
		self.ee_frontbox_position=box_position-np.array([box_size/2 + self.ee_thickness,0.0,0.0])
		self.ee_objective_position=box_objective_position-np.array([box_size/2 + self.ee_thickness,0.0,0.0])
		
		self.text_file_name=text_file_name #to save data
		
#		self.create_scene()
#		self.panda.reset()
#		self.view_type("aesthetic")

	def save_info(self,f)-> None:
		print("to complete")
        

	def display_info(self)-> None:
		print(str(self.panda.ee_velocity()))
		print("to complete")

	def random_mvt(self)-> None:
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
		
	def set_robot_position(self, objective_position: np.array, objective_direction: np.array):
		print("to complete")
        

	def go_in_front_of_box(self)-> None:
		print("to complete")
        
        
	def push_box(self)-> None:
		print("to complete")
               
	def throw_box(self)-> None:
		print("to complete")
		
	def redo_simulation_from_saved_data(self,saveddatafile:str,period:float=None):
		f= open("datafiles/"+saveddatafile,'r')
		a=0
		while f.readline().strip()=="_":
			a=a+1
			start=time.process_time()
			for joint_i in [0,1,2,3,4,5,6,7,8]:
				self.panda.hard_set_joint_positions([float(f.readline().strip())], [joint_i])
			delay=time.process_time()-start
		#	print("delay:"+str(delay))
			if delay<period/2:
				self.panda.render()
			time.sleep(period-float(delay))
			self.panda.step()
			self.panda.render()
		print("aaaaaaa"+str(a))
		f.close()
        
