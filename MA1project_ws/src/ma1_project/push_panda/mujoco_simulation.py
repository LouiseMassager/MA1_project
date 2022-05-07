import time
import os
import numpy as np

from mujoco_py import load_model_from_path, MjSim, MjViewer
from mujoco_panda import MujocoRobot
from simulation import Simulation

import time,rospy


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

		
	def redo_simulation_from_saved_data(self,saveddatafile:str,period:float=0.001):
		self.panda.set_timestep(period)
		f= open("datafiles/"+saveddatafile,'r')
		r=rospy.Rate(1/period)	#only to have a more accurate visualisation of the simulation ('on time' while not necessary for the results)
		start=rospy.get_time()
		while f.readline().strip()=="_":
			joints=[]
			for joint_i in range(9):
				joints.append(float(f.readline().strip()))
			
			for i in range(10) :	
				self.panda.hard_set_joint_positions(joints, [0,1,2,3,4,5,6,7,8])
				self.panda.step()
			r.sleep()
#			for joint_i in [0,1,2,3,4,5,6,7,8]:
#				self.panda.hard_set_joint_positions(float(f.readline().strip()), joint_i)
#			self.panda.step()
#			r.sleep()
		f.close()
		print('duration: '+str(rospy.get_time()-start))
		while not rospy.is_shutdown():						#stay in position at end
			self.panda.hard_set_joint_positions(joints, [0,1,2,3,4,5,6,7,8])
			self.panda.step(render=True)
        
	def save_info(self,f)-> None:
        	pass
        	
	def set_robot_position(self, objective_position: np.array, objective_direction: np.array):
        	pass
        	
	def go_in_front_of_box(self)-> None:
		self.panda.hard_set_joint_positions([0.22329628372887014,0.3235385023518342,0.021995274091036018,
		-2.722718083051394,-0.07327165512698616,3.0458963065802136,
		1.1024833840870198,-4.52722370717516e-07,1.2021413893037649e-06], [0,1,2,3,4,5,6,7,8])
		self.panda.step(render=True)
        
	def push_box(self)-> None:
        	pass
               
	def throw_box(self)-> None:
        	pass
        
