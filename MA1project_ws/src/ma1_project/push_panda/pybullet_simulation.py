#!/usr/bin/env python3
from pybullet_panda import Panda				#link to robot model
from pybullet_handler import PyBulletHandler as PH		#link to Pybullet
from simulation import Simulation

import numpy as np
import time,os

import rospy

"""Simulation in Pybullet.

    Args:
        sim (PH, optionnal): Simulation instance.
        box_size (float, optionnal): Size of the boxes.
        box_position (np.ndarray, optionnal): Position of the box to be pushed.
        box_objective_position (np.ndarray, optionnal): Reference position of the box to be pushed.
        ee_objective_position (np.ndarray, optionnal): End-effector reference orientation.
        folderpath (str,optionnal): Path of the push_panda folder. To be specified for ROS.
"""
class PybulletSimulation(Simulation):
	def __init__(
		self,
		sim: PH = PH(render=True),
		box_size:float=0.04,
		box_position:np.array=np.array([0.4, 0.1, 0.04/2]),
		box_objective_position:np.array=np.array([0.7, 0.1, 0.04/2]),
		ee_objective_direction:np.array=np.array([1.0,0.0,0.0,0.0]),
		folderpath:str=os.getcwd(),
	) -> None:
		
		self.sim=sim
		
		self.panda =Panda(self.sim, block_gripper=True,folderpath=folderpath)
		self.panda_name=self.panda.body_name

		self.box_position=box_position
		self.box_objective_position=box_objective_position
		self.ee_objective_direction=ee_objective_direction
		self.box_size=box_size
		self.ee_thickness=0.01
		self.ee_frontbox_position=box_position-np.array([box_size/2 + self.ee_thickness,0.0,0.0])
		self.ee_objective_position=box_objective_position-np.array([box_size/2 + self.ee_thickness,0.0,0.0])
		
		self.create_scene()
		self.panda.reset()
		self.view_type("aesthetic")
		self.publisher=None
		self.folderpath=folderpath
		
	
	def set_publisher(self,pub) -> None:
		self.publisher=pub
	
	def view_type(self,settings:str) -> None:
		"""Set the camera pose to ether an aesthetic view or a view closer to the box
        	Args:
	     		settings (str): "aestetic" or "push"
		"""
		if (settings=="aesthetic"):
			self.sim.place_visualizer(self.box_position, 0.9, 50.0, -25.0) #aesthetic view
		elif(settings=="push"):
			self.sim.place_visualizer(self.box_position+np.array([0.1,0.0,0.25]), 0.5, 10.0, 20.0) #aesthetic view
		else:
			print("erroneous view_type() input")
		
	def initiate_box(self) -> None:
		"""Create a box to be pushed.
	        """
		self.sim.create_box(
		    body_name="object",
		    half_extents=np.ones(3) * self.box_size / 2,
		    mass=1.0,
		    ghost=False,
		    position=self.box_position,
		    rgba_color=np.array([209, 0, 0, 1]), #red
		)
		
	def create_scene(self) -> None:
		"""Create the initial scene: create the box, the reference for the box, the table and ground.
	        """
		self.sim.create_plane(z_offset=-0.4,color=np.array([0.8, 0.8, 0.8, 1.0]))
		self.sim.create_table(length=1.1, width=0.7, height=0.4, color=np.array([0.65,0.65,0.65,1]), x_offset=+0.3)
		self.initiate_box()
		self.sim.create_box(
		    body_name="target",
		    half_extents=np.ones(3) * self.box_size / 2,
		    mass=0.0,
		    ghost=True,			    #transparent=> can go through
		    position=self.box_objective_position,
		    rgba_color=np.array([6, 168, 0, 0.5]), #green
		)
		
		
	def save_info(self,f)-> None:
		"""Save in the file f the end-effector position, joints angles and velocities. 
	        Args:
	            f (file): file to write.
	        """
		f.write("end-effector's position: "+str(self.panda.get_ee_position()))
		for j in self.panda.joint_indices:
			f.write("\n")
			f.write("joint"+str(j)+" angle: "+str(self.panda.get_joint_angle(j))+"\n")
			f.write("joint"+str(j)+" velocity: "+str(self.panda.get_joint_velocity(j))+"\n")
		f.write("\n"+100*"_"+"\n")
	
	def save_joints_angle(self,f)-> None:
		"""Save in the file f the end-effector position, joints angles and velocities. 
	        Args:
	            f (file): file to write.
	        """
		f.write("_"+"\n")
		for j in self.panda.joint_indices:
			f.write(str(self.panda.get_joint_angle(j))+"\n")
		
			
	def display_info(self)-> None:
		"""Display in the terminal the end-effector position, joints angles and velocities. 
	        """
		print("end-effector's position: "+str(self.panda.get_ee_position()))
		for j in self.panda.joint_indices:
			print("joint"+str(j)+" angle: "+str(self.panda.get_joint_angle(j)))
			print("joint"+str(j)+" velocity: "+str(self.panda.get_joint_velocity(j)))
	
	def publish_info(self)-> None:
		"""Publish through the publisher the end-effector joints angles. 
	        """
		rospy.loginfo("end-effector's position: "+str(self.panda.get_ee_position()))
		self.publisher.publish("end-effector's position: "+str(self.panda.get_ee_position()))
		for j in self.panda.joint_indices:
			rospy.loginfo("joint"+str(j)+" angle: "+str(self.panda.get_joint_angle(j)))
			self.publisher.publish("joint"+str(j)+" angle: "+str(self.panda.get_joint_angle(j)))
			rospy.loginfo("joint"+str(j)+" velocity: "+str(self.panda.get_joint_velocity(j)))
		print("\n")
		
	def random_mvt(self)-> None:
		"""Moves the robots randomly by changing the joints angles one by one.
	        """
		for j in range(7):
			initial=self.sim.get_joint_angle(self.panda_name, j)
			for i in range(10):
				self.sim.set_joint_angles(self.panda_name, np.array([j]), np.array([initial+i/10])) 
				print("joint "+ str(j)+" angle is: "+str(self.sim.get_joint_angle(self.panda_name, j)))
				time.sleep(0.1)

	def set_robot_position(self, objective_position: np.array, objective_direction: np.array=None):
		"""Moves the end-effector in the position objective_position (potentially with specified orientation).
		Args:
	            objective_position (np.array): wanted position of the end-effector.
	            objective_direction (np.array): wanted orientation of the end-effector.
	        """
		if objective_direction is None:
			target_angles=self.sim.inverse_kinematics(self.panda_name,11, objective_position)
		else:
			target_angles=self.sim.inverse_kinematics(self.panda_name,11, objective_position,objective_direction)

		self.sim.set_joint_angles(self.panda_name, self.panda.joint_indices, target_angles)
		self.sim.step()	#update simulation/render only after 3rd iteration
		self.sim.render()
		if self.publisher is not None:
			self.publish_info()

	def go_in_front_of_box(self):
		"""Moves the end-effector in front of the box.
	        """
		self.set_robot_position(self.ee_frontbox_position,self.ee_objective_direction)


	def push_box(self, datafile:str =None,step:float=0.001):
		"""Push the box with the end-effector (and potentially save the joints angle in datafile).
		Args:
	            datafile (str): name of the text file where to store the joints angles.
	            step (float): step interval for the push simulation. It is the distance achieved at each step of the push simulation
	        """
		self.view_type("push")
		self.go_in_front_of_box()
		pos=self.panda.get_ee_position()
		initialpos=pos
		i=1
		if datafile is not None:
			f= open(self.folderpath+"/datafiles/"+datafile,'w+')
		while np.linalg.norm(self.panda.get_ee_position()-(self.ee_objective_position))>(step/2+0.0001) : 
			i+=1
			self.set_robot_position(initialpos+i*np.array([step,0,0]),self.ee_objective_direction)
			if datafile is not None:
				self.save_joints_angle(f)
		if datafile is not None:
			f.close()
	
	def redo_simulation_from_saved_data(self,datafile:str,period:float):
		"""Push the box with the end-effector based on the joints angle in datafile sent at a certain period .
		Args:
	            datafile (str): name of the text file where to store the joints angles.
	            period (float): period at which send the joints angles saved.
	        """
		self.go_in_front_of_box()
		f= open(self.folderpath+"/datafiles/"+datafile,'r')
		r=rospy.Rate(1/period)
		start=rospy.get_time()
		while f.readline().strip()=="_":
			for joint_i in self.panda.joint_indices:
				self.sim.set_joint_angles(self.panda_name, np.array([joint_i]), np.array([float(f.readline().strip())]))
			self.sim.step()	
			self.sim.render()
			r.sleep()
		f.close()
		print('duration: '+str(rospy.get_time()-start))
		
		
	def push_box_at_fixed_speed(self,speed:float=2,period:float=0.005,datafile:str =None):
		"""Push the box at constant speed with the end-effector and save the joints angle in datafile.
		This way of doing the simulation is non-optimized. It is advised to rather use push_box() then redo_simulation_from_saved_data().
		Args:
	            datafile (str): name of the text file where to store the joints angles.
	            period (float): period at which send the joints angles saved.
	            speed (float): speed of the.
	        """
		ee_speed=np.array([speed,0,0])
		self.view_type("push")
		self.go_in_front_of_box()
		if datafile is not None:
			f= open(self.folderpath+"/datafiles/"+datafile,'w+')
		pos=self.panda.get_ee_position()
		tic=time.time()
		
		while np.linalg.norm(pos-(self.ee_objective_position))>0.0011 : #error of +- 0.001
		        
			new_pos=pos+ee_speed*period
			target_angles=self.sim.inverse_kinematics(self.panda_name,11,new_pos,self.ee_objective_direction)
			self.sim.set_joint_angles(self.panda_name, self.panda.joint_indices, target_angles)
			self.sim.step()	#update simulation/render only after 3rd iteration
			self.sim.render()
			time.sleep(period-float(tic-time.time()))
			tic=time.time()
			if datafile is not None:
				self.save_joints_angle(f)

			pos=self.panda.get_ee_position()
			
		if datafile is not None:	
			f.close()
		
		
	
	def throw_box(self):
		"""Throw the box away with the end-effector.
	        """
		self.go_in_front_of_box()
		pos=self.panda.get_ee_position()
		l = [0.005,0.0075,0.01,0.015,0.027,0.043,0.06,0.08,0.11,0.13,0.10,0.05,0,-0.09,0]		
		for elem in l:
			self.set_robot_position(pos+np.array([elem,0,0]),np.array([1.0,0.2,-0.2,0.0]))


		
	
#PROBLEM: 
#when use current position to compute following step : self.panda.get_ee_position(+np.array([0.001,0,0]) 
#computation error add at each => see that end-effector lift up a bit at the end

#	def erronous_method_push_box(self):	
#		self.go_in_front_of_box()
#		pos = self.panda.get_ee_position()
#		initialpos=pos
#		f= open("/datafiles/"+self.text_file_name,'w')
#		while np.linalg.norm(pos-(self.ee_objective_position))>0.002 :
#			print("la: "+str(np.linalg.norm(pos-(self.ee_objective_position))))
#		for i in range(1000):
#		self.set_robot_position(self.panda.get_ee_position(+np.array([0.001,0,0]),self.ee_objective_direction)
#			pos=self.panda.get_ee_position()
#			tic=time.process_time()
#			print("temps : "+str(tic)+" et position: "+str(pos))
#			f.write(str(tic)+" "+str(pos)+"\n")
#			self.timeposarray.append((tic,pos))
#		f.close()




