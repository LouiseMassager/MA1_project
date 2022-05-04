from mujoco_simulation import MujocoSimulation

import rospy
import os,sys,time
import numpy as np

sim=MujocoSimulation()
panda=sim.panda

#inputs : datafile to get joints angles and period at which should send commands
datafile = str(sys.argv[1])
period = float(sys.argv[2])

#wait a bit before start of simulation
for i in range(1000):	
	panda.step()
	panda.render()
	
#display information on terminal
sim.display_info()	

#do the simulation based on the joints angles in datafile
sim.redo_simulation_from_saved_data(datafile,period)

#don't stop simulation until break it (CTRL+C)
while not rospy.is_shutdown():
	panda.step()
	panda.render()

