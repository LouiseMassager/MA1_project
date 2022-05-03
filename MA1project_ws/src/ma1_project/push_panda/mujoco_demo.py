from mujoco_simulation import MujocoSimulation

import time
import os
import numpy as np

sim=MujocoSimulation()
panda=sim.panda

for i in range(1000):
	panda.step()
	panda.render()

sim.display_info()

#time.sleep(10)

#sim.random_mvt()


values=[0.6]
indices=[3]
print(np.asarray(values))
panda.hard_set_joint_positions(values, indices)

#for i in range(3000):
#	panda.hard_set_joint_positions([float(i/3000),float(i/3000),float(i/3000)], [0,1,2])
#	panda.step()
#	panda.render()

sim.redo_simulation_from_saved_data('jointsangles.txt',0.01)

while True:
	panda.step()
	panda.render()

