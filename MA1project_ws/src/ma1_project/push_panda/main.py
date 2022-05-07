#!/usr/bin/env python3

from pybullet_simulation import PybulletSimulation
import time, sys, os
from colorama import Fore, Back, Style

import rospy, rospkg
from std_msgs.msg import String

        
def printb(string:str):
	"""Display string in the terminal in blue 
        Args:
            string (str): message to display
        """
	print(Fore.BLUE)
	print(string)
	print(Style.RESET_ALL)

def printg(string:str):
	"""Display string in the terminal in green
        Args:
            string (str): message to display
        """
	print(Fore.GREEN)
	print(string)
	print(Style.RESET_ALL)

def printr(string:str):
	"""Display string in the terminal in red
        Args:
            string (str): message to display
        """
	print(Fore.RED)
	print(string)
	print(Style.RESET_ALL)


        
def get_folderpath(typeoftest:str="launched with python3") -> str:
	"""Get the path of the push_panda folder
        Args:
            typeoftest (str): "rosuse" if launched by ROS
        Returns:
            folderpath (str): path of the push_panda folder
        """
	if typeoftest == "rosuse":
		ros_root = rospkg.get_ros_root()
		r = rospkg.RosPack()
		folderpath=r.get_path('ma1_project')+"/push_panda/"
	else:
		folderpath=os.getcwd()
	return folderpath
	
def check_datafile(mode:str,datafile:str,step:float=0.001):
	"""Ensure format and availability of the wanted datafile
        If datafile is "default", change it to fit the format "jointsangles"+"_s"+str(step)+".txt".
        If mode=="read" and datafile is not in the datafiles folder, make a simulation to create the datafile.
        Args:
            mode (str): "read" or "write".
            datafile (str): name of the file requested in the datafiles folder
            step (float): step interval for the push simulation. It is the distance achieved at each step of the push simulation
        Returns:
            datafile (str): name of the file requested in the datafiles folder
        """
	if datafile=="default":
			datafile="jointsangles"+"_s"+str(step)+".txt"
			
	if (mode=="read") and (not os.path.isfile("datafiles/"+datafile)):
		printr("doing online simulation since no data previously existing")
		try:
			step=float(datafile[14:len(datafile)-4])
		except:
			printr("careful, will run for a step of "+str(step)+" !")
		push_online_test(datafile,step)
		PS.go_in_front_of_box()
		PS.initiate_box()
		printr("end of online simulation")
	return datafile

def default_test():
	"""Launch a series of tasks to verify the good operations of the most used functions of PybulletSimulation
        """
	PS.display_info()
	printg("1. initialize done")

	time.sleep(5.0)
	PS.random_mvt()
	printg("2. random movement done")
	time.sleep(3.0)

	PS.panda.reset()
	printg("3. reset default position")
	time.sleep(3.0)

	PS.random_mvt()
	PS.set_robot_position(PS.ee_objective_position,PS.ee_objective_direction)
	printg("4. should go back to default position with inverse kinematics")
	time.sleep(3.0)

	PS.set_robot_position(PS.box_objective_position,PS.ee_objective_direction)
	printg("5. should go to box")
	time.sleep(3.0)

	PS.go_in_front_of_box()
	PS.display_info()
	printg("6. should go in front of box")
	time.sleep(3.0)

	PS.push_box()
	printg("7. should have pushed red box in green box position !")
	time.sleep(10.0)

	#PS.go_in_front_of_box()
	PS.redo_simulation_from_saved_data(0.007)
	printg("YAAAAY")
	

def throw_test():
	"""Launch a throw simulation of PybulletSimulation
        """
	PS.throw_box()

def push_online_test(datafile:str,step:float=None):
	"""Launch a push simulation and store the joints angles stored in the text file datafile
	Args:
            datafile (str): name of the file requested in the datafiles folder
            step (float): step interval for the push simulation. It is the distance achieved at each step of the push simulation.
        """
	if step is not None:
		PS.push_box(datafile,step)
	else:
		PS.push_box(datafile)
	printg("should have pushed red box in green box position !")


def push_offline_test(datafile:str,period:float=0.007):
	"""Launch a push simulation based on the joints angles stored in the text file datafile
	Args:
            datafile (str): name of the file requested in the datafiles folder
            step (float): step interval for the push simulation. It is the distance achieved at each step of the push simulation.
        """
	PS.redo_simulation_from_saved_data(datafile,period)
	printg("should have pushed red box in green box position based on entries in file "+ datafile+"!")


def publish_offline(topic:str='chatter',datafile:str="jointsangles.txt",period:float=0.01):
	"""Publish the joints angles in the ROS topic topic
	Args:
            topic (str): ROS topic name
            datafile (str): name of the file requested in the datafiles folder
        """
	pub=rospy.Publisher(topic,String,queue_size=10)
	rate=rospy.Rate(1/period)
	f= open(folderpath+"datafiles/"+datafile,'r')
	while not rospy.is_shutdown():
		f= open(folderpath+"datafiles/"+datafile,'r')
		for line in f.readlines():
			jointangle_str=str(line.strip())
			rospy.loginfo(jointangle_str)
			pub.publish(jointangle_str)
			rate.sleep()
		f.close()
		
def publish_online(topic:str='chatter'):
	"""Launch a push simulation and publish the joints angles in the ROS topic topic
	Args:
            topic (str): ROS topic name
        """
	PS.set_publisher(rospy.Publisher(topic,String,queue_size=10))


if __name__ == '__main__':
	rospy.init_node('main', anonymous=True)
	typeoftest = str(sys.argv[1]).lower()
	folderpath=get_folderpath(typeoftest)
	PS=PybulletSimulation(folderpath=folderpath)
	
	'''run with python3 command'''
	if typeoftest== "throw":
		printb("start of throw test")
		throw_test()
	elif typeoftest == "push_offline":
		printb("start offline push test")
		datafile = check_datafile("read",str(sys.argv[2]))
		period = float(sys.argv[3])
		push_offline_test(datafile,period)
	elif typeoftest == "push_online":
		printb("start online push test")
		datafile = str(sys.argv[2])
		try:
			step = float(sys.argv[3])
			datafile=check_datafile("write",datafile,step)
			push_online_test(datafile,step)
		except:
			datafile=check_datafile("write",datafile)
			push_online_test(datafile)
	elif typeoftest == "basic":
		printb("start of default test")
		default_test()
	elif typeoftest == "push_constant_speed":
		datafile = str(sys.argv[2])
		speed = float(sys.argv[3])
		try:
			period = float(sys.argv[4])	#impose period (necessary if computer limitation)
			step = speed * period
			datafile=check_datafile("read",datafile,step)
		except:
			period=0.001/speed		#else use default period
			datafile=check_datafile("read",datafile)
		push_offline_test(datafile,period)
		printg("should have pushed red box in green box position at fixed speed!")
	elif typeoftest!="rosuse":
		printr("incorrect entry, please choose an entry between:\n'basic' \n'throw'\n'push_offline' 'datafile_name.txt' period\n'push_online' 'datafile_name.txt' <step>\n'push_constant_speed' 'datafile_name.txt' speed <period>\n")
		'''run with roslaunch command:'''
	else: 	
		datafile = str(sys.argv[2])
		period = float(sys.argv[3])
		publish_offline(datafile=datafile,period=period)
		
	for i in range(50):			#wait a bit for box to slide
		PS.sim.step()
		PS.sim.render()
		
	PS.view_type("zoompush")		#at the end zoom on target position
	while not rospy.is_shutdown():
		PS.sim.step()
		PS.sim.render()
