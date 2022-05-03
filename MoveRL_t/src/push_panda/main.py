#!/usr/bin/env python3

from pybullet_simulation import PybulletSimulation
import time
import sys
from colorama import Fore, Back, Style
import os

import rospy
from std_msgs.msg import String

import os
import pathlib
folderpath=str(os.path.abspath(os.path.join(os.getcwd(),'..')))+"/MA1project_ws/src/ma1_project/push_panda/"

#PYBULLET DEMO

PS=PybulletSimulation()


def printb(string:str):
	print(Fore.BLUE)
	print(string)
	print(Style.RESET_ALL)

def printg(string:str):
	print(Fore.GREEN)
	print(string)
	print(Style.RESET_ALL)

def printr(string:str):
	print(Fore.RED)
	print(string)
	print(Style.RESET_ALL)
	
def check_datafile(mode:str,datafile:str,step:float=0.001):
	if datafile=="default":
			datafile="jointsangles"+"_s"+str(step)+".txt"
			
	if (mode=="read") and (not os.path.isfile("datafiles/"+datafile)):
		printr("doing online simulation since no data previously existing")
		push_online_test(datafile,step)
		PS.go_in_front_of_box()
		PS.initiate_box()
		printr("end of online simulation")
	return datafile

def default_test():
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
	PS.throw_box()
	time.sleep(5.0)

def push_online_test(saveddatafile:str,step:float=None):
	if step is not None:
		PS.push_box(saveddatafile,step)
	else:
		PS.push_box(saveddatafile)
	printg("should have pushed red box in green box position !")
	time.sleep(10.0)

def push_offline_test(saveddatafile:str,period:float=0.007):
	PS.redo_simulation_from_saved_data(saveddatafile,period)
	printg("should have pushed red box in green box position based on entries in file "+ saveddatafile+"!")
	time.sleep(10.0)

def publish_offline():
	pub=rospy.Publisher('chatter',String,queue_size=10)
	rate=rospy.Rate(10) #10Hz
	print(folderpath+"datafiles/jointsangles.txt")
	f= open(folderpath+"datafiles/jointsangles.txt",'r')
	while not rospy.is_shutdown():
		f= open(folderpath+"datafiles/jointsangles.txt",'r')
		for line in f.readlines():
			jointangle_str=str(line.strip())
			rospy.loginfo(jointangle_str)
			pub.publish(jointangle_str)
			rate.sleep()
		f.close()
		
def publish_online():
	PS.set_publisher(rospy.Publisher('chatter',String,queue_size=10))
#	PS.set_publisher()

if __name__ == '__main__':
	rospy.init_node('main', anonymous=True)
#	pub=rospy.Publisher('chatter',String,queue_size=20)
#	test_str="initiation %s"
#	rospy.loginfo(test_str)
#	rospy.Publisher('chatter',String,queue_size=20)
#	publish_online()
#	push_online_test("aaa.txt")
	publish_offline()
	
#	rospy.spin()
#	try:
#		publish()
#	except rospy.ROSInterruptException:
#		pass
	try:
		typeoftest = str(sys.argv[1]).lower()
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
			#PS.push_box_at_fixed_speed()
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
		else:
			printr("erronous entry")
	except:
		printr("incorrect entry, please choose an entry between:\n'basic' \n'throw'\n'push_offline' 'datafile_name.txt' period\n'push_online' 'datafile_name.txt' <step>\n'push_constant_speed' 'datafile_name.txt' speed <period>\n")
		printb('(!) careful, the period should be taken long enough for your computer !')



