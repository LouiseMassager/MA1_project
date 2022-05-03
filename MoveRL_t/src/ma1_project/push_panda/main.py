#!/usr/bin/env python3

import time,sys,os
from colorama import Fore, Back, Style
import rospy, rospkg
from std_msgs.msg import String, Float64

folderpath=None

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

def publish_offline(datafile:str="jointsangles.txt",period:float=0.007):
	pub1=rospy.Publisher('/panda/joint1_position_controller/command',Float64,queue_size=20)
	pub2=rospy.Publisher('/panda/joint2_position_controller/command',Float64,queue_size=20)
	pub3=rospy.Publisher('/panda/joint3_position_controller/command',Float64,queue_size=20)
	pub4=rospy.Publisher('/panda/joint4_position_controller/command',Float64,queue_size=20)
	pub5=rospy.Publisher('/panda/joint5_position_controller/command',Float64,queue_size=20)
	pub6=rospy.Publisher('/panda/joint6_position_controller/command',Float64,queue_size=20)
	pub7=rospy.Publisher('/panda/joint7_position_controller/command',Float64,queue_size=20)
	datapath=folderpath+"datafiles/"+datafile
	f= open(datapath,'r')
	while not rospy.is_shutdown():
		f= open(datapath,'r')
		start=time.process_time()
		while f.readline().strip()=="_":
			for i in range(9):
				jointangle_float=float(f.readline().strip())
				if i==0:
					pub1.publish(jointangle_float)
				elif i==1:
					pub2.publish(jointangle_float)
				elif i==2:
					pub3.publish(jointangle_float)
				elif i==3:
					pub4.publish(jointangle_float)
				elif i==4:
					pub5.publish(jointangle_float)
				elif i==5:
					pub6.publish(jointangle_float)
				elif i==6:
					pub7.publish(jointangle_float)
			delay=time.process_time()-start
			if period-float(delay)>0:
				time.sleep(period-float(delay))
			else:
				printr("NEGATIVE PERIOD-DELAY: "+str(period-float(delay)))
			start=time.process_time()

		f.close()

def get_folderpath() -> str:
	ros_root = rospkg.get_ros_root()
	r = rospkg.RosPack()
	folderpath=r.get_path('ma1_project')+"/push_panda/"
	return folderpath

if __name__ == '__main__':
	rospy.init_node('main', anonymous=True)
	folderpath=get_folderpath()
	publish_offline("jointsangles_s0.01.txt",0.001)
