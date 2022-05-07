#!/usr/bin/env python3

import time,sys,os
from colorama import Fore, Back, Style
import rospy, rospkg
from std_msgs.msg import String, Float64

folderpath=None
pub1=rospy.Publisher('/panda/joint1_position_controller/command',Float64,queue_size=20)
pub2=rospy.Publisher('/panda/joint2_position_controller/command',Float64,queue_size=20)
pub3=rospy.Publisher('/panda/joint3_position_controller/command',Float64,queue_size=20)
pub4=rospy.Publisher('/panda/joint4_position_controller/command',Float64,queue_size=20)
pub5=rospy.Publisher('/panda/joint5_position_controller/command',Float64,queue_size=20)
pub6=rospy.Publisher('/panda/joint6_position_controller/command',Float64,queue_size=20)
pub7=rospy.Publisher('/panda/joint7_position_controller/command',Float64,queue_size=20)

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

def put_in_front_of_box(datafile:str="jointsangles.txt",period:float=0.007):
	datapath=folderpath+"datafiles/"+datafile
	f= open(datapath,'r')
	f.readline().strip()
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
	f.close()

def publish_offline(datafile:str="jointsangles.txt",period:float=0.007,iterations:int=1):
	datapath=folderpath+"datafiles/"+datafile
	#while not rospy.is_shutdown():
	put_in_front_of_box(datafile,period)
	time.sleep(15) #time to manually put the box in front of the end-effector
	for a in range(iterations):
		f= open(datapath,'r')
		r=rospy.Rate(1/period)
		start=rospy.get_time()
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
			r.sleep()
		f.close()
		print('duration: '+str(rospy.get_time()-start))

def get_folderpath() -> str:
	ros_root = rospkg.get_ros_root()
	r = rospkg.RosPack()
	folderpath=r.get_path('ma1_project')+"/push_panda/"
	return folderpath

if __name__ == '__main__':
	rospy.init_node('main', anonymous=True)
	folderpath=get_folderpath()
	#inputs : datafile to get joints angles and period at which should send commands
	datafile = str(sys.argv[1])
	period = float(sys.argv[2])
	publish_offline(datafile,period)
