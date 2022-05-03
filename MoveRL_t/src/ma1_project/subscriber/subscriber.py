#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo(rospy.get_caller_id()+ 'I heard %s', data.data)
	print(100*"_")
	print(str(data.data))

def listener():
	rospy.init_node('subscriber', anonymous=True)
	rospy.Subscriber('chatter',String,callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
