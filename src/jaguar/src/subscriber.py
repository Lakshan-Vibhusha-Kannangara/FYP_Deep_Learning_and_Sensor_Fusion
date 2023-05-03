# DO NOT skip the next commented line
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from sensor_msgs.msg import NavSatFix


def callback(data):
	
	# print the actual message in its raw format
	rospy.loginfo("imu: %s", data.linear_acceleration.x)
	# otherwise simply print a convenient message on the terminal

def callback2(data):
	
	# print the actual message in its raw format
	rospy.loginfo("joint states: %s", data.position)
	# otherwise simply print a convenient message on the terminal

def callback3(data):
	
	# print the actual message in its raw format
	rospy.loginfo("latitude: %s", data.latitude)
		

def main():
	
	# initialize a node by the name 'listener'.
	# you may choose to name it however you like,
	# since you don't have to use it ahead
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("imu", Imu, callback)
	rospy.Subscriber("jaguar_robot/joint_states", JointState, callback2)
	rospy.Subscriber("gps/fix", NavSatFix ,callback3)
	# spin() simply keeps python from
	# exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	
	# you could name this function
	try:
		main()
	except rospy.ROSInterruptException:
		pass

