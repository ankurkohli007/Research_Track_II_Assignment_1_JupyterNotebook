#!/usr/bin/env python3


"""
.. module:: obstacle_avoidance
    :platform: Unix
    :synopsis: Python module for the obstacle avoidance
.. moduleauthor:: Ankur Kohli

This mode allows to use the robot in manual mode. The robot can be controlled with keyboards.
By receiving the inputs, the robot moves forward/backward, right/left. However, this mode tells
about the distance to the obtacles. Using the /scan to detect the walls around the robot.

Subscribes to:
 /laser_scan
 
Publishes to:
 /avoid
 
This node aims to activate a security feature for driving with the teleop_key modality. The *subscription* to the ``/laser_scan`` topic, the node will be able to get info about the robot's surroundings. The subscription to the topic will give back the ``ranges[0,720]`` array to the subscribed callback. This data structure contains the distance values between the robot and the surrounding walls for a span of 180º degrees in front of the robot. The array simulates the info that a set of lasers would retrieve in an actual environment.

The node will later elaborate the data acquired to publish it on the ``custom_controller`` custom topic through the ``Avoid.msg`` custom message.

"""

# imorting required libraries
from __future__ import print_function

from sensor_msgs.msg import LaserScan
import rospy
from final_assignment.msg import Avoid #custom import


l = True # determines the appearance of a wall at the left side of robot
r = True # determines the appearance of a wall at the right side of robot
f = True # determines the appearance of a wall at the front side of robot


"""
CallBack function is declared and it is used to check the presence of the wall which is close to the robot and what is the wall direction.
"""
def CallBack_avoid(msg):
	"""
	 In this function, data will be obtain and manage accordingly from the /laser_scan. When 
	 this CallBack function receives the laser scan values in terms of ranges[] which is an 
	 array, and these ranges are categorized to perform it's operations. 
	
	 From 0 to 143: which represents the right side of the scanned area.
	 From 144 to 287: which represents the front right side of the scanned area.
	 From 288 to 431: which represents the front side of the scanned area.
	 From 432 to 575: which represents the front left side of the scanned area.
	 From 576 to 719: which represents the left side of the scanned area.

	 gloabal keyword allows us to modify the variable outside of the current scope. It is 
	 used to create a global variable and make changes to the variable in a local context.
	 
	 Args:
	 msg (sensor_messages/LaserScan.msg): contains `ranges` array which provides the 
	 distances of each laser with respect to the objects in the enviroment.
	 
	No Returns
	"""

	global l
	global r
	global f
	
	active_=rospy.get_param("/active") # Active parameters value is assigned to the local variables
	
	
	if active_ == 3:
		
		right = min(msg.ranges[0:143])	        # right checking laser span.
		front = min(msg.ranges[288:431])	# front checking laser span.
		left = min(msg.ranges[576:719])	# left checking laser span.
		
	      # robot is close to right wall.
		if right < 1.0:
			r = False
		else:
			r = True
	      
	      # robot is close to front wall
		if front < 1.0:
			f = False
		else:
			f = True
	      
	      # robot is close to left wall
		if left < 1.0:
			l = False
		else:
		        l = False
	
	#If all the conditions are okay than Operation 3 "Obstacle avoidance operations to drive 
	#the robot assisting them (using keyboard) to avoid collisions" is stopped
		
	else: 
	        r = True
	        f = True
	        l = True
		
# main() is the the point of execution to perform operations
def main():
	"""
	Firstly, we will declare the definition of the subscriber CallBack and the pubblisher. 
	Three pub_msg will be assigned to ok_ for local variables and it will constantly 
	pubblished with a rate of 10 hz rate to the custom topic custom_controller.
	
	No Returns
	"""

	global l
	global r
	global f
	
	# Publishing
	pub = rospy.Publisher('custom_controller', Avoid, queue_size = 10)	
	rospy.init_node('avoidence') # node is initialized
	sub = rospy.Subscriber('/scan', LaserScan, CallBack_avoid) # /scan topic is subscribed
	rate = rospy.Rate(5) 
	
	pub_msg = Avoid() # custom message
	
        # The most common usage patterns for testing for shutdown in rospy
	while not rospy.is_shutdown():
	
		pub_msg.left = l    # custom message field is assigned for left wall
		pub_msg.right = r   # custom message field is assigned for right wall
		pub_msg.front = f   # custom message field is assigned for front wall
		
		# message fields is published
		pub.publish(pub_msg)		
		
		#rate.sleep() will dynamically choose the correct amount of time to sleep to respect the given frequency (delay of 10)
		rate.sleep()
		
#this is used to execute some code only if the file was run directly, and not imported
if __name__=="__main__":
	main()
	
	
	
