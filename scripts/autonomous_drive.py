#!/usr/bin/python3


"""
.. module:: autonomous_drive
    :platform: Unix
    :synopsis: Python module for autonomous_drive mode.
.. moduleauthor:: Ankur Kohli

Action client:
    /move_base
    
This node is called the autonomous_driving node that allows the robot to reach a specified position provided by user.

Regarding on the input of the user, those inputs will be sent as a goal to the action server /move_base. 

During the movement, the robot sends feedback regarding status of the goal either reached or cancelled with publishers like /move_base/goal or /move_base/cancel respectively. The robot can create a map with help of gmapping. User is also able
to cancel the goal, and provide a new one. 

Subscribes to:
 /nav_msgs/odometry topic where the simulator publishes the robot position.
 
Action:
 MoveBaseAction
 MoveBaseGoal
 
The Action Client-Service communicate via a "ROS Action Protocol", which is built on top of ROS messages. The client and server then provide a simple API for users to request goals (on the client side) or to execute goals (on the server side) via function calls and callbacks. 

Through out the coding of this node I implemented only the *Actionclient* side of the whole structure using the already existing server of the following action messages:

* ``MoveBaseAction``
* ``MoveBaseGoal``
 
The Actionlib feature, an ActionServer receives the goal message from an ActionClient. In the case of this assignment, the goal is to move the robot's base position. The goal would be a MoveBaseGoal message that contains information about where the robot should move to in the world. For controlling all the robot positions in space, the goal would contain the *target_pose* parameters (stamp, orientation, target position, etc).

"""

# importing required libraries
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import time
import math
import rospy

#for present node state
state="                                                                  "

# uses a flag to remember something or to leave a sign for another program
goalf = False

"""this function will refresh the parameters continously and assigning their updated value to the gloabal varaibles"""
def refresh_variables(): 
	"""
	gloabal keyword allows us to modify the variable outside of the current scope. It is 
	used to create a global variable and make 	changes to the variable in a local context.
	
	No Args 
	
	No Returns
	"""
	global desired_position_x, desired_position_y, active_
	active_ = rospy.get_param('active')
	desired_position_x = rospy.get_param('des_pos_x')
	desired_position_y = rospy.get_param('des_pos_y')
"""
CallBack_odometry function callback's to the topic odom (odometry) which is required to recover the present of the robot in the environment in terms of x and y. Also, the information about the odom of the robot is allocated to gloabl position_ variable.
"""
def CallBack_odometry(msg): 
	"""
	Here, the arguement msg (/odom.msg): this will hold the present position of the robot in
	the enviroment in the terms of x and y
	
	Args:
	 msg (/odom.msg): this variable will contain the current x/y position of the robot in
	 the enviroment.
	 
	 No Returns
	"""

	global position_
	position_ = msg.pose.pose.position
	
"""
This CallBack function is to recover the information about the condition of the robot's position once it reached at the goal position. Moreover, robot will reached at the goal then the target_flag variable will change it's value from 0 to 1. This change will set to the idle state operation of the robot behaviour.
"""
def CallBack(status,result):
	"""
	Arguments status and result states that actionlib_goalStatus and MoveBaseResult
	
	Args:
	 status (actionlib_goalStatus): integer code corrisponding to the terminal state.
	 result (MoveBaseResult): result of the goal processing.
	 
	No Returns
	"""
	global goalf
	
	if status == 3:
		print(" Goal Succeeded!"+state)
		print("\n")
		goalf = True
	
"""
This function is used to define a new goal for the robot through "action client". Additionally, ROS Actions have a client-to-server communication relationship with a specified protocol. The actions use ROS topics to send goal messages from a client to the server. We can also cancel goals using the action client. After receiving a goal, the server processes it and can give information back to the client.
"""
def action_client_set_goal():
	"""
	This function will set the goal
	
	No Args 
	
	No returns
	"""

	goal.target_pose.pose.position.x = desired_position_x
	goal.target_pose.pose.position.y = desired_position_y
	print("AUTONOMOUS DRIVING STATE INITIATED"+state) # send_goal function will activated and keep tracing of the robot target through "CallBack" function.
	print("\n")
	client.send_goal(goal,CallBack) 

"""
This function will initialize the "action client". Also, action server will receive goal message through action client.
"""
def action_client_init():

	global client 
	global goal
	 
	"""
	action client is initialized and created the SimpleActionClient, passing the type of the
	action
	"""
	client = actionlib.SimpleActionClient('move_base',MoveBaseAction) # actionlib package 
	#provides a standardized interface for interfacing with preemptible tasks.
	
	client.wait_for_server() # Waits until the action server has started up and started 
	#istening for goals.
	"""
	Here, MoveBaseGoal is a goal message type and this will contains all the data about the 
	robot behaviour while reaching at the defined goal
	"""
	goal = MoveBaseGoal() # goal messages initialized
	#we'll send a goal to the robot to move forward	
	goal.target_pose.header.frame_id = "map" # goal message set
	goal.target_pose.header.stamp = rospy.Time.now()
 	#goal.target_pose.pose.orientation.w = 1.0 
 
#This function will sets the reset for present robot's behaviour. Furthermore, this function #will activates only when the robot will not reach to the defined target within a limited span #of time lets say 40 seconds.	
def CallBack_reset(event):
	"""
	event is time variable

	active_ is the parameter which will set to 0 and this resets whole state of the
	operations
	
	Args:
	 event (float) : for time variable 
	
	No Returns
	"""
	if active_==1:
		print ("Gol time exceeded :" + str(event.current_real)+state)
		print("\n")
		print("ROBOT failed to reach the defined target location within a span of 40 seconds")
		print("\n")
		# Setting parameters on the parameters server
		rospy.set_param('active', 0)
		
"""
main() is the the point of execution to perform operations
"""
def main():
	"""
	In main(), robot's behaviour will constantly supervised. While loop will start spin 
	right away when the CallBack function subscribed and the node is initialized. During the 
	loop execution node will call the last defined functions as per the present state of the 
	robot defined by the user using global parameters variables. Lastly, it will display the 
	excution states as well.
	
	Fetching the values from the Parameter Server. We can also optionally pass in a default 
	value to use if the parameter is not set
	
	No returns
	"""
	
	active_ = rospy.get_param('active')
	"""
	This parameter will continously supervise the current robot's operatio
	"""
	desired_position_x = rospy.get_param('des_pos_x')
	"""
	This parameter will fetch the x coordinate of the target position which is entered by 
	the user
	"""
	desired_position_y = rospy.get_param('des_pos_y')
	"""
	This parameter will fetch the y coordinate of the target position which is entered by 
	the user.
	"""
	
	global goalf
	rospy.init_node('go_to_desired_pos') # node initialized
	sub_odom = rospy.Subscriber('/odom', Odometry, CallBack_odometry) # Odometry CallBack is subscribed
	""" 
	subscriber can get access to a "connection header", which includes debugging information 
	such as who sent the message, as well information like whether or not a message was 
	latched.
	
	Another, convenience class which makes a best effort at maintaining a particular rate 
	for a loop.
	
	Also, Flags are declared to supervised present robot's operations.
	
	"""
	rate = rospy.Rate(10) # Loop sleep rate
	f1 = False	
	f2 = False
	
	action_client_init() # action client is initialized
	
	i = 0	# to store and print Robot's behaviour and its state
	while(1):
	
		refresh_variables()	# refreshing vaiables after every loop cycle.
		
       
       #If the active_ paramter is set by the user to 1, the node will get to the 
       #active state. 
	
		if active_ == 1:
			
			if f == True:
				actio_client_set_goal() # The new goal position will be set.
       
       #ROS has builtin time and duration primitive types, which rospy provides as the rospy.
       
				rospy.Timer(rospy.Duration(40), CallBack_reset) # reset functio will initiated
				
				f1 = False
				f2 = True
			
	    
		else:
			# idle state of the robot 
			if f1 == False and f2 == False:
				
				print("OPERATION 1: Autonomously driving is STOPEED")
				print("\n")
				f1 = True
			
			
			if f1 == False and f2 == True:
				
				# checking whether robot reached at target or not
				if goalf == True:
					#  If the goal is reached and then the operation 1 is stopped
					print("OPERATION 1: Autonomously driving is STOPEED"+state)
					f1 = True
					f2 = False
					goalf = False
			
				else:
 
#If the robot's failed to reached at the target position when the user initiated the robot's #operations or else the time is exceeded.

					print("TIME EXCEEDED, ROBOT FAILED TO RAH THE TARGET & OPERATION 1 STOPEED!! "+state)
					client.cancel_goal()
					f1 = True
					f2 = False
				
		
		# present robot position		
		if(i % 10 == 0):
		
			print("X-Coordinate "+ str(position_.x)+ "Y-Coordinate" + str(position_.y), end = '\r')
		i = i+1
	    		
	rate.sleep() # rate.sleep() will dynamically choose the correct amount of time to sleep to respect the given frequency.
     
#this is used to execute some code only if the file was run directly, and not imported.
if __name__ == '__main__':
	main()            
