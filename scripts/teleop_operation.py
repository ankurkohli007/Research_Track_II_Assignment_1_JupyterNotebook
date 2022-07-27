#!/usr/bin/env python3


"""
.. module:: teleop_operation
    :platform: Unix
    :synopsis: Python module for the teleop operation
.. moduleauthor:: Ankur Kohli

This node allows to use the robot in manual mode. The robot can be controlled with keyboards.
BY receiving the inputs, the robot moves forward/backward, right/left. 


Publishes to:
 /cmd_vel
 
The script is based on the standard ROS `teleop_twist_keyboard.py <http://wiki.ros.org/teleop_twist_keyboard>`.

This node is constantly checking which keys are pressed on a PC keyboard and based on the pressed keys, publishes twist messages on the ``/cmd_vel`` topic. Twist message defines what should be the linear and rotational speeds of a mobile robot.

Some functions and changes were made to the code to merge it with the `avoidence.py` publisher and to manage the alternity of the activation state.

Just like all the other programs, I had to manage the alternity of the *idle* to the *active* state through the use of an if-statement. Since the node also includes the functionalities of the third modality, the statement will set the node to an active modality if the ``active`` param is either set to 3 or 2.

A new function named ``stop_motion`` to the ``PublishThread`` class. This function will make the robot stop once the driving modality gets switched. The function will set the linear and angular velocity to 0 with the `twist` message through the ``/cmd_vel`` topic.
"""
# importing required libraries 

from __future__ import print_function

import threading
from sensor_msgs.msg import LaserScan
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from final_assignment.msg import Avoid

from geometry_msgs.msg import Twist
import time
from std_srvs.srv import *
import sys, select, termios, tty


# initializing messae
msg = """
This node makes the robot move using keynoard and this will publish to twist.
---------------------------
Moving around:
           k   
      j    i    l 

For Holonomic mode (strafing), hold down the shift key:
---------------------------
           K    
      U    I    L
anything else : stop

q/z : accelerate/decelrate velocity by 10%
w/x : accelerate/decelrate only linear velocity by 10%
e/c : accelerate/decelrate only angular velocity by 10%

CTRL-C to quit
"""

l = True # determines the appearance of a wall at the left side of robot
r = True # determines the appearance of a wall at the right side of robot
f = True # determines the appearance of a wall at the front side of robot
fl = True # determines the appearance of a wall at the front-left side of robot
fr = True # determines the appearance of a wall at the front-right side of robot
moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        'k':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        'K':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

# controls for speed 
speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

# PublishingThread() is defined, class with its relative methods.
class PublishThread(threading.Thread):
	
    # this self variable represents the instance of the object itself
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        # Publishing to a topic. We can also create a handle to publish messages to a topic using the rospy.Publisher class.
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
    
        """
          Args:
	  self
	  	 
	  No Returns
        """
        i = 0
        # The most common usage patterns for testing for shutdown in rospy
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Let's wait for the subscribers to connect".format(self.publisher.name))
            rospy.sleep(0.5)
            i = i + 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Ohh!! Received shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
    
	
        """
          Args:
	  self
	  x: for x coordinates
          y: for y coordinates
          z: for z coordinates
          th: for threshold
          speed: for speed of the robot
          turn: for turning variable	 
	  No Returns
        """    	
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
    	 
    	 
        """
          Args:
	  self
	  	 
	  No Returns
        """
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    # this function is used to stop the robot
    def stop_motion(self):
        """
        This halt_robot function is added to the "PublishThread" class. This function will make 
        the robot movement halt once the driving 
        operation is switched. This function will set the linear and angular velocity to 0 with 
        the "Twist" message through the "/cmd_vel" 
        topic.
        """
        twist = Twist()
        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        
        self.publisher.publish(twist)

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publishing
            self.publisher.publish(twist)

        # stopped message is published when hread exits
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)
        
# Defining getKey, with which we'll get the input from the keyboard by the user.
def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def CallBack_avoidance(msg):
	"""
	The node which is subscribes to the custom topic "custom_controller" implemented for publishing the "Avoidingwall.msg" message which 
	contains the info about the walls surrounding the robot. 
	
	The CallBack will subscribed and sets some local variables which is equal to the published fields of the custom messages. 
	
	This will use such variables function to change some inputs from the keyboards and also to prevent the user to drive the robot into 
	walls.

	gloabal keyword allows us to modify the variable outside of the current scope. It is used to create a global variable and make 
	changes to the variable in a local context.
	"""
	global l
	global r
	global f
	global fl
	global fr
	
	r = msg.right
	f = msg.front
	l = msg.left
	fr = msg.fright
	fl = msg.fleft
	
# This function will avoid a ertain command if the direction of the robot is obstructed
def new_command(dictionary):
    """
    This function will going to use pop command and this pop command will remove some keys from the commands right away. 
    
    This removal will inccurs as per the values received by the last callback function to the "\custom_controller" topic. 
    
    These recieved values from the teleop node are relocated in the local variables such as
    
    r:
    	  1 = the wall is not close to the right of the robot. The user will be able to turn right. 
    	  0 = the wall is close to the right of the robot. The user will not be able to turn right.
    
    l:
    	  1 = the wall is not close to the left of the robot. The user will be able to turn left. 
    	  0 = the wall is close to the left of the robot. The user will not be able to turn left.
    
    f:
    	  1 = the wall is not close to the front of the robot. The user will be able to drive straight. 
    	  0 = the wall is close to the front of the robot. The user will not be able to drive straight.
    	  
    fr:
    	  1 = the wall is not close to the fron-right of the robot. The user will be able to turn front-right. 
    	  0 = the wall is close to the front-right of the robot. The user will not be able to turn front-right.
    
    fl:
    	  1 = the wall is not close to the front-left of the robot. The user will be able to turn front-left. 
    	  0 = the wall is close to the front-left of the robot. The user will not be able to turn front-left.	
    
    """
    global l
    global r
    global f
    global fl
    global fr
    
	# This will check if the wall are turned on in any combinations. Also, this function will disable the corrisponding directions 
	# command
    if not f == True and not r == True and not l == True:
        dictionary.pop('i')
        dictionary.pop('j')
        dictionary.pop('l')
        
    elif not l == True and not f == True and r == True:
        dictionary.pop('i')
        dictionary.pop('j')
        
    elif l == True and not f == True and not r == True:
        dictionary.pop('i')
        dictionary.pop('l')
        
    elif not l == True and f == True and not r == True:
        dictionary.pop('l')
        dictionary.pop('j')
        
    elif l == True and not f == True and r == True:
        dictionary.pop('i')
        
    elif not l == True and f == True and r == True:
        dictionary.pop('j')
        
    elif l == True and f == True and not r == True:
        pdictionary.pop('l')
        






def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

# this is used to execute some code only if the file was run directly, and not imported.
if __name__=="__main__":

    rospy.init_node('teleop_avoid')     # Node is initialized
    active_=rospy.get_param("/active")  # Active parameter value is assigned to the local variable.
    rospy.Subscriber("custom_controller", Avoid, CallBack_avoidance)	# custom topic is subscribed.
    
    # Flags are declared to supervised present robot's operations.
    f1 = True									
    f2 = False									
    
    
    # initial parametrs setup
    settings = termios.tcgetattr(sys.stdin) 
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.1)
    
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    rate = rospy.Rate(5)
    pub_thread.wait_for_subscribers()
    pub_thread.update(x, y, z, th, speed, turn)
    moveBindings_temp = {}
    
    # Printing robot's position informations and user interfaces commands.
    print(msg)				
    print(vels(speed,turn))		
    
    while not rospy.is_shutdown():
        	
        active_=rospy.get_param("/active")        # Refreshing the operation parameters value

        moveBindings_temp = moveBindings.copy()   # Copying of the actual moveBindings command into a temporary
        
        
        # Checking whether the 2nd and 3rd operation is active or not
        if active_ == 2 or active_ == 3:		
        	
            if f2 == False:
            	print("ACTIVATED OPERATION "+ str(active_))
            	
            f2 = True
            key = getKey(key_timeout)

	    # Here, we calculates a new commands and also taking care of the messages recived from the avoidance node.
            new_command(moveBindings_temp)				
            
            
            if key in moveBindings_temp.keys():

                x = moveBindings_temp[key][0] 
                y = moveBindings_temp[key][1]
                z = moveBindings_temp[key][2]
                th = moveBindings_temp[key][3]

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skiping updating /cmd_vel if key_reset and robot's stopped already
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)
            flag = 1

        else:
            if f1 == True:
                pub_thread.stop_motion() 
                print("TELEOP OPERATION STOPEED!!")
            f1 = False
            f2 = False

        rate.sleep() # rate.sleep() will dynamically choose the correct amount of time to sleep to respect the given frequency
            



