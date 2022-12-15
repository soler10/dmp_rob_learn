#!/usr/bin/env python3

import sys
import rospy
import time
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState
import tf
from ur_ikfast import ur_kinematics
#import kineik

import roboticstoolbox as rtb
import numpy as np
#pel dmp:
import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *
import sys
import random

#global variables:
num_bases = 10 

class UR3Reach():

    """ Move a robot joint to a given position. """

    def __init__(self):

        # Menu choices
        self.joints = {
            1: 'shoulder_pan_joint',
            2: 'shoulder_lift_joint',
            3: 'elbow_joint',
            4: 'wrist_1_joint',
            5: 'wrist_2_joint',
            6: 'wrist_3_joint'
        }
        # Initialize a dictionary to store joint position/set_point/error
        self.joint = {}
        self.pos={}
        self.traj=[]
        # Initialize TF listener
        self.listener = tf.TransformListener()
        # Run publishers
        [rospy.Publisher('/ur3/' + v + '_position_controller/command',
         Float64, queue_size=1) for v in self.joints.values()]
        # Set rate
        self.rate = rospy.Rate(10)
        a=np.array([3])

    def state_callback(self, msg):

        """ Stores in a dictionary the set point
        of a joint, its actual position and the pose error
        measured as the difference between the desired position
        and the current one.
        :return:
        """

        self.joint['set_point'] = msg.set_point
        self.joint['process_value'] = msg.process_value
        self.joint['error'] = msg.error
        
        
    def state_callback2(self, msg):

        """ Stores in a dictionary the set point
        of a joint, its actual position and the pose error
        measured as the difference between the desired position
        and the current one.
        :return:
        """

        self.pos['name'] = msg.name
        
        self.pos['position'] = msg.position
        self.pos['tpm'] = msg.effort

    def wait_for_confirmation(self, timeout, max_error):

        """ Tracks the execution of a robot's action. It waits
        until the robot succesfully reaches a desired position with some
        given tolerance. If moving the robot is taking more time than
        a certain maximum waiting time (arg in seconds), then it stops
        waiting and returns a 'False' flag to indicate the robot
        didn't reach the final position.
        :return: Bool
        """

        # Store current time
        start = time.time()
        elapsed = 0
        # The error has to be lower than the maximum and elapsed time
        # should not exceed the maximum waiting time
        while abs(self.joint['error']) > max_error and not elapsed > timeout:
            elapsed = time.time() - start
            self.rate.sleep()
        # Return a confirmation flag
        if elapsed > timeout:
            return False
        else:
            return True

    def reach_destination(self, joint, query):
        """ Creates a subscriber to a given joint state,
        and waits until the robot has reached a desired position.
        If the robot wasn't able to get there, it prints
        a warning messsage to notify the user the task wasn't completed.
        :return:
        """

        joint = str(joint)
        topic = '/ur3/' + joint + '_position_controller/state'
        sub = rospy.Subscriber(topic, JointControllerState, self.state_callback)
        # Make sure the subscriber has information
        while not self.joint:
            self.rate.sleep()
        # Make sure the final pose matches the query
        while self.joint['set_point'] != query:
            self.rate.sleep()
        # Make sure the robot reaches the actual position (here we pass a max timeout
        # in case the robot gets stuck - see function 'wait_for_confirmation')
        result = self.wait_for_confirmation(timeout=5, max_error=5e-3)
        if not result:
            rospy.logwarn("Max timeout! Joint couldn't reach destination point.")
        # Unsubscribe to the topic until next call
        sub.unregister()
        
        #self.reach_destination(self.joints[query_joint], query_position)
   
    def get_pose(self):
    	
    	#topic = '/ur3/' + joint + '_position_controller/state'
    	topic='/ur3/joint_states'
    	sub = rospy.Subscriber(topic, JointState, self.state_callback2)
    	# Make sure the subscriber has information
    	#sys.stdout.write('aaaaaaaaaaaaaaaaaaaaaaah '+format(self.pos)+'\n')
    	while not self.pos:
            self.rate.sleep()
    	# Make sure the final pose matches the query
    	set_p= self.pos['position']
    	
    	sub.unregister()



    
    def publish_pos(self, joint, query):
    	""" Creates a publisher to a given joint and sends
    	the desired position of it.:return:"""
    	topic = '/ur3/' + joint + '_position_controller/command'
    	pub = rospy.Publisher(topic, Float64, queue_size=1)
    	pub.publish(query)
    

	


    def print_menu_choices(self, n):

        """ Shows in the terminal the list of joints
        that can be actionable.
        :return:
        """

        sys.stdout.write('\r' + '='*17 + ' QUERY #{} '.format(n) + '='*17 + '\n')
        sys.stdout.write('\r Please, select one of the following joints: \n')
        sys.stdout.write("\n".join("{}\t{}".format(k, v) for k, v in self.joints.items()) + "\n")

    def print_summary_report(self, trans, rot):

        """ Shows in the terminal a summary report indicating
        the joint's goal position, its actual current position,
        the error in the position relative to the desired set-point,
        and the position and orientation of the end effector relative
        to the base.
        :return:
        """

        sys.stdout.write('\r' + '** SUMMARY **' + '\n')
        sys.stdout.write('\r' +
            '=> Goal position: {}'.format(round(self.joint['set_point'], 4)) + '\n' +
            '=> Current position: {}'.format(round(self.joint['process_value'], 4)) + '\n' +
            '=> Error: {}'.format(round(self.joint['error'], 4)) + '\n' +
            '=> Position of the end effector relative to the base {}'.format(trans) + '\n' +
            '=> Orientation of the end effector relative to the base {}'.format(rot) + '\n'
        )

    def runner_trajectory(self):

        """ Main executor. The program runs until it's interrupted. """

        new_traj=str(input('New trajectory demo:[y/n] '))
        query = 1
        ur3_arm = ur_kinematics.URKinematics('ur3')
        ###############################################################################
        #Configure the initial point
        self.get_pose()
        joint_angles_ini=self.pos['position']
        pose_quat = ur3_arm.forward(joint_angles_ini)
        #pose quat: A vector of 7 values: a 3x1 translation (tX), and a 1x4 quaternion (w + i + j + k) as input for inverse!!!!!!!!!!!!!!
        new_pos=ur3_arm.inverse(pose_quat, True)
        sucre=new_pos
        for i in range(6):
        	query_joint=i+1
        	query_position=pose_quat[i]
        	self.publish_pos(self.joints[query_joint], query_position)
        	#self.reach_destination(self.joints[query_joint], query_position)
        
        
        #raw_input('Please, press <enter> to run the program...')
        # Clean terminal
        #sys.stderr.write("\x1b[2J\x1b[H")
        if new_traj == 'Y' or new_traj == 'y':
        	print('keep pressing s to save the points')
        	traj=[]
        	sys.stdout.write('Want to save another point? \n')
        	#save=str(input('save the point?[s/d] \n'))
        	sys.stdout.write('here\n')
        	save='s'
        	while save == 's':
        		#choose new point
        		self.print_menu_choices(query)
        		query_joint = int(input('Enter the associated joint number: '))
        		if query_joint not in self.joints.keys():
        			sys.stdout.write('\033[1;31m') # Print in RED
        			sys.stdout.write('\r' + 'Please, enter a valid number.' + '\n')
        			sys.stdout.write('\033[0;0m') # Reset color
        			continue
        			
        		move= str(input('Enter up or down: [u/d]: '))
        		#troba ultima join value
        		self.get_pose()
        		joint_angles=self.pos['position']
        		query_position=joint_angles[query_joint-1]
        		#query_position=joint
        		#sys.stdout.write('query_position'+format(query_position))
        		if move == 'u':
        			query_position = query_position + 0.05
        		else:
        			query_position=query_position - 0.05
        		query_position=float(query_position)
        		# Send position to robot
        		self.publish_pos(self.joints[query_joint], query_position)
        		self.reach_destination(self.joints[query_joint], query_position)
        		save = str(input('save next point? [s/d] '))        		
        		
        		
        		trans, rot = self.listener.lookupTransform("base_link", 'wrist_3_link', rospy.Time(0))
        		pa= self.listener.lookupTransform("base_link", 'wrist_3_link', rospy.Time(0))
        		sys.stdout.write('paaaa'+format(pa)+'\n')       		
        		traj.append(pa)
        		print('omg')
        		
        		      	
        	
        	print(traj)
        	sys.stdout.write('trajectoria o ke'+format(traj)+'\n')
        	
        		
		
        		
        	
        else:
        	traj=[[0.441, 0.1142, 0.0315,0.1297, 0.9504, 0.0028, 0.2826],[0.4103, 0.1982, 0.0316,0.0355, 0.9583, 0.0307, 0.282],[0.3874, 0.24, 0.0317,-0.0147, 0.9588, 0.0454, 0.2801],[0.3617, 0.277, 0.0317,-0.0617, 0.9571, 0.0591, 0.277],[0.3319, 0.3119, 0.0319,-0.11, 0.9528, 0.0728, 0.2733],[0.3379, 0.3125, 0.0662,-0.1061, 0.9481, 0.0787, 0.2891],[0.3437, 0.3203, 0.111,-0.1045, 0.9221, 0.1076, 0.3566],[0.3454, 0.3161, 0.1566,-0.0977, 0.9145, 0.1143, 0.3757],[0.2996, 0.3632, 0.1589,-0.161, 0.8966, 0.1489, 0.3847],[0.2798, 0.3788, 0.1587,-0.1833, 0.8924, 0.1593, 0.3804],[0.2741, 0.3777, 0.2028,-0.1818, 0.8765, 0.1767, 0.4092],[0.2675, 0.3726, 0.2472,-0.1764, 0.857, 0.1953, 0.443],[0.26, 0.3675, 0.2953,-0.1655, 0.8164, 0.2285, 0.5039],[0.2344, 0.3451, 0.3688,-0.153, 0.7745, 0.2575, 0.5571],[0.1679, 0.2872, 0.4846,-0.1234, 0.6693, 0.3145, 0.6617],[0.1056, 0.3154, 0.4847,-0.1918, 0.6533, 0.3806, 0.6257],[0.0656, 0.2655, 0.5356,-0.166, 0.5825, 0.4182, 0.6769],[0.0051, 0.1893, 0.5835,-0.134, 0.4916, 0.458, 0.7285],[-0.0704, 0.092, 0.6092,-0.0903, 0.374, 0.4928, 0.7805],[-0.1264, 0.0214, 0.6053,-0.0615, 0.2907, 0.5134, 0.8051],[-0.2148, -0.0899, 0.5613,-0.01, 0.1408, 0.5367, 0.8319]]
        	
        	#inipose=[0.32, 0.2, 0.04
        return traj
        

	
#	
               	
            	

if __name__ == '__main__':
	rospy.init_node('ur3_reach_pose')
	ur3_reach = UR3Reach()
	
	#define a trajectory:
	traject=ur3_reach.runner_trajectory()
	#sys.stdout.write('trajectory: '+format(traject)+' \n')
	
	
		
