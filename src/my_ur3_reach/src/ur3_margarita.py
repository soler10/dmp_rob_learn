#!/usr/bin/env python3

import sys
import rospy
import time
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState
import tf
from ur_ikfast import ur_kinematics
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

        new_traj=int(input('write 1 for new trajectory demo: '))
        query = 1
        ur3e_arm = ur_kinematics.URKinematics('ur3')
        ###############################################################################
        #Configure the initial point
        self.get_pose()
        joint_angles_ini=self.pos['position']
        pose_quat = ur3e_arm.forward(joint_angles_ini)
        pose_quat[0]=0.5
        pose_quat[1]=0.01
        pose_quat[2]=0.03
        new_pos=ur3e_arm.inverse(pose_quat, True)
        sucre=new_pos
        for i in range(6):
        	query_joint=i+1
        	query_position=pose_quat[i]
        	self.publish_pos(self.joints[query_joint], query_position)
        	#self.reach_destination(self.joints[query_joint], query_position)
        
        
        #raw_input('Please, press <enter> to run the program...')
        # Clean terminal
        sys.stderr.write("\x1b[2J\x1b[H")
        if new_traj == 1:
        	trans, rot = self.listener.lookupTransform("base_link", 'wrist_3_link', rospy.Time(0))
        	# Round numbers in the lists for 'nicer' printing
        	trans =  [round(x, 4) for x in trans]
        	rot =  [round(x, 4) for x in rot]
        	sys.stdout.write('\r' +'=> Actual position of the end effector relative to the base {}'.format(trans) + '\n' +'=> Actual orientation of the end effector relative to the base {}'.format(rot) + '\n')
        	sys.stdout.write('Write desired new position of EE: \n')
        	x_pos = float(input('position relative to base x: '))
        	y_pos = float(input('position relative to base y: '))
        	z_pos = float(input('position relative to base z: '))
        	sys.stdout.write('position: '+str(x_pos)+' '+str(y_pos)+' '+str(z_pos)+'\n')
        	#look for the value of the joints for a given position
        	self.get_pose()
        	joint_angles=self.pos['position']
        	#tenim valors joints actuals
        	pose_quat = ur3e_arm.forward(joint_angles)
        	actual_pose=ur3e_arm.inverse(pose_quat, True)[0]
        	#setting new values for desired position
        	pose_quat[0]=x_pos
        	pose_quat[1]=y_pos
        	pose_quat[2]=z_pos
        	new_pos=ur3e_arm.inverse(pose_quat, True)
        	if new_pos.size<1:
        		new_pose=sucre
        		sys.stdout.write('not a feasible pose, default pose set \n')
        	sys.stdout.write('new joint vals: '+format(new_pos)+' end of new pose\n')
        	
        	for i in range(6):
        		query_joint=i+1
        		query_position=pose_quat[i]
        		self.publish_pos(self.joints[query_joint], query_position)
        		#self.reach_destination(self.joints[query_joint], query_position)
        	query += 1
        	traj=[[0.2938, 0.2651, 0.0542],[0.3285, 0.2202, 0.0554],[0.3381, 0.2271, 0.1717],[0.3401, 0.2232, 0.2292],[0.3263, 0.2207, 0.2673],[0.377, 0.2347, 0.2784],[0.4173, 0.2471, 0.3376],[0.4716, -0.1127, 0.3378], [0.381, -0.2998, 0.3381]]
        	
        else:
        	traj=[[0.2938, 0.2651, 0.0542],[0.3285, 0.2202, 0.0554],[0.3381, 0.2271, 0.1717],[0.3401, 0.2232, 0.2292],[0.3263, 0.2207, 0.2673],[0.377, 0.2347, 0.2784],[0.4173, 0.2471, 0.3376],[0.4716, -0.1127, 0.3378], [0.381, -0.2998, 0.3381]]
        	#inipose=[0.32, 0.2, 0.04
        return traj
        

	
#####################################################################################################            
#funcions del dmpi 


#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain, 
                   D_gain, num_bases):
    demotraj = DMPTraj()
    
        
    for i in range(len(traj)):
        pt = DMPPoint();
        pt.positions = traj[i]
        demotraj.points.append(pt)
        demotraj.times.append(dt*i)
            
    k_gains = [K_gain]*dims
    d_gains = [D_gain]*dims
        
    print( "Starting LfD...")
    rospy.wait_for_service('learn_dmp_from_demo')
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(demotraj, k_gains, d_gains, num_bases)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    print( "LfD done" )   
            
    return resp;


#Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


#Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    print( "Starting DMP planning...")
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                   seg_length, tau, dt, integrate_iter)
           
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)
        
    print ("DMP planning done"  )
            
    return resp;

############### funcions del dmp #############################
def dmp_plan():
    #Create a DMP from a 2-D trajectory now 3-D
    dims = 3                
    dt = 1.0                
    K = 100                 
    D = 2.0 * np.sqrt(K)      
    num_bases = 8  #number of basis functions        
    traj = [[1.0,1.0,1.0],[2.0,2.0,2.0],[3.0,4.0, 4.0],[6.0,8.0,8.0]]
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)
    #sys.stdout.write('here the resp -> '+format(resp)+'\n')
    #sys.stdout.write('here the resp.dmp_list 0 '+format(resp.dmp_list[0])+'\n')
    #Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)

    #Now, generate a plan
    x_0 = [0.0,0.0]          #Plan starting at a different point than demo 
    x_dot_0 = [0.0,0.0]   
    t_0 = 0                
    goal = [8.0,7.0,8.0]         #Plan to a different goal than demo
    goal_thresh = [0.2,0.2,0.2]
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * resp.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1  
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh,                        seg_length, tau, dt, integrate_iter)
    #sys.stdout.write('plan crec: '+format(plan)+' \n')

    return plan
            
               	
            	

if __name__ == '__main__':
	rospy.init_node('ur3_reach_pose')
	ur3_reach = UR3Reach()
	traject=ur3_reach.runner_trajectory()
	#sys.stdout.write('runner ha acabao \n')
	plan=dmp_plan()
	points_0=plan.plan.points[0]
	type(points_0)
	sys.stdout.write('plan crec: '+format(points_0)+' \n')
	
		
