#!/usr/bin/env python

import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest
#from cob_srvs.srv import SetString, SetStringRequest
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class StateMachine(object):
    def __init__(self):

        self.node_name = "Student SM"

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.pick_server_name = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.place_server_name = rospy.get_param(rospy.get_name() + '/place_srv')
        self.cube_pose = rospy.get_param(rospy.get_name() + '/cube_pose').split(',')
        #self.cube_pose_topic = '/manipulation_client/marker_pose_topic'
        self.cube_pose_topic = rospy.get_param(rospy.get_name()+ '/aruco_pose_topic')
        #the picking and placing poses of the robot
        #self.pick_pose_robot = rospy.get_param(rospy.get_name()+ '/pick_pose')
        #self.place_pose_robot = rospy.get_param(rospy.get_name()+ '/place_pose')
        # Subscribe to topics
        # here we need to add attributes for the different subscribers we are going to use
        self.place_marker_pose = rospy.get_param(rospy.get_name() +'/place_pose_topic')

        # Wait for service providers
        rospy.wait_for_service(self.mv_head_srv_nm, timeout=30)
        rospy.wait_for_service(self.pick_server_name, timeout=30)
        rospy.wait_for_service(self.place_server_name, timeout=30)

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        self.cube_pose_publisher = rospy.Publisher(self.cube_pose_topic,PoseStamped,queue_size=10)
        self.place_pose_publisher = rospy.Publisher(self.place_marker_pose,PoseStamped,queue_size=10)
        self.rate = rospy.Rate(10)
        # Set up action clients
        rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to play_motion action server", self.node_name)

        # Init state machine

        self.state = 0
        rospy.sleep(3)
        self.check_states()

    def check_states(self):
        print("running check_states in sm_students.py")
        while not rospy.is_shutdown() and self.state != 4:
            #state 0 tuck the arm
            if self.state == 0:
                rospy.loginfo("%s: Tucking the arm...", self.node_name)
                goal = PlayMotionGoal()
                goal.motion_name = 'home'
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)
                success_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))

                if success_tucking:
                    rospy.loginfo("%s: Arm tuck: ", self.play_motion_ac.get_result())
                    self.state = 1
                else:
                    self.play_motion_ac.cancel_goal()
                    rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
                    self.state = 5

                rospy.sleep(1)

            #state 1: pick the cube
            if self.state == 1:
            	try:
                    msg = PoseStamped()
                    msg.header.frame_id = "base_footprint"
                    msg.header.stamp = rospy.Time.now()
                    msg.header.seq = 1
                    msg.pose.position.x = float(self.cube_pose[0])
                    msg.pose.position.y = float(self.cube_pose[1])
                    msg.pose.position.z = float(self.cube_pose[2])
                    msg.pose.orientation.x = float(self.cube_pose[3])
                    msg.pose.orientation.y = float(self.cube_pose[4])
                    msg.pose.orientation.z = float(self.cube_pose[5])
                    msg.pose.orientation.w = float(self.cube_pose[6])
                    self.cube_pose_publisher.publish(msg)
                    self.rate.sleep()
                    rospy.loginfo("%s: Picking cube...", self.node_name)
                    pick_cube_server = rospy.ServiceProxy(self.pick_server_name, SetBool)  #not sure if its supposed to be SetBool here
                    pick_cube_server.wait_for_service()
                    pick_up_req  = pick_cube_server()

                    if pick_up_req.success == True:
                        self.state = 2
                        rospy.loginfo("%s: Picking cube succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Picking cube failed!", self.node_name)
                        self.state = 999

                    rospy.sleep(3)

                except rospy.ServiceException, e:
                    print "Service call to move_head server failed: %s"%e

            if self.state == 2:
                #move the robot towards the other table
                move_msg = Twist()
                move_msg.angular.z = -1
                converged = False
                cnt = 0
                rospy.loginfo("%s: Moving towards table", self.node_name)
                while not rospy.is_shutdown() and cnt < 31:
                    self.cmd_vel_pub.publish(move_msg)
                    self.rate.sleep()
                    cnt = cnt + 1

                move_msg.linear.x = 1
                move_msg.angular.z = 0
                cnt = 0
                rospy.sleep(2)
                while not rospy.is_shutdown() and cnt < 10:  #i changed this back to 10 cause the robot kept touching the table
                    self.cmd_vel_pub.publish(move_msg)
                    self.rate.sleep()
                    cnt = cnt + 1

                rospy.loginfo("%s: Navigation towards table completed", self.node_name)
                self.state = 3
                rospy.sleep(1)
            # state 3: place the cube
            if self.state == 3:
            	try:
                    rospy.loginfo("%s: Placing cube...", self.node_name)
                    place_cube_server = rospy.ServiceProxy(self.place_server_name, SetBool)  #not sure if its supposed to be SetBool here
                    place_cube_server.wait_for_service()
                    place_req  = place_cube_server()

                    if place_req.success == True:
                        self.state = 4
                        rospy.loginfo("%s: Placing the cube succeded!", self.node_name)
                    else:
                        rospy.logerr("%s: Placing the cube failed!", self.node_name)
                        self.state = 3

                    rospy.sleep(3)

                except rospy.ServiceException, e:
                    print "Service call to place server failed: %s"%e

            # Error handling
            if self.state == 999:
                rospy.logerr("%s: State machine failed. Check your code and try again!", self.node_name)
                return
        rospy.loginfo("%s: State machine finished!", self.node_name)
        return



    def check_states_old(self):

        while not rospy.is_shutdown() and self.state != 4:

            # State 0: Move the robot "manually" to door
            if self.state == 0:
                move_msg = Twist()
                move_msg.linear.x = 1

                rate = rospy.Rate(10)
                converged = False
                cnt = 0
                rospy.loginfo("%s: Moving towards door", self.node_name)
                while not rospy.is_shutdown() and cnt < 25:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                self.state = 1
                rospy.sleep(1)

            # State 1:  Tuck arm
            if self.state == 1:
                rospy.loginfo("%s: Tucking the arm...", self.node_name)
                goal = PlayMotionGoal()
                goal.motion_name = 'home'
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)
                success_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))

                if success_tucking:
                    rospy.loginfo("%s: Arm tuck: ", self.play_motion_ac.get_result())
                    self.state = 2
                else:
                    self.play_motion_ac.cancel_goal()
                    rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
                    self.state = 5

                rospy.sleep(1)

            # State 2:  Move the robot "manually" to chair
            if self.state == 2:
                move_msg = Twist()
                move_msg.angular.z = -1

                rate = rospy.Rate(10)
                converged = False
                cnt = 0
                rospy.loginfo("%s: Moving towards table", self.node_name)
                while not rospy.is_shutdown() and cnt < 5:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                move_msg.linear.x = 1
                move_msg.angular.z = 0
                cnt = 0
                while not rospy.is_shutdown() and cnt < 15:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                self.state = 3
                rospy.sleep(1)

            # State 3:  Lower robot head service
            if self.state == 3:
            	try:
                    rospy.loginfo("%s: Lowering robot head", self.node_name)
                    move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
                    move_head_req = move_head_srv("down")

                    if move_head_req.success == True:
                        self.state = 4
                        rospy.loginfo("%s: Move head down succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Move head down failed!", self.node_name)
                        self.state = 5

                    rospy.sleep(3)

                except rospy.ServiceException, e:
                    print "Service call to move_head server failed: %s"%e

            # Error handling
            if self.state == 5:
                rospy.logerr("%s: State machine failed. Check your code and try again!", self.node_name)
                return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return


# import py_trees as pt, py_trees_ros as ptr

# class BehaviourTree(ptr.trees.BehaviourTree):

# 	def __init__(self):

# 		rospy.loginfo("Initialising behaviour tree")

# 		# go to door until at door
# 		b0 = pt.composites.Selector(
# 			name="Go to door fallback",
# 			children=[Counter(30, "At door?"), Go("Go to door!", 1, 0)]
# 		)

# 		# tuck the arm
# 		b1 = TuckArm()

# 		# go to table
# 		b2 = pt.composites.Selector(
# 			name="Go to table fallback",
# 			children=[Counter(5, "At table?"), Go("Go to table!", 0, -1)]
# 		)

# 		# move to chair
# 		b3 = pt.composites.Selector(
# 			name="Go to chair fallback",
# 			children=[Counter(13, "At chair?"), Go("Go to chair!", 1, 0)]
# 		)

# 		# lower head
# 		b4 = LowerHead()

# 		# become the tree
# 		tree = pt.composites.Sequence(name="Main sequence", children=[b0, b1, b2, b3, b4])
# 		super(BehaviourTree, self).__init__(tree)

# 		# execute the behaviour tree
# 		self.setup(timeout=10000)
# 		while not rospy.is_shutdown(): self.tick_tock(1)


# class Counter(pt.behaviour.Behaviour):

# 	def __init__(self, n, name):

# 		# counter
# 		self.i = 0
# 		self.n = n

# 		# become a behaviour
# 		super(Counter, self).__init__(name)

# 	def update(self):

# 		# count until n
# 		while self.i <= self.n:

# 			# increment count
# 			self.i += 1

# 			# return failure :(
# 			return pt.common.Status.FAILURE

# 		# succeed after counter done :)
# 		return pt.common.Status.SUCCESS


# class Go(pt.behaviour.Behaviour):

# 	def __init__(self, name, linear, angular):

# 		# action space
# 		self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
# 		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

# 		# command
# 		self.move_msg = Twist()
# 		self.move_msg.linear.x = linear
# 		self.move_msg.angular.z = angular

# 		# become a behaviour
# 		super(Go, self).__init__(name)

# 	def update(self):

# 		# send the message
# 		rate = rospy.Rate(10)
# 		self.cmd_vel_pub.publish(self.move_msg)
# 		rate.sleep()

# 		# tell the tree that you're running
# 		return pt.common.Status.RUNNING


# class TuckArm(pt.behaviour.Behaviour):

# 	def __init__(self):

# 		# Set up action client
# 		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

# 		# personal goal setting
# 		self.goal = PlayMotionGoal()
# 		self.goal.motion_name = 'home'
# 		self.goal.skip_planning = True

# 		# execution checker
# 		self.sent_goal = False
# 		self.finished = False

# 		# become a behaviour
# 		super(TuckArm, self).__init__("Tuck arm!")

# 	def update(self):

# 		# already tucked the arm
# 		if self.finished:
# 			return pt.common.Status.SUCCESS

# 		# command to tuck arm if haven't already
# 		elif not self.sent_goal:

# 			# send the goal
# 			self.play_motion_ac.send_goal(self.goal)
# 			self.sent_goal = True

# 			# tell the tree you're running
# 			return pt.common.Status.RUNNING

# 		# if I was succesful! :)))))))))
# 		elif self.play_motion_ac.get_result():

# 			# than I'm finished!
# 			self.finished = True
# 			return pt.common.Status.SUCCESS

# 		# if I'm still trying :|
# 		else:
# 			return pt.common.Status.RUNNING



# class LowerHead(pt.behaviour.Behaviour):

# 	def __init__(self):

# 		# server
# 		mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
# 		self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
# 		rospy.wait_for_service(mv_head_srv_nm, timeout=30)

# 		# execution checker
# 		self.tried = False
# 		self.tucked = False

# 		# become a behaviour
# 		super(LowerHead, self).__init__("Lower head!")

# 	def update(self):

# 		# try to tuck head if haven't already
# 		if not self.tried:

# 			# command
# 			self.move_head_req = self.move_head_srv("down")
# 			self.tried = True

# 			# tell the tree you're running
# 			return pt.common.Status.RUNNING

# 		# react to outcome
# 		else: return pt.common.Status.SUCCESS if self.move_head_req.success else pt.common.Status.FAILURE




if __name__ == "__main__":

	rospy.init_node('main_state_machine')
	try:
		#StateMachine()
		StateMachine()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()