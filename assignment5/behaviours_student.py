# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.


import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse

#THIS WAS NOT IMPORTED BEFORE, I'M JUST TRYING TO IMPLEMENT THE E PART AS A BEHAVIOUR TREE
from geometry_msgs.msg import PoseStamped,Vector3Stamped
from std_srvs.srv import Empty, SetBool, SetBoolRequest


checkers = {"picked":False,"placed":False,"read_msg":False, "placed_correctly":False}
class counter(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        rospy.loginfo("Initialising counter behaviour.")

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(counter, self).__init__(name)

    def update(self):

        # increment i
        self.i += 1

        # succeed after count is done
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS


class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        rospy.loginfo("Initialising go behaviour.")
        # action space
        #self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular

        # become a behaviour
        super(go, self).__init__(name)

    def update(self):
        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()

        # tell the tree that you're running
        return pt.common.Status.RUNNING


class tuckarm(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):

        # already tucked the arm
        if self.finished:
            return pt.common.Status.SUCCESS

        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING

class pick_cube(pt.behaviour.Behaviour):
    """
    The behaviour for picking up the cube. Right now i hard coded the cube pose, but we need to use visual feeback
    """

    def __init__(self):
        rospy.loginfo("Initialising the pick cube behaviour")
        self.pick_server_name = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.pick_cube_server = rospy.ServiceProxy(self.pick_server_name, SetBool)
        rospy.wait_for_service(self.pick_server_name,timeout=30)
        self.picked = False
        self.successful = False
        super(pick_cube,self).__init__("Pick cube")



    def update(self):
        if not checkers["picked"]:
            self.successful = False
            self.picked = False
            checkers["picked"] = True
        if self.picked:
            if self.successful:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE

        else:
            self.successful = self.pick_cube_server().success
            self.picked = True
            return pt.common.Status.RUNNING

        """if not self.detected:
            return pt.common.Status.RUNNING

        if self.picked:
            return pt.common.Status.SUCCESS

        elif self.detected:
            pick_request = self.pick_cube_server()
            if pick_request.success == True:
                self.picked = True
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE"""



class place_cube(pt.behaviour.Behaviour):
    """
    The placing cube behaviour
    """
    def __init__(self):
        rospy.loginfo("Initialising the place cube behaviour")
        self.place_server_name = rospy.get_param(rospy.get_name() + '/place_srv')
        #rospy.wait_for_service(self.place_server_name,timeout=30)
        self.placed = False
        self.successful = False
        super(place_cube,self).__init__("Place cube")


    def update(self):
        if not checkers["placed"]:
            self.placed = False
            self.succesful = False
            checkers["placed"] = True
        if self.placed:
            if self.successful:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE
        else:
            self.place_cube_server = rospy.ServiceProxy(self.place_server_name, SetBool)
            self.place_cube_server.wait_for_service()
            place_req = self.place_cube_server()
            self.successful = place_req.success
            rospy.loginfo("self.sucessful: %s",str(self.successful))
            self.placed = True
            return pt.common.Status.RUNNING

class is_placed(pt.behaviour.Behaviour):
    """
    Check whether the cube has been placed on the second table or not
    """
    def __init__(self):
        #we need to check the aruco pose topic n see if there are messages there
        self.marker_pose_topic = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        super(is_placed,self).__init__("Is placed?")

    def update(self):
        try:
            #see if there is at least one message on the topic, then the marker is visible and we should return SUCCESS
            if not checkers["read_msg"]:
                checkers["read_msg"] = True
                msg = rospy.wait_for_message(self.marker_pose_topic,PoseStamped,1)
                checkers["placed_correctly"] = True
                return pt.common.Status.SUCCESS
            else:
                if checkers["placed_correctly"]:
                    return pt.common.Status.SUCCESS
                else:
                    return pt.common.Status.FAILURE
        except:
            checkers["placed_correctly"] = False
            return pt.common.Status.FAILURE


class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raisesthe head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movehead, self).__init__("Lower head!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING
