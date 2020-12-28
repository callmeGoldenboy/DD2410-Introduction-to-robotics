#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot,sqrt

rospy.init_node('controller',anonymous=True)
# Use to transform between frames
tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)

# The exploration simple action client
goal_client = actionlib.SimpleActionClient('get_next_goal',irob_assignment_1.msg.GetNextGoalAction)
goal_client.wait_for_server()
# The collision avoidance service client
rospy.wait_for_service('get_setpoint')
control_client = rospy.ServiceProxy('get_setpoint',GetSetpoint) ###########################needs to be checked 
# The velocity command publisher
pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
# The robots frame
robot_frame_id = "base_link"
# Max linear velocity (m/s)
max_linear_velocity = 0.3
# Max angular velocity (rad/s)
max_angular_velocity = 0.5

def move(path):
    global control_client, robot_frame_id, pub
    rate = rospy.Rate(10)
    # Call service client with path
    response = control_client(path)
    # Transform Setpoint from service client
    transform = tf_buffer.lookup_transform(robot_frame_id,response.setpoint.header.frame_id,rospy.Time(0))
    transformed_setpoint = tf2_geometry_msgs.do_transform_point(response.setpoint,transform) #transform the setpoint to base_link frame
    # Create Twist message from the transformed Setpoint
    twist_msg = Twist()
    twist_msg.angular.z = max_angular_velocity * atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)
    twist_msg.linear.x = max_linear_velocity * sqrt(transformed_setpoint.point.y ** 2 + transformed_setpoint.point.x ** 2)
    if twist_msg.angular.z > max_angular_velocity:
        twist_msg.angular.z = max_angular_velocity
    if twist_msg.linear.x > max_linear_velocity:
        twist_msg.linear.x = max_linear_velocity
    # Publish Twist
    pub.publish(twist_msg)
    rate.sleep()
    # Call service client again if the returned path is not empty and do stuff again
    i = 0
    while response.new_path.poses:
          i = i + 1
          print("There are poses "+ str(i))
          response = control_client(response.new_path)
          transformer = tf_buffer.lookup_transform(robot_frame_id,response.setpoint.header.frame_id,rospy.Time(0))
          new_setpoint = tf2_geometry_msgs.do_transform_point(response.setpoint,transformer) #transform the setpoint to base_link frame
          # Create Twist message from the transformed Setpoint
          msg = Twist()
          msg.angular.z = max_angular_velocity * atan2(new_setpoint.point.y, new_setpoint.point.x)  ###########doesnt feel right
          msg.linear.x = max_linear_velocity * sqrt(new_setpoint.point.y ** 2 + new_setpoint.point.x ** 2) ##############doesnt feel right
          if msg.angular.z > max_angular_velocity:
              msg.angular.z = max_angular_velocity
          if msg.linear.x > max_linear_velocity:
              msg.linear.x = max_linear_velocity
          pub.publish(msg)
          rate.sleep()
    # Send 0 control Twist to stop robot
    stop = Twist()
    stop.angular.z = 0
    stop.linear.x = 0
    pub.publish(stop)
    rate.sleep()
    # Get new path from action server

def get_path():
    try:
        global goal_client
        i = 0
        while True:
            # Get path from action server
            goal_client.wait_for_server()
            goal = irob_assignment_1.msg.GetNextGoalGoal()
            goal_client.send_goal(goal)
            goal_client.wait_for_result()
            result = goal_client.get_result()
            i = i + 1
            print("Got path " + str(i))
            if result.path == [] or result.gain == 0:
                break
            else:
                # Call move with path from action server
                move(result.path)
    except Exception as e:
        print(e)

if __name__ == "__main__":
    # Init node
    rospy.init_node('controller',anonymous=True)
    # Init publisher
    pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    # Init simple action client
    goal_client = actionlib.SimpleActionClient('get_next_goal',irob_assignment_1.msg.GetNextGoalAction)
    rospy.wait_for_service('get_setpoint')
    # Init service client
    control_client = rospy.ServiceProxy('get_setpoint',GetSetpoint) ###########################needs to be checked 
    # Call get path
    get_path()
    # Spin
    rospy.spin()
