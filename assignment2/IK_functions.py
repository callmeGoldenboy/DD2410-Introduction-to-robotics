#! /usr/bin/env python3
import numpy as np
import math

#GLOBAL VALUES FOR SCARA ROBOT (link lengths)
l0 = 0.07
l1 = 0.3
l2 = 0.35

#given a point [x,y,z], what configuration of q does the scara  robot need to reach said point?
def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    #we know that the end frame and base fram are the same height, meaning the z value for the end effector is the same as the input z value
    cos2 = ((x-l0)**2 + y**2 - l1**2 - l2**2)/(2*l1*l2)  #cosinus of theta2
    sin2 = math.sqrt(1-cos2**2)  #sinus of theta2
    #to calcualte thetha 1 we use k1 and k2 where k1 and k2 are defined the following way (given in the JJ Craig book)
    k1 = l1 + l2*cos2
    k2 = l2 * sin2
    theta2 = np.arctan2(sin2,cos2)  #we can calculate theta2 by using the inverse of tangent 
    theta1 = np.arctan2(y,x-l0) - np.arctan2(k2,k1)

    return [theta1,theta2,z]

#GLOBAL VALUES FOR KUKA ROBOT
H = 0.311
M= 0.39
L = 0.4
N = 0.078

#given a point [x,y,z], which is the desired position of the end effector, a 3x3 matrix R (which is the desired orientation of the end effector, and joint positions (q1,...,q7) which are the current joint positions,
#calculate the vectore q, containing the 7 joint values that give the desired pose of the end-effector
def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions
    """
    Fill in your IK solution here and return the seven joint values in q
    """
    error_tolerance = 0.5
    while True:
      jacobian_matrix,T = get_jacobian_matrix(joint_positions) #get the jacobian matrix and the final transformation matrix 
      pseudo_inverse = np.linalg.pinv(jacobian_matrix) #get the pseudo inverse of the jacobian matrix
      """
      To find inverse kinematics theta, start with an approximation theta_hat = theta + e, 
      where e is an error factor. We then know that X + e_x = K(theta + e_theta). The pose of the end effector (X) plus an error factor of the pose (e_x) is equal to the forward kinematics of the sum of the true configuration (theta) and the error of the configuration (e_theta)
      X + e_x = K(theta + e_theta)
      K(theta) + e_x = K(theta + e_theta)
      with linear approxiamtion we get that
      e_x ~ Jacobian(theta) * e_theta,
      which leads to the following equation, assuming the jacobian is invertible:
      e_theta ~ invers_Jacobian(theta) * e_x
      """
      x_1 = np.dot(T,np.array([0,0,N,1]))
      x_1 = x_1[:3]
      x_1[2] += H # get the translational section from the transformation matrix 
      e_x = x_1 - point #translational error
      ### now we need to get the rotational error e_r
      e_r = get_rotational_error(R,T)
      e_tot = np.concatenate((e_x,e_r))
      e_theta = np.dot(pseudo_inverse,e_tot)
      q = q - e_theta
      if np.linalg.norm(e_theta) < error_tolerance:
          break
    return q

def get_rotational_error(R,T):
    rotational_error =  0.5*(np.cross(np.dot(R, [1, 0, 0]), np.dot(T,[1, 0, 0, 0])[:3]) + np.cross(np.dot(R, [0, 1, 0]), np.dot(T,[0, 1, 0, 0])[:3])+ np.cross(np.dot(R, [0, 0, 1]), np.dot(T,[0, 0, 1, 0])[:3]))
    return rotational_error


def get_jacobian_matrix(joints):
  #h_matrix at i+1 contains the homogenous transformation matrix A from i to i-1

  #could have written in with for loop and shit but aint nobody got time for that 
  a0 = [0,0,0,0] #just a formality so we start at the right index and can get a1 matrix by searching h_matrix[1]
  a1 = [[math.cos(joints[0]),0,math.sin(joints[0]),0],[math.sin(joints[0]),0,-math.cos(joints[0]),0],[0,1,0,0],[0,0,0,1]]
  a2 = [[math.cos(joints[1]),0,-math.sin(joints[1]),0],[math.sin(joints[1]),0,math.cos(joints[1]),0],[0,-1,0,0],[0,0,0,1]]
  a3 = [[math.cos(joints[2]),0,-math.sin(joints[2]),0],[math.sin(joints[2]),0,math.cos(joints[2]),0],[0,-1,0,L],[0,0,0,1]]
  a4 = [[math.cos(joints[3]),0,math.sin(joints[3]),0],[math.sin(joints[3]),0,-math.cos(joints[3]),0],[0,1,0,0],[0,0,0,1]]
  a5 = [[math.cos(joints[4]),0,math.sin(joints[4]),0],[math.sin(joints[4]),0,-math.cos(joints[4]),0],[0,1,0,M],[0,0,0,1]]
  a6 = [[math.cos(joints[5]),0,-math.sin(joints[5]),0],[math.sin(joints[5]),0,math.cos(joints[5]),0],[0,-1,0,0],[0,0,0,1]]
  a7 = [[math.cos(joints[6]),-math.sin(joints[6]),0,0],[math.sin(joints[6]),math.cos(joints[6]),0,0],[0,0,1,0],[0,0,0,1]]
  
  h_matrix = [a0,a1,a2,a3,a4,a5,a6,a7]
  
  #now we calculate the transformation matrix from each joint to the next, all the way to the end effector frame
  #ti is the transformation matrix from frame i to the base frame
  
  #t_matrix contains the transformation matrices from neighboring joints
  t1 = a1
  t2 = np.dot(a1,a2)
  t3 = np.dot(t2,a3)
  t4 = np.dot(t3,a4)
  t5 = np.dot(t4,a5)
  t6 = np.dot(t5,a6)
  t7 = np.dot(t6,a7)  # this is also the final transformation matrix from the end effector fram to the base frame
  
  t_matrix = [0,a1,t2,t3,t4,t5,t6,t7]
  z0 = [0,0,1]
  extract = [0,0,1,0] #extract the 3rd coulmn, which contains the relative vectors expressing the oreintaiton of the joint in space
  z1 = np.dot(a1,extract)[:3]
  z2 = np.dot(t2,extract)[:3]
  z3 = np.dot(t3,extract)[:3]
  z4 = np.dot(t4,extract)[:3]
  z5 = np.dot(t5,extract)[:3]
  z6 = np.dot(t6,extract)[:3]
  
  p0 = [0,0,0]
  extract = [0,0,0,1]  #extract the coloumn 4th coloumn, which contains the relative translations that we want
  #the p values here are the contriubuitions of each joint to the linear veclocity of end effector
  p1 = np.dot(t1,extract)[:3]
  p2 = np.dot(t2,extract)[:3]
  p3 = np.dot(t3,extract)[:3]
  p4 = np.dot(t4,extract)[:3]
  p5 = np.dot(t5,extract)[:3]
  p6 = np.dot(t6,extract)[:3]

  #here is the sum of all the values of singular joint contribuition, giving us the final linear velocity
  p_final = np.dot(t7,extract)[:3]

  #calculate the jacobian matrix the follwoing way:
  # each coloumn i in the jacobain matrix can be seen as the motion caused by join i

  j_matrix=np.transpose([np.concatenate((np.cross(z0,p_final-p0), z0)),np.concatenate((np.cross(z1,p_final-p1), z1)),np.concatenate((np.cross(z2,p_final-p2), z2)),np.concatenate((np.cross(z3,p_final-p3), z3)),np.concatenate((np.cross(z4,p_final-p4), z4)),np.concatenate((np.cross(z5,p_final-p5), z5)),np.concatenate((np.cross(z6,p_final-p6), z6))])

  return j_matrix,t7

