#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dubins import *
import math
import random
import numpy as np 


STEP_SIZE = 100  # to be chosen personally
OFFSET = 0.2
dt = 0.01

class Node():
    def __init__(self,x=None,y=None,theta=0,parent=None):
          self.x = x
          self.y = y
          self.parent = parent
          self.theta = theta 
          self.phi = 0
          #the values array has phi at first index and steps at second index
          self.values = []


    def print_parent(self):
        print("Parent of {}, {}, is {} {}".format(self.x,self.y,self.parent.x,self.parent.y))
    
    def __str__(self):
        return ("({},{},{},{})".format(self.x,self.y,self.theta,self.phi))


class Tree():

    def __init__(self,root,goal_node):
        self.root = root  
        self.number_of_nodes = 1 #number of vertices at the start (start position)
        self.nodes = [root]
        self.goal_node = goal_node


#check if a given x,y value is outside of the boundries 
def is_out_of_bounds(car,x,y):
    if car.xlb < x < car.xub and car.ylb < y < car.yub:
        return False
    else:
        return True

#gives a random point within the boundaries
def get_random_point(car):
    x = random.uniform(car.xlb + 0.2, car.xub - 0.2)  # dont want to get a point exactly on the boundry 
    y = random.uniform(car.ylb + 0.2 ,car.yub - 0.2)
    if check_if_point_inside_obstacle(x,y,car):
        x,y = get_random_point(car)
    
    return x,y


def check_if_point_inside_obstacle(x,y,car):
    for obstacle in car.obs:
      d = math.sqrt((x-obstacle[0])**2 + (y-obstacle[1])**2)
      if  d  <= obstacle[2] + OFFSET:
          return True
    return False

def get_distance_between_points(x0,y0,x1,y1):
    return math.sqrt((x1 -x0)**2 + (y1 - y0)**2)


def find_closest_node(tree,x,y):
    closest_node = None
    min_distance = 1000000000000
    for node in tree.nodes:
        d = get_distance_between_points(node.x,node.y,x,y)
        if d < min_distance:
            min_distance = d
            closest_node = node

    return closest_node

#get the steering angle between the neareset node and the random node we generated 
def get_phi_angle(random_point,nearest_node):
    vector = [math.cos(nearest_node.theta),math.sin(nearest_node.theta),0]
    vector2 = [random_point[0] - nearest_node.x, random_point[1] - nearest_node.y,0]
    normal_product = np.linalg.norm(vector) * np.linalg.norm(vector2)
    cos_phi = np.dot(vector,vector2)/normal_product
    sin_phi = np.cross(vector,vector2)[2]/normal_product
    phi = math.atan2(sin_phi,cos_phi)
    if phi > math.pi/4:
        phi = math.pi/4
    if phi < -math.pi/4:
        phi = -math.pi/4
    
    return phi

#returns the path and values (values being an array of arrays, where each array is of the form [phi,step] where phi is the steering angle to reach the given node and step is the amount of stpes to take towards the node
def get_path_and_values_from_tree(tree):
    #start from goal node and traverse backgwards till we hit the root node
    path = [tree.goal_node]
    values = [tree.goal_node.values]
    while path[-1].parent is not None:
        print("This is the parent of the current node")
        print(path[-1].parent)
        values.append(path[-1].parent.values)
        path.append(path[-1].parent)

    #we dont want the last element since it is repeated
    
    final_values = list(reversed(values[:-1]))
    final_path = list(reversed(path[:-1]))
    return final_path,final_values
    
#given a point x,y and the car instance, check whether the point is outside the boundaries       
def check_if_out_of_bounds(x,y,car):
        out_of_bounds = True
        if car.xlb < x < car.xub and car.ylb < y < car.yub:
            out_of_bounds = False
        return out_of_bounds 

def create_tree(tree,car):
    x, y, theta = 0,0,0
    x_arr,y_arr,theta_arr = [0] * STEP_SIZE ,[0] * STEP_SIZE,[0] * STEP_SIZE
    while True:
        x,y = get_random_point(car)
        print("Random point is x:{} , y: {}".format(x,y))
        closest_node = find_closest_node(tree,x,y)
        print(closest_node)
        phi = get_phi_angle([x,y], closest_node)
        print("Phi is {}".format(phi))
        x = closest_node.x
        y = closest_node.y
        theta = closest_node.theta
        reached_goal = False
        hit_obstacle = False
        out_of_bounds = False
        #we start from the closest node and step towards the point x,y until we hit an obstacle, are at the point or are within the goal area
        #at each x,y,theta returned from the step function, we check if the point is within boundaries, within an obstacle or within the goal range
        for counter in range(0,STEP_SIZE):
            x_arr[counter],y_arr[counter],theta_arr[counter] = step(car,x,y,theta,phi)
            x,y,theta = x_arr[counter],y_arr[counter],theta_arr[counter]
            #check if the point is close enough to the goal node 
            if get_distance_between_points(x,y,tree.goal_node.x,tree.goal_node.y) < 0.15:
                print("distance reached")
                reached_goal = True

            if check_if_point_inside_obstacle(x,y,car):
                print("Point inside obstacle")
                hit_obstacle = True

            if is_out_of_bounds(car,x,y):
                out_of_bounds = True 
            if reached_goal or hit_obstacle or out_of_bounds:
                break
        if reached_goal:
            tree.goal_node.parent = closest_node
            tree.goal_node.phi = phi 
            tree.goal_node.values =[phi,counter]
            break
        
        #if we hit obstacle at first step then we just continue to next iteration of loop 
        if counter == 0:
            print("counter is 0")
            continue 
        if counter != STEP_SIZE - 1:
            print("We didnt reach step size. counter is: {}".format(counter))
            counter = int(counter/4) # divide counter by four to put car in safe point again if we didnt take STEP_SIZE amount of steps towards the point 
        
        #once we get as close as possible to the generated point, we create a node at that point and connect it to nearest node
        node = Node(x,y,theta)
        node.parent = closest_node
        node.phi = phi
        node.values = [phi,counter]
        tree.nodes.append(node)
    
    path,values = get_path_and_values_from_tree(tree)
    time_values = [0]
    control_values = []
    for i in range(0,len(values)):
        time_values.append(time_values[-1] + values[i][1] * dt)
        control_values.append(values[i][0])
    return control_values,time_values,path


def solution(car):

    ''' <<< write your code below >>> '''
    controls=[0]
    times=[0,1]
    root = Node(car.x0,car.y0,0)
    goal_node = Node(car.xt,car.yt,0)
    tree = Tree(root,goal_node)
    print("root x: {}, root y: {}".format(tree.root.x,tree.root.y))
    controls, times ,path= create_tree(tree,car)

    ''' <<< write your code above >>> '''

    return controls, times


if __name__ == "__main__":
    car = Car()
    solution(car)
