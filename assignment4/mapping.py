#!/usr/bin/env python3


# Python standard library
from math import cos, sin, atan2, fabs,sqrt

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap

class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()


        """
        Fill in your solution here
        """
     
        rect_indexes = [10000,10000,-10000,-10000]  #index in the order minium x, minimum y, maximum x, maximum y
        obstacle_indexes = []
        #iterate through the laser ranges 
        for i in range(0,len(scan.ranges)):
            #check if the length of the range is bigger than the min range but smaller than the max range, because then it means that there was an obstacle that was hit by the laser 
            if scan.ranges[i] > scan.range_min and scan.ranges[i] < scan.range_max:
              #convert the laser scan ranges and bearing to coordinates in the laser frame
              laser_frame_x = cos(robot_yaw + scan.angle_min + scan.angle_increment * i)
              laser_frame_y = sin(robot_yaw + scan.angle_min + scan.angle_increment * i)
              #converte the coordinates to the map frame. Start from the origin and add the robots position as well as the range length multiplied by the angle between the point (laser_frame_x,laser_frame_y)
              map_frame_x =  pose.pose.position.x + scan.ranges[i] * laser_frame_x - origin.position.x
              map_frame_y =  pose.pose.position.y + scan.ranges[i] * laser_frame_y - origin.position.y
              #convert the coordinates to map indices (use int(x) to convert from float to int and not round(x)) by dividing the coordinates with the resolution of the grid map 
              x = int(map_frame_x/resolution)
              y = int(map_frame_y/resolution)
              obstacle_indexes.append([x,y])
              #calculate the free cells from the start to the end grid cell
              #first get the index of the robot in terms of the grid map and then use raystrace function to get the free cells 
              robot_index_x = int((pose.pose.position.x - origin.position.x)/resolution)
              robot_index_y = int((pose.pose.position.y - origin.position.y)/resolution)
              free_cells = self.raytrace((robot_index_x,robot_index_y),(x,y))
              for cell in free_cells:
                  self.add_to_map(grid_map,cell[0],cell[1],self.free_space)

              if x < rect_indexes[0]:
                  rect_indexes[0] = x

              if x > rect_indexes[2]:
                  rect_indexes[2] = x

              if y < rect_indexes[1]:
                  rect_indexes[1] = y

              if y > rect_indexes[3]:
                  rect_indexes[3] = y 
              #fill in the occupied cells
              #self.add_to_map(grid_map,x,y,self.occupied_space)


        #the obstacle MUST be added afte the free space. After 15 tries, it worked :)

        for obstacle in obstacle_indexes:
            self.add_to_map(grid_map,obstacle[0],obstacle[1],self.occupied_space)
            
        """
        For C only!
        Fill in the update correctly below.
        """ 
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = rect_indexes[0]
        # The minimum y index in 'grid_map' that has been updated
        update.y = rect_indexes[1]
        # Maximum x index - minimum x index + 1
        update.width = rect_indexes[2] - rect_indexes[0] + 1
        # Maximum y index - minimum y index + 1
        update.height = rect_indexes[3] - rect_indexes[1] + 1
        # The map data inside the rectangle, in row-major order.
        update.data = []
        print(rect_indexes)

        """for j in range(rect_indexes[1],rect_indexes[1] + update.height):
            for i in range(rect_indexes[0], rect_indexes[0] + update.width):
                if self.is_in_bounds(grid_map,rect_indexes[0] + i, rect_indexes[1] + j):
                    update.data.append(grid_map[rect_indexes[0] + i, rect_indexes[1] + j]) """


        for i in range(0,update.height):
            for j in range(0,update.width):
                    update.data.append(grid_map[rect_indexes[0] + j, rect_indexes[1] + i])

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """
        """
        Fill in your solution here
        """
        for i in range(0,grid_map.get_width()):
            for j in range(0,grid_map.get_height()):
                #first check if the cell is occupied space
                if grid_map[i,j] == self.occupied_space:
                    #if it is we need to find all the cells that are within self.radius from the occupied gridcell
                    #for that, we check all the cells that are - self.radius and + self.radius away from the current occupied grid cell
                    for width in range(i-self.radius,i+self.radius):
                        for height in range (j - self.radius, j + self.radius):
                              #calculate the distance between the current cell and the occupied cell
                              d = sqrt((i - width)**2 + (j - height)**2)
                              if d <= self.radius and grid_map[width,height] != self.occupied_space:
                                  self.add_to_map(grid_map,width,height,self.c_space)

        # Return the inflated map
        return grid_map
