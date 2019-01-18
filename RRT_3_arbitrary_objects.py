#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
Title: Rapidly-Exploring Random Tree (RRT) algorithm
Time: September 17, 2018 during MSR Hackathon
Author: Chenge Yang
Email: chengeyang2019@u.northwestern.edu
"""

import numpy as np
import scipy
import math
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches


class RRT(object):

    def __init__(self, map):
        # Map: 0 for barrier, 1 for free space
        self.map = map[:,:,0].astype(int).T
        self.SIZE = 100 # Size of the world (100*100)

        self.q_init = [40.0, 60.0] # Start point
        self.q_final = [60.0, 40.0] # End point
        self.delta_q = 0.2 # Percentage of length to keep between q_rand and q_near when generating new point q_new
        self.dis_num = 100 # Discretization number of line collision checking

        # Store the final route from the start point to end point
        self.qs = [] # Store the [x,y] coordinate of each point
        self.qs_parent = [] # Store the index of each point's parent
        self.qs.append(self.q_init)
        self.qs_parent.append(0)

        self.fig = plt.figure()


    def random_vertex_generate(self):
        # Generate random vertex
        q_rand = [self.SIZE*np.random.rand(), self.SIZE*np.random.rand()]
        return q_rand


    def collision_check_point(self, point):
        # Check if a point is in collision with the geometry
        if self.map[int(math.floor(point[0])), int(math.floor(point[1]))] == 0:
            # Collision detected
            return True
        else:
            # Non-collision
            return False


    def collision_check_line(self, q_start, q_end):
        # Check if a line is in collision with the geometry

        # Step length
        dx = (q_end[0] - q_start[0]) / self.dis_num
        dy = (q_end[1] - q_start[1]) / self.dis_num
        q = [int(q_start[0]), int(q_start[1])]
        for i in range(self.dis_num):
            x = q_start[0] + i * dx
            y = q_start[1] + i * dy
            # Skip repeated points
            if int(x)==q[0] and int(y)==q[1]:
                continue
            q = [int(x), int(y)]
            if self.collision_check_point(q) == True:
                return True

        # Collision check of q_end
        if self.collision_check_point(q_end) == True:
            return True

        return False


    def nearest_vertex_check(self, q_rand):
        # Search for the nearest existing point

        # Maximum length in the map
        min_length = 2 * self.SIZE ** 2

        # Find the nearest point in Euclidean distance
        for index, q in enumerate(self.qs):
            length = (q_rand[0]-q[0]) ** 2 + (q_rand[1]-q[1]) ** 2
            if length < min_length:
                min_length = length
                index_near = index
                q_near = q
        return index_near, q_near


    def new_point_generate(self, q_near, q_rand, index_near):
        # Generate q_new according to delta_q
        x = q_near[0] + (q_rand[0]-q_near[0]) * self.delta_q
        y = q_near[1] + (q_rand[1]-q_near[1]) * self.delta_q
        q_new = [x,y]

        # Write into qs & q_parent
        self.qs.append(q_new)
        self.qs_parent.append(index_near)

        # Plot the new generated line
        self.new_point_plot(q_near, q_new)

        return q_new


    def connection_check(self, q_new):
        # Check if the new vertex can connect with the final point
        if not rrt.collision_check_line(q_new, self.q_final):
            self.qs.append(self.q_final)
            self.qs_parent.append(len(self.qs)-2)
            # Plot the line connect to the end point
            self.new_point_plot(q_new, self.q_final)
            return True
        else:
            return False


    def new_point_plot(self, q_near, q_new):
        # Plot the new generated line
        verts=[q_near, q_new]
        codes=[Path.MOVETO,Path.LINETO]
        path = Path(verts, codes)
        patch = patches.PathPatch(path)
        ax = self.fig.add_subplot(111)
        ax.add_patch(patch)


    def figure_generate(self):
        # Create figure
        ax = self.fig.add_subplot(111)

        # Plot the N_map
        plt.imshow(self.map.T)

        # Plot the initial and end points
        ax.plot(self.q_init[0], self.q_init[1], '*')
        ax.plot(self.q_final[0], self.q_final[1], '*')

        #plot the route from start point to end point
        i = len(self.qs)-1
        while (True):
            x = [self.qs[i][0], self.qs[self.qs_parent[i]][0]]
            y = [self.qs[i][1], self.qs[self.qs_parent[i]][1]]
            ax.plot(x, y, "r")
            i = self.qs_parent[i]
            if i == 0:
                break

        plt.show()




if __name__=="__main__":
    # Load the N_map and initialize the class
    N_map = plt.imread("images/N_map.png")
    rrt = RRT(N_map)


    # Main loop of RRT
    q_new = rrt.q_init
    while True:
        # Check if the new vertex can connect with the final point
        # If yes, finish the loop
        if rrt.connection_check(q_new) == True:
            break

        # Generate random vertex
        q_rand = rrt.random_vertex_generate()

        # Check if the random vertex is in collision with the geometry
        if rrt.collision_check_point(q_rand) == True:
            continue

        # Search for the nearest point in map
        index_near, q_near = rrt.nearest_vertex_check(q_rand)

        # Check if the line between q_near and q_rand collides with the geometry
        if rrt.collision_check_line(q_near, q_rand) == True:
            continue

        # Generate new vertex according to delta_q
        q_new = rrt.new_point_generate(q_near, q_rand, index_near)


    rrt.figure_generate()
