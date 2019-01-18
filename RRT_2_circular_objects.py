#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
Title: Rapidly-Exploring Random Tree (RRT) algorithm
Time: September 17, 2018 during MSR Hackathon
Author: Chenge Yang
Email: chengeyang2019@u.northwestern.edu
"""

import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches


class RRT(object):


    def __init__(self, K):

        self.SIZE = 100 #size of the world (100*100)
        self.q_init = [10,10] #start point
        self.q_final = [90,90] #End point
        self.delta_q = 0.2
        self.fig = plt.figure()

        self.qs = [] #store the [x,y] coordinate of each point
        self.qs_parent = [] #store the index of each point's parent
        self.qs.append(self.q_init)
        self.qs_parent.append(0)
        self.q_new = self.q_init


    def generate_circles(self, num, mean, std):
        """
        This function generates /num/ random circles with a radius mean defined by
        /mean/ and a standard deviation of /std/.

        The circles are stored in a num x 3 sized array. The first column is the
        circle radii and the second two columns are the circle x and y locations.
        """
        self.circles = np.zeros((num,3))
        # generate circle locations using a uniform distribution:
        self.circles[:,1:] = np.random.uniform(mean, self.SIZE-mean, size=(num,2))
        # generate radii using a normal distribution:
        self.circles[:,0] = np.random.normal(mean, std, size=(num,))
        return self.circles


    #check if q_init and q_final are inside the circles
    def initial_collosion_check(self):
        for circle in self.circles:
            length1 = math.sqrt((self.q_init[0]-circle[1])**2+(self.q_init[1]-circle[2])**2)
            length2 = math.sqrt((self.q_final[0]-circle[1])**2+(self.q_final[1]-circle[2])**2)
            if length1 < circle[0] or length2 < circle[0]:
                return False
        return True


    def random_vertex_generate(self):
        self.q_rand = [100*np.random.rand(), 100*np.random.rand()]
        return self.q_rand


    #if q_rand is inside a circle, return False as collision happens
    def collision_check1(self):
        for circle in self.circles:
            length = math.sqrt((self.q_rand[0]-circle[1])**2+(self.q_rand[1]-circle[2])**2)
            if length < circle[0]:
                return False
        return True


    def nearest_vertex_check(self):
        min_length = 20000
        for index, q in enumerate(self.qs):
            length = (self.q_rand[0]-q[0])**2+(self.q_rand[1]-q[1])**2
            if length < min_length:
                min_length = length
                self.index_near = index
                self.q_near = q
        return self.index_near, self.q_near


    #if the line between q_near and q_random overlap with a circle, return False
    def collision_check2(self):
        for circle in self.circles:
            #calculate the [x,y] coordinate of the intersection of the tangent
            u_up = ((circle[1]-self.q_near[0])*(self.q_rand[0]-self.q_near[0])+(circle[2]-self.q_near[1])*(self.q_rand[1]-self.q_near[1]))
            u_down = ((self.q_near[0]-self.q_rand[0])**2+(self.q_near[1]-self.q_rand[1])**2)
            u = u_up / u_down
            x = self.q_near[0]+u*(self.q_rand[0]-self.q_near[0])
            y = self.q_near[1]+u*(self.q_rand[1]-self.q_near[1])
            #check if the intersection point is on the line between q_near and q_rand
            if self.q_near[0] < self.q_rand[0]:
                #on the line
                if x > self.q_near[0] and x < self.q_rand[0]:
                    length = math.sqrt((circle[1]-x)**2+(circle[2]-y)**2)
                    if length < circle[0]:
                        return False
            else:
                if x > self.q_rand[0] and x < self.q_near[0]:
                    length = math.sqrt((circle[1]-x)**2+(circle[2]-y)**2)
                    if length < circle[0]:
                        return False
        return True


    def new_point_generate(self):
        #generate q_new according to delta_q and write into qs & q_parent
        x = self.q_near[0]+(self.q_rand[0]-self.q_near[0])*self.delta_q
        y = self.q_near[1]+(self.q_rand[1]-self.q_near[1])*self.delta_q
        self.q_new = [x,y]
        self.qs.append(self.q_new)
        self.qs_parent.append(self.index_near)

        #plot the new generated line
        verts=[self.q_new, self.q_near]
        codes=[Path.MOVETO,Path.LINETO]
        path = Path(verts, codes)
        patch = patches.PathPatch(path)
        ax = self.fig.add_subplot(111)
        ax.add_patch(patch)


    def connection_check(self):
        for circle in self.circles:

            #calculate the [x,y] coordinate of the intersection of the tangent
            u_up = ((circle[1]-self.q_new[0])*(self.q_final[0]-self.q_new[0])+(circle[2]-self.q_new[1])*(self.q_final[1]-self.q_new[1]))
            u_down = ((self.q_new[0]-self.q_final[0])**2+(self.q_new[1]-self.q_final[1])**2)
            u = u_up / u_down
            x = self.q_new[0]+u*(self.q_final[0]-self.q_new[0])
            y = self.q_new[1]+u*(self.q_final[1]-self.q_new[1])

            #check if the intersection point is on the line between q_near and q_rand
            if self.q_new[0] < self.q_final[0]:
                #on the line
                if x > self.q_new[0] and x < self.q_final[0]:
                    length = math.sqrt((circle[1]-x)**2+(circle[2]-y)**2)
                    if length < circle[0]:
                        return False
            else:
                if x > self.q_final[0] and x < self.q_new[0]:
                    length = math.sqrt((circle[1]-x)**2+(circle[2]-y)**2)
                    if length < circle[0]:
                        return False

        self.qs.append(self.q_final)
        self.qs_parent.append(len(self.qs)-2)
        return True


    def figure_generate(self):
        #plot the circles
        #ax = plt.subplot(111)
        ax = self.fig.add_subplot(111)
        fcirc = lambda x: patches.Circle((x[1],x[2]), radius=x[0], fill=True, alpha=0.5, fc='k', ec='k')
        circs = [fcirc(x) for x in self.circles]
        for c in circs:
            ax.add_patch(c)

        #plot the initial and final new_point
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

        #set figure parameters
        plt.xlim([0,self.SIZE])
        plt.ylim([0,self.SIZE])
        plt.xticks(range(0,self.SIZE + 1))
        plt.yticks(range(0,self.SIZE + 1))
        ax.set_aspect('equal')
        ax.set_xticklabels([])
        ax.set_yticklabels([])

        plt.show()


if __name__=="__main__":
    K = 5000
    rrt = RRT(K)
    #generate and plot circles
    rrt.generate_circles(10, 8, 3)
    #check for the start and end points
    while (rrt.initial_collosion_check() == False):
        rrt.generate_circles(10, 8, 3)

    for i in range(K):
        #check if the new vertex can connect with the final point
        #if yes, jump out of the loop
        if rrt.connection_check() == True:
            break
        #generate random vertex
        q_rand = rrt.random_vertex_generate()
        #check if the random vertex is inside the circles
        if rrt.collision_check1() == False:
            continue
        #search for the nearest existing vertex
        rrt.nearest_vertex_check()
        #check if the line between q_rand and q_near intersects with the circles
        if rrt.collision_check2() == False:
            continue
        #generate new vertex according to the delta_q
        rrt.new_point_generate()

    rrt.figure_generate()
