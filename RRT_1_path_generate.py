#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
Title: Rapidly-Exploring Random Tree (RRT) algorithm
Time: September 17, 2018 during MSR Hackathon
Author: Chenge Yang
Email: chengeyang2019@u.northwestern.edu
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

K=10000
delta_q=0.2
q=[]
fig = plt.figure()
q.append([50.0,50.0])


for i in range(1,K+1):
    q_rand=[(100*np.random.rand()),(100*np.random.rand())]

    min_length=5000
    min=0
    for j in range(0,i):
        length=(q_rand[0]-q[j][0])**2+(q_rand[1]-q[j][1])**2
        if length<min_length:
            min_length=length
            min=j
    q.append([q[min][0]+(q_rand[0]-q[min][0])*delta_q,q[min][1]+(q_rand[1]-q[min][1])*delta_q])

    verts=[q[min],q[i]]
    codes=[Path.MOVETO,Path.LINETO]
    path = Path(verts, codes)
    patch = patches.PathPatch(path)
    ax = fig.add_subplot(111)
    ax.add_patch(patch)

ax.set_xlim([0,100])
ax.set_ylim([0,100])
plt.show()
