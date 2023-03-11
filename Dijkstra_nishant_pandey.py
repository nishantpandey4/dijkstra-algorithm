#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ENPM 661
Project 2

@author: Nishant Pandey
UID: 119247556
Github link of repository: https://github.com/nishantpandey4/projec2_661
"""

#Import all packages

import numpy as np
from queue import PriorityQueue
from cv2 import VideoWriter, VideoWriter_fourcc 
import Node

#All necessary functions for execution

def create_line(point1, point2, x, y, t):
    """
    Calculate the distance from a point to a line defined by two other points.

    Args:
        point1 (tuple): The first point on the line.
        point2 (tuple): The second point on the line.
        x (float): The x-coordinate of the point.
        y (float): The y-coordinate of the point.
        tolerance (float, optional): A small value used to avoid division by zero.
            Defaults to 1e-6.

    Returns:
        float: The distance from the point to the line.
    """

    a=(point2[1] - point1[1])
    b=x - point1[0]
    c=point2[0] - point1[0]
    d=point1[1] + t - y
    dis = ( a*b ) / ( c) + (d)
    
    return dis
#following function creates a map

def create_map():

    map = np.zeros((250,600)) # given dimension (y,x)
    
    r = 0                                       # radius
    c = 5                                       # clearance
    t = r + c                                   # Total clearance
    
    # Clearance is also added in the same color code, so the obstacles are appear to be inflated.
   
    for i in range(map.shape[1]):
        for j in range(map.shape[0]):
            #Iscoceles triangle 
            
           if (i>(460-t)and i<(510+t)and j>(25-t) and j< (225+2*t) and create_line((460,225),(510,125),i,j,t) > 0
               and create_line((460,25),(510,125),i,j,t) < 0):
                map[j,i]=1
            # Lower rectangle

           if (i > (100-t) and i < (150+t) and create_line((100,150),(150,150),i,j,t) < 0 
                and create_line((100,250),(150,250),i,j,t) > 0):
                map[j,i]=1
                
            #Upper rectangle
            
           if (i > (100-t) and i < (150+t) and create_line((100,0),(150,0),i,j,t) < 0 
                and create_line((100,100),(150,100),i,j,t) > 0):
                map[j,i]=1
                
            # Hexagonal Obastacle 
            
           if (i > (235.05-t) and i < (364.95+t) and create_line((235.05,87.5),(300,50),i,j,t) < 0 
                and create_line((300,50),(364.95,87.5),i,j,t) < 0 
                and create_line((235.05,162.5),(300,200),i,j,t) > 0 
                and create_line((300,200),(364.95,162.5),i,j,t) > 0):
                map[j,i]=1
            
            # Boundaries of the map
           if (i > 0 and i < t):
                map[j,i] = 1
           if (i < 600 and i > (600-t)):
                map[j,i] = 1
           if (j > 0 and j < t):
                map[j][i] = 1
           if (j < 250 and j >(250 - t)):
                map[j][i] = 1    
    return map
