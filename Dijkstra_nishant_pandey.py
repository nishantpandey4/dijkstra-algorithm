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