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
# To check weather the point collides with the obstacle or not 
def check_collision(point,map):
    flag = False
    
    if map[point[1],point[0]] == 1:
        flag = True
    
    return flag
# Getting the goal node

def goal(map):
    
    flag = False
    while not flag:
        goal_node = [int(item) for item in input("\n goal node:").split(',')]#Taking input from the user 
        goal_node[1] = 250 - goal_node[1]#converting to given frame of the map
        if (len(goal_node) == 2 and (0 <= goal_node[0] <= 600) and (0 <= goal_node[1] <= 250)):#setting boundries
            if not check_collision(goal_node,map):
                flag = True
            else:
                print("collision with obstacle \n")
        else:
            print(" enter a valid goal node.\n")
            flag = False
        
    return goal_node
#Explores the neighbors of the current node
def exp(node,map):

    x = node.x
    y = node.y
    #defining action sets Actions Set = {(1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,1), (1,-1), (-1,-1)} 
    m = [(x, y + 1),(x + 1, y),(x, y -1),(x - 1, y),(x + 1, y + 1),(x + 1, y - 1),(x - 1, y - 1),(x - 1, y + 1)]
    v_paths = []
    for pos, m in enumerate(m):
        if not (m[0] >= 600 or m[0] < 0 or m[1] >= 250 or m[1] < 0):
            if map[m[1]][m[0]] == 0:
                cost = 1.4 if pos > 3 else 1
                v_paths.append([m,cost])

    return v_paths
# performs Dijkstra algorithm
def Dijkstra_algorithm(goal_node, map):
 
    flag = False #Initializing flag input 
    while not flag:
        start_node = [int(item) for item in input("\n start node: ").split(',')]#taking user unput for start node
        start_node[1] = 250 - start_node[1]#converting it from open cv frame to given map frame

        if (len(start_node) == 2 and (0 <= start_node[0] <= 600) and (0 <= start_node[1] <= 250)):# setting the boundries for start node
            if not check_collision(start_node,map):#checking for collision with any obstacle
                flag = True
            else:   
                print("collision with obstacle \n")
        else:
            print("enter a valid start node.\n")
            flag = False
            
    print("\n Executing algo\n")

    q = PriorityQueue()                                                              # Priority queue for open nodes
    visited = set([])                                                                # Set conataining visited nodes
    node_objects = {}                                                                # dictionary of nodes
    distance = {}                                                                    # distance 
    
    distance = {(i, j): float('inf') for i in range(map.shape[1]) for j in range(map.shape[0])}

    distance[str(start_node)] = 0                                                    # Start node has cost of 0
    visited.add(str(start_node))                                                     # Add start node to visited list
    node = Node.Node(start_node,0,None)                                              # Create instance of Node
    node_objects[str(node.pos)] = node                                               # Assigning the node value in dictionary
    q.put([node.cost, node.pos])                                                     # Inserting the start node in priority queue

    while not q.empty():                                                             # Iterate until the queue is empty
        node_temp = q.get()                                                          # Pop node from queue
        node = node_objects[str(node_temp[1])]  
                                     
        # Check of the node is the goal node
        if node_temp[1][0] == goal_node[0] and node_temp[1][1] == goal_node[1]:      
            print(" Success\n")
            node_objects[str(goal_node)] = Node.Node(goal_node,node_temp[0], node)
            break
        
        for next_node, cost in exp(node,map):                                    # Explore neighbors

            if str(next_node) in visited:                                            # Check if action performed next node is already visited
                temp = cost + distance[str(node.pos)]                           # Cost to come
                if temp < distance[str(next_node)]:                             # Update cost
                    distance[str(next_node)] = temp
                    node_objects[str(next_node)].parent = node

            else:                                                                    # If next node is not visited
                visited.add(str(next_node))
                a_c = cost + distance[str(node.pos)]
                distance[str(next_node)] = a_c
                new_node = Node.Node(next_node, a_c, node_objects[str(node.pos)])
                node_objects[str(next_node)] = new_node
                q.put([a_c, new_node.pos])
    # used for backtracking
    reverse_path = []                                                                    # Empty reversed path list 
    goal = node_objects[str(goal_node)]                                              # Get the goal from dictionary
    reverse_path.append(goal_node)                                                       # Add the goal to reversed path list 
    parent_node = goal.parent                                                        # Get parent of goal node
    while parent_node:                              
                reverse_path.append(parent_node.pos)                                             # Add coordinates of parent node
                parent_node = parent_node.parent                                             # Update parent
            
    path = list(reversed(reverse_path))                                                  # Forward path 

    return node_objects, path
#Displaying animation 
def Animate(node_objects, path, map):
    
    print(" Creating animation video...")                                              
    fourcc = VideoWriter_fourcc(*'mp4v')
    video = VideoWriter('./animation_video.mp4', fourcc, float(300), (600, 250)) #.mp4 format
    
    nodes = node_objects.values()                                                    # Get the values from dictionary(objects of class Node)
    nodes = list(nodes)
    img = np.dstack([map.copy() * 0, map.copy() * 0, map.copy() * 255])              # Convert binary map image to RGB
    img = np.uint8(img)
    video.write(img)
    
    
    for i in range(len(nodes)):                                                      # Add visited nodes to video frame
        img[nodes[i].pos[1], nodes[i].pos[0], :] = np.array([255,255,0])
        video.write(img)
        
    for i in range(len(path) - 1):                                                   # Add generated path to video frame 
        img[path[i][1], path[i][0], :] = np.array([255,0,255])
        video.write(img)
        
    video.release()
    print("Animation video saved.")
 
    

map = create_map()
goal_node = goal(map)
node_objects, path = Dijkstra_algorithm(goal_node, map)
Animate(node_objects, path, map)

