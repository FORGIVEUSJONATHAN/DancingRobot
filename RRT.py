#! /usr/bin/env python3

import numpy as np
import cv2
import copy


class Node:
    def __init__(self, value, children):
        self.value = value
        self.children = children
        self.parent = None
        
    def addchild(self,child):
        child.parent = self
        self.children.append(child)
        
    def getparent(self):
        return self.parent
    
    def size(self):
        sum = 0
        for i in self.children:
            sum += i.size()
        return sum
    def makenodelist(self,f):
        f(self)
        if self.children != []:
            for i in self.children:
                i.makenodelist(f)

class RRT:
    def __init__(self,start,target,image_path_base):
        self.image_path = f"{image_path_base}/map.pgm"
        self.image_path_save = f"{image_path_base}/map1.pgm"
        self.image = cv2.imread(self.image_path,0)
        self.map_width = self.image.shape[1]
        self.map_height = self.image.shape[0]
        
        self.visualization = copy.deepcopy(self.image)
        
        self.start = start
        self.target = target
        self.start_node = Node(start,[])
        self.target_node = Node(target,[])
        self.change_node_color_large(self.start_node)
        self.change_node_color_large(self.target_node)

        self.path_tree = self.start_node
        self.node_near = None
        self.node_random = None
        self.goal_flag = False
    
    def distance(self,x1,y1,x2,y2):
        px = float(x2)-float(x1)
        py = float(y2)-float(y1)
        return (px**2 + py**2)**0.5
    
    def node_distance(self,node1,node2):
        (x1,y1) = node1.value
        (x2,y2) = node2.value
        return self.distance(x1,y1,x2,y2)

    ## find the nearest node in the function
    def nearest_node(self):
        (x2,y2) = self.node_random.value
        l = []
        self.path_tree.makenodelist(lambda x: l.append(x))
        min_dist = np.inf
        node_record = None
        for i in l:
            (x1,y1) = i.value
            dist = self.distance(x1,y1,x2,y2)
            if dist < min_dist:
                min_dist = dist
                node_record = i
        return node_record
    
    def move_node(self, step_size = 15):
        dist = self.node_distance(self.node_near,self.node_random)
        if dist > step_size: ## if the root can grow.
            (x1,y1) = self.node_near.value
            (x2,y2) = self.node_random.value
            (px,py) = (x2-x1,y2-y1)
            angle = np.arctan2(py,px)
            (x_new,y_new) = (int(np.rint(x1 + step_size * np.cos(angle))),
                            int(np.rint(y1 + step_size * np.sin(angle))))
            if not self.checkObstacle(x1,y1,x_new,y_new):
                ## add the target node if it is in the range
                if self.distance(x_new,y_new,self.target[0],self.target[1]) < step_size:
                    
                    self.node_near.addchild(self.target_node)
                    ## the goal is reached
                    self.goal_flag = True
                    return self.target_node
                ## else add the new node
                else:
                    new_node = Node((x_new,y_new),[])
                    self.node_near.addchild(new_node)
                    self.change_node_color(new_node)
            

    
    def checkObstacle(self,x1,y1,x2,y2):
        for i in range(10):
            k = i/10
            x = x1*k + x2*(1-k)
            y = y1*k + y2*(1-k)
            yindex = int(np.rint(y))
            xindex = int(np.rint(x))
            for i in range(-4,5):
                for k in range(-4,5):
                    if self.image[yindex+i,xindex+k]==0: ## it is occupied
                        return True ## there is an obstacle
        return False ## no obstacles

    def random_sample(self):
        x = int(np.random.uniform(0, self.map_width))
        y = int(np.random.uniform(0, self.map_height))
        self.node_random = Node((x,y),[])
    
    def change_node_color(self, node):
        (x,y) = node.value
        for i in [-1,0,1]:
            for k in [-1,0,1]:
                self.visualization[y+i,x+k] = 0 ## make it black
        
        cv2.imwrite(self.image_path_save, self.visualization)

    def change_node_color_large(self, node):
        (x,y) = node.value
        for i in [-2,-1,0,1,2]:
            for k in [-2,-1,0,1,2]:
                self.visualization[y+i,x+k] = 0 ## make it black
        cv2.imwrite(self.image_path_save, self.visualization) 
        
    def find_path(self, iteration = 5000):
        for i in range(iteration):
            if not self.goal_flag:
                self.random_sample()
                self.node_near = self.nearest_node()
                self.possible_node = self.move_node()
                
            else:
                return self.possible_node, self.path_tree

                    