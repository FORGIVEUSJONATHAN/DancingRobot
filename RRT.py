#! /usr/bin/env python

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

class rrt:
    def __init__(self,start,target,image_path):
        self.image_path = image_path
        self.image = cv2.imread(image_path,0) 
        self.map_width = self.image.shape[1]
        self.map_height = self.image.shape[0]
        print(self.map_width,self.map_height)
        
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
            print("the new points are", x_new,y_new)
            if not self.checkObstacle(x1,y1,x_new,y_new):
#                 print("YYYYYYY")
                ## add the target node if it is in the range
                if self.distance(x_new,y_new,self.target[0],self.target[1]) < step_size:
#                     print(self.path_tree.value)
                    
                    self.node_near.addchild(self.target_node)
#                     print(self.path_tree.size())
                    ## the goal is reached
                    self.goal_flag = True
                    return self.target_node
                ## else add the new node
                else:
                    print(self.path_tree.value)
                    new_node = Node((x_new,y_new),[])
                    self.node_near.addchild(new_node)
                    self.change_node_color(new_node)
#                     print(self.path_tree.size())
                    print("new node is added")
            

    
    def checkObstacle(self,x1,y1,x2,y2):
        for i in range(10):
            k = i/10
            x = x1*k + x2*(1-k)
            y = y1*k + y2*(1-k)
#             print(int(np.rint(y)),int(np.rint(x)))
            yindex = int(np.rint(y))
            xindex = int(np.rint(x))
            for i in [-2,-1,0,1,2]:
                for k in [-2,-1,0,1,2]:
                    if self.image[yindex+i,xindex+k]==0: ## it is occupied
#                         print("obs point",x,y)
#                         print(True)
                        return True ## there is an obstacle
        return False ## no obstacles

    def random_sample(self):
        x = int(np.random.uniform(0, self.map_width))
        y = int(np.random.uniform(0, self.map_height))
        self.node_random = Node((x,y),[])
        print("The random smaple is",x,y)
    
    def change_node_color(self, node):
        (x,y) = node.value
        for i in [-1,0,1]:
            for k in [-1,0,1]:
                self.visualization[y+i,x+k] = 0 ## make it black
        
        cv2.imwrite('map1.pgm', self.visualization)

    def change_node_color_large(self, node):
        (x,y) = node.value
        for i in [-2,-1,0,1,2]:
            for k in [-2,-1,0,1,2]:
                self.visualization[y+i,x+k] = 0 ## make it black
        cv2.imwrite('map1.pgm', self.visualization) 
        
    def find_path(self, iteration = 2000):
        for i in range(iteration):
            if not self.goal_flag:
                self.random_sample()
                self.node_near = self.nearest_node()
#                 print("nearest point is", self.node_near.value)
                self.possible_node = self.move_node()
                
            else:
                print("we find the goal")
                return self.possible_node, self.path_tree
        
if __name__ == '__main__':
    ## position (x,y)
    robot_postion = (300,325) 
    target_position = (400,200)
    image_path = 'map.pgm'
    path_object = rrt(robot_postion,target_position,image_path)
    last_node, tree = path_object.find_path()
    current = last_node
    lists = []
    while current!=None:
        lists.append(current.value)
        current = current.parent
    print(lists.reverse())
                    