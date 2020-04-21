#%%
# Imports
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from math import sin, cos, tan, pi, sqrt, atan, degrees, radians, floor
#for generating a mat file I need to import scipy.io.savemat
import scipy.io as scio
plt.style.use('seaborn-pastel')
from celluloid import Camera
import os
from random import random
from datetime import datetime


#%% Making Directories

try:
    os.makedirs('results\\way_points')
except FileExistsError:
    print('obs directory exists')
    
#%% classes
#Class Node
class Node:
    def __init__(self, x, y): #initialization the node
        self.x = x
        self.y = y
    #Method Overridiing, here I am defining (-) sign as Euclidean distance
    #So node_1 - node_2 will return the euclidean distance between the two nodes.
    def __sub__(self, other): #calculate Euclidean distance between two nodes
                             #This method will be used for collision checking
                             #and our Cost function.
        xx = (self.x - other.x)**2
        yy = (self.y - other.y)**2
        return sqrt(xx + yy)
    
    
    def GetCoords(self): #The method to return the coordinates of a node
        return self.x, self.y
    
class Pose(Node):
    def __init__(self, x, y, psi, parent, cost = 0):
        self.x = x
        self.y = y
        self.psi = psi
        self.parent = parent
        self.cost = cost
    def HeadDist(self, other): #this method is for calculating the heading 
        return self.psi -other.psi #difference between two poses
    def __sub__(self, other): #calculate Euclidean distance between two nodes
                             #This method will be used for collision checking
                             #and our Cost function.
        xx = (self.x - other.x)**2
        yy = (self.y - other.y)**2
        return sqrt(xx + yy)
    def Show(self, marker = 'D', color = 'k'):
        pose, = plt.plot(self.x, self.y, marker , color = color)
        return pose,

    
class Obstacle(Node):
    def __init__(self, node1 , node2, num_nodes = 2):
        self.num = num_nodes #between each 1 m we put self.num spaces (or self.num -1 nodes)
        x1_x2_range = abs(node1.x - node2.x) * self.num + 1 #multiplied to self.num since I want
        y1_y2_range = abs(node1.y - node2.y) * self.num + 1 # to have a node at each 1/self.num
        self.points = []                             # and add 1 since I 
        self.step = 1/self.num                       #generate all nodes from the 
                                                     #start till the final node and by subtracting at first I am loosing one node
        x = round(node1.x - self.step, 3)            #I have used abs. So, if
        for i in range(x1_x2_range):                 #the user enters the main 
            x = round(x + self.step, 3)              #nodes by mistacke, it will
            y = round(node1.y - self.step, 3)        #not recieve an error
            for j in range(y1_y2_range):
                y = round(y + self.step,3)
                point = Node(x, y)
                #self.points.append(point)
                if point.x == node1.x or point.x == node2.x:
                    self.points.append(point)
                elif point.y == node1.y or point.y == node2.y:
                    self.points.append(point)
        self.obs_size =len(self.points)
        
       
    def Show(self): #This method will draw the obstacle on the figure
        coord_xs = []
        coord_ys = []
        for i in range(self.obs_size):
            x, y = self.points[i].GetCoords()
            coord_xs.append(x)
            coord_ys.append(y)
        
        self.coord_xs = np.array(coord_xs)
        self.coord_ys = np.array(coord_ys)
        obs, = plt.plot(self.coord_xs, self.coord_ys, 'X', color = 'm')
        return obs,
        
        
    def GetCoords(self): #This method will return the list of nodes
        return self.points #of the obstacle
    
class Subtrajectory:
    '''This class will generate the first subtrajectories'''
    def __init__(self, pose_init, v = 1, l = 1, T = 1, n = 2, nd = 3, delta_max = pi/6):
        self.pose_init = pose_init
        self.v = v
        self.l = l
        self.T = T
        self.n = n
        self.nd = nd
        self.delta_max = delta_max
        self.Generate()
    def Generate(self):
        '''defining the time increment'''
        self.dt = self.T / (self.n - 1)
        self.pose_list = np.zeros([self.nd, self.n], dtype = Pose)
        self.final_list = np.zeros([self.nd], dtype = Pose)
        for i in range(self.nd):
            self.pose_list[i][0] = self.pose_init #putting the initial pose of all branches as the initial pose
        for i in range(self.nd):
            my_delta = self.delta_max - 2 * i * self.delta_max / (self.nd-1) 
            for j in range(1, self.n):
                psi =  self.pose_list[i][j-1].psi + (self.v * tan(my_delta) / self.l) * self.dt
                x =  self.pose_list[i][j-1].x + self.v * cos(psi) * self.dt
                y =  self.pose_list[i][j-1].y + self.v * sin(psi) * self.dt
                new_pose = Pose(x, y, psi, self.pose_init)
                self.pose_list[i][j] = new_pose
                self.final_list[i] = new_pose
    
    def Show(self):
        try:
            x_list = []
            y_list = []
            for group_pose in self.pose_list:
                for pose in group_pose:
                    x_list.append(pose.x)
                    y_list.append(pose.y)
                    plt.plot(x_list, y_list , color = 'blue')
        except AttributeError:
            print('at firts use method Generate() please')
            
class Tree:
    def __init__(self, start_pose, obstacle_list, goal):
        '''Tree will take start, goal and obstacle nodes'''
        self.start_pose = start_pose
        self.goal = goal
        self.obs_list = obstacle_list
        self.state = True
        '''Then It will create two different lists'''
        self.initial_poses_list1 = [] #for running a random algorithm to choose between them for generating branches
        self.initial_poses_list2 = []
        self.initial_poses_list3 = []
        self.initial_poses_list4 = []
        self.new_poses_list = [] #for putting the new genaratied poses and evaluate them with cost
        self.final_list = []
        self.new_poses_list.append(start_pose) #Start pose is the first new pose
        self.goal_dist_list = []
        
    def Cost(self):
        while(self.new_poses_list != []):
        # To check the initial point
        # Before all of the inner loops we put an if statement to get sure that
        # we are still allowed to be in the while loop.
                           
                               
                
            #To check if new poses have collision
            num_obs = len(self.obs_list)
            for obs_count in range(num_obs):
                obs_size = self.obs_list[obs_count].obs_size
                for obs_p_count in range(obs_size):
                    pose_num = len(self.new_poses_list)
                    for pose_count in range(pose_num):
                        obs_dist = self.obs_list[obs_count].points[obs_p_count]  - self.new_poses_list[pose_count]
                        if obs_dist < 0.7:
                            self.new_poses_list[pose_count].cost = 'f'

                #To check if we have reached the goal
            pose_num = len(self.new_poses_list)
            for pose_count in range(pose_num):
                goal_dist = self.new_poses_list[pose_count] - self.goal
                if self.new_poses_list[pose_count].cost != 'f':
                    self.new_poses_list[pose_count].cost = goal_dist
                rounded_goal_dist = round(goal_dist, ndigits = 3)
                self.goal_dist_list.append(rounded_goal_dist)
                if goal_dist < 0.7:
                    print('*******************')
                    print('We reached the goal')
                    print('*******************')
                    self.final_list.append(self.new_poses_list[pose_count])
                    self.state = False # to stop generate from generating trees. 
                else:
                    print(f'Dist_to_Goal : {goal_dist} m')

            my_goal_dist_set = set(self.goal_dist_list)
            num_set = len(my_goal_dist_set)
            goal_dist_set_array = np.zeros([num_set])
            i = 0
            for set_member in my_goal_dist_set:
                goal_dist_set_array[i] = set_member
                i += 1
            mean_value_dist_goal = goal_dist_set_array.mean()
            quarter_value_dist_goal = mean_value_dist_goal / 2
            second_quarter_value_dist_goal = mean_value_dist_goal * 3 / 2
            num_poses_2 = len(self.new_poses_list)
            for pose_count in range(num_poses_2):
                if self.new_poses_list[pose_count].cost != 'f':
                    if self.new_poses_list[pose_count].cost <= quarter_value_dist_goal :
                        self.initial_poses_list1.append(self.new_poses_list[pose_count])
                        
                    elif self.new_poses_list[pose_count].cost <= mean_value_dist_goal :
                        self.initial_poses_list2.append(self.new_poses_list[pose_count])
                        
                    elif self.new_poses_list[pose_count].cost <= second_quarter_value_dist_goal:
                        self.initial_poses_list3.append(self.new_poses_list[pose_count])                    
                    else:
                        self.initial_poses_list4.append(self.new_poses_list[pose_count])
            self.new_poses_list = []
            self.goal_dist_list = []
    def Generate(self):
        while ((self.initial_poses_list1 != [] or self.initial_poses_list2 != [] or self.initial_poses_list3 != [] or self.initial_poses_list4 != [])and self.state == True):
            #generating the random number
            if self.initial_poses_list1 != []:
                size_of_the_list = len(self.initial_poses_list1)
                random_number = size_of_the_list * random()
                random_number = floor(random_number)
                pose_init = self.initial_poses_list1[random_number]
                self.initial_poses_list1.remove(self.initial_poses_list1[random_number])
            
            elif self.initial_poses_list2 != []:
                size_of_the_list = len(self.initial_poses_list2)
                random_number = size_of_the_list * random()
                random_number = floor(random_number)
                pose_init = self.initial_poses_list2[random_number]
                self.initial_poses_list2.remove(self.initial_poses_list2[random_number])
                
            elif self.initial_poses_list3 != []:
                size_of_the_list = len(self.initial_poses_list3)
                random_number = size_of_the_list * random()
                random_number = floor(random_number)
                pose_init = self.initial_poses_list3[random_number]
                self.initial_poses_list3.remove(self.initial_poses_list3[random_number])     
                
            else:
                size_of_the_list = len(self.initial_poses_list4)
                random_number = size_of_the_list * random()
                random_number = floor(random_number)
                pose_init = self.initial_poses_list4[random_number]
                self.initial_poses_list4.remove(self.initial_poses_list4[random_number])
            
            subtraj = Subtrajectory(pose_init)
            for pose in subtraj.final_list:
                self.new_poses_list.append(pose)
            self.Cost()
            subtraj.Show()
            

    def Show(self):
        self.way_point_x_list = []
        self.way_point_y_list = []
        for pose in self.final_list:
            self.parent_list = [pose]
            parent = pose.parent
            while parent != None:
                self.parent_list.append(parent)
                parent = self.parent_list[-1].parent
        for pose in self.parent_list:
            self.way_point_x_list.append(pose.x)
            self.way_point_y_list.append(pose.y)
            pose, = pose.Show()

        
        self.way_point_x_list = np.array(self.way_point_x_list)
        self.way_point_y_list = np.array(self.way_point_y_list)
        np.save('results\\way_points\\way_points_x', self.way_point_x_list)
        np.save('results\\way_points\\way_points_y', self.way_point_y_list)
        return pose,



#%%
pose_init = Pose(18, 8, 3 * pi / 2, None)

# Defining Obstacles 
'barrier definers'
node1 = Node(0, 0)
node2 = Node(20, 0)
node3 = Node(0, 10)
node4 = Node(20, 10)

'first obstacle'
node5 = Node(2, 0)
node6 = Node(4, 7)

'second obstacle'
node7 = Node(6, 3)
node8 = Node(8, 10)

'third obstacle'
node9 = Node(10, 0)
node10 = Node(12, 7)

'fourth obstacle'
node11 = Node(14, 3)
node12 = Node(16, 10)

test_obs1 = Obstacle(node1, node2, 2) #the lower horizental barrier
test_obs2 = Obstacle(node1, node3, 2) #the left vertical barrier
test_obs3 = Obstacle(node3, node4, 2) #the upper horizental barrier
test_obs4 = Obstacle(node2, node4, 2) #the right vertical barrier

test_obs5 = Obstacle(node5, node6, 2)
test_obs6 = Obstacle(node7, node8, 2)
test_obs7 = Obstacle(node9, node10, 2)
test_obs8 = Obstacle(node11, node12, 2)

obstacle_list = []
obstacle_list.append(test_obs1)
obstacle_list.append(test_obs2)
obstacle_list.append(test_obs3)
obstacle_list.append(test_obs4)
obstacle_list.append(test_obs5)
obstacle_list.append(test_obs6)
obstacle_list.append(test_obs7)
obstacle_list.append(test_obs8)

goal = Node(1, 2)
my_tree = Tree(pose_init, obstacle_list, goal)


'''Plotting the figure'''

plt.figure(figsize = (10, 5))
handles = []
labels= []
plt.xlabel('x [m]')
plt.ylabel('y [m]')

start, = pose_init.Show(marker = 'X')

goal, = plt.plot(goal.x, goal.y, 'D' , color = 'c')


for obs in obstacle_list:
    obs, = obs.Show()
my_tree.Cost()
my_tree.Generate()


way_point, = my_tree.Show()

handles.append(start)
handles.append(goal)
handles.append(obs)
handles.append(way_point)
labels.append('Start')
labels.append('Goal')
labels.append('Obstacle')
labels.append('Way Points')
plt.legend(handles, labels, loc = 'upper left')
plt.savefig('result.png')
# 
# subtraj = Subtrajectory(pose_init)
# subtraj.Show()
# obs1.Show()
# =============================================================================


#my_tree = Tree(pose_init, None)
#my_tree.Show()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    