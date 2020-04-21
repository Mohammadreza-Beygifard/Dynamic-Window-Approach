'''
Run the code to generate the Trajectory
'''
#%%
# Imports
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from math import sin, cos, tan, pi, sqrt, atan, degrees, radians
#for generating a mat file I need to import scipy.io.savemat
import scipy.io as scio
plt.style.use('seaborn-pastel')
from celluloid import Camera
import os

#%%
# Class definitions
'''
Class Node is the main class and all of the classes that we will define (Obstacle
and SubTrajectory) will use this class to provide nodes. Basically our obstacles
and Trajectories are a combination of nodes so this class is the basic class
for us.

'''
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

'''
Class Pose will be used to integrate the heading angle of the robot along with
its cartesean coordinates in one Object.

'''
#Class Pose  
class Pose(Node):
    def __init__(self, x, y, psi = pi):
        self.x = x
        self.y = y
        self.psi = psi
        
    def HeadDist(self, other): #this method is for calculating the heading 
        return self.psi -other.psi #difference between two poses
    def Show(self):
        pose, = plt.plot(self.x, self.y, 'D', color ='c')
        return pose,

'''

Class TejectoryPoint takes poses and the corresponding velocities and 
steering angles of each pose. The objects of this class are points of a
trajectory.

'''
#Class Trajectory
class TrajectoryPoint(Pose):
    def __init__ (self, x, y, psi, v_r, v_f, d_r, d_f, v, goal_dist, cost):
        self.x = x
        self.y = y
        self.psi = psi
        self.v_r = v_r
        self.v_f = v_f
        self.d_r = d_r
        self.d_f = d_f
        self.v = v
        self.obs_dist_list = [] #for evaluating the distance between each trajectory_point_obj and obstacle_point_object
        self.addmissible_velocity = [] #this is for evaluating the admissible velocity
        self.cost = cost # When it is -2 it means the costfunction has not checked it
        self.goal_dist = goal_dist   # When it is -1 it means it is a forbidden trajectory 
                                     # from 0 to + infinity it means cost function has evaluated it and has put
                                     # a value for it.
    
    #this method is a setter for the attribute obs_dist_list of the object trajectory_point
    #it will set the a list of all the distances between each point and the obstacles.
    def SetObsDistList(self, obs_dist):
        self.obs_dist_list.append(obs_dist)
        
    def SetAdmisVel(self, admis_vel):
        self.addmissible_velocity.append(admis_vel)
        
        

#Class Obstacle
'''
in class Obstacle this is the type of obstacles we are making:
    
    
    O (x1, y2)                            O(x2, y2) main_node_2
    
    
    
    
    
    
    O(x1, y1)  main_node_1                O(x2, y1)
    
The coordinates need to be integer and this is because of the mechanism of puting
nodes between each two points.
Then it will put nodes between them at each 1 / num_nodes cm. finally the class Obstacle
provides some methods to plot the obstacle and also return the points that
has made the obstacles.

Important Each of the points of the obstacle are nodes so by accessing them we have 
access to the methods we have defined at class Node.

To get access to each point of each obstacle I shall write, for example
obs1.ponits[0], in this example I am accessing the first obstacle point.
and for knowing the size of iterations we can use, for example, obs1.obs_size 
which is an integer that gives us the number of nodes of an obstacle.

self.step is the distance between nodes of the obstacle.
  
'''
class Obstacle(Node):
    def __init__(self, node1 , node2, num_nodes):
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
    
        

#This class returns the search space for the velocity       
class SearchSpaceV:
    def __init__(self, v):
        self.v = v
        self.v_dot = 0.1 #I assumed the acceleration for the robot as 0.1 m/s**2
        self.t = 2       #I assumed the time for generating subtrajectories is 1s
        self.search_space = []
        self.vel_min = self.v - self.v_dot * self.t
        self.vel_max = self.v + self.v_dot * self.t

        
class SearchSpaceD:
    def __init__(self, d):
        self.d = d
        self.d_dot = 0.3 #I assumed the acceleration for the robot as 0.05 rad/s**2
        self.t = 2       #I assumed the time for generating subtrajectories is 1s
        self.search_space = []
        self.d_min = self.d - 0.5 * self.d_dot * self.t ** 2
        self.d_max = self.d + 0.5 * self.d_dot * self.t ** 2

        

'''
The list for trajectories is like this:
  The whole Subtrajectories -> [ 
                                 [trajpoints],
                                 [trajpoints],
                                  .
                                  .
                                  .
                                 [trajpoints] <- each branch of subtrajectories with 
                              ]             respect to steering angles
                                
'''
      
class SubTrajectory:
    def __init__(self, pose_init #initial position
                 , v_r_init, v_f_init, d_r_init, d_f_init, l, num_d, num_v, num_p):
    
        self.l_f = l * 0.5 #I assume The rear and front lengths equal
        self.l_r = l * 0.5 #and I assumed them half of the total length of the vehicle.
        self.x_init = pose_init.x
        self.y_init = pose_init.y
        self.psi_init = pose_init.psi
        self.v_r_init = v_r_init
        self.v_f_init = v_f_init
        self.d_r_init = d_f_init
        self.d_f_init = d_f_init
        self.vel_r_search = SearchSpaceV(v_r_init)
        self.vel_f_search = SearchSpaceV(v_f_init)
        self.d_r_search = SearchSpaceD(d_r_init)
        self.d_f_search = SearchSpaceD(d_f_init)
        self.nd = num_d # the number of steering angles that I want to consider in the search space, it should be always odd number to have the middle value
        self.nv = num_v#the number of velocities that I want to consider in the search space, it should be always odd number to have the middle value
        self.np = num_p# the number of nodes in each branch of subtrajectory
        self.vel_r_search_space = np.linspace(self.vel_r_search.vel_min, self.vel_r_search.vel_max, self.nv)
        self.vel_f_search_space = np.linspace(self.vel_f_search.vel_min, self.vel_f_search.vel_max, self.nv)
        self.d_r_search_space = np.linspace(self.d_r_search.d_min, self.d_r_search.d_max, self.nd)
        self.d_f_search_space = np.linspace(self.d_f_search.d_min, self.d_f_search.d_max, self.nd)
        #for loops for making the subtrajectories:
        '''
        In the above lists I am saving points which I name trajectory points, they have 
        the x, y, psi and v_r, v_f, d_r, d_f as their characteristic. 
        '''
        #the total amount of poses that we have in our subtrajectory
        #The generator for loops.
        #initializing a numpy array for puting trajectory point objects
        #this numpy array is a multidimentional numpy array that makes
        #difference between rear and front steering angles and velocities
        #and the final dimension is the number of points in each sub_trajectory
        self.list1 = np.zeros([self.nd, self.nd, self.nv, self.nv, self.np], dtype = TrajectoryPoint)
        # for loop counters
        d_r_count = -1 
        for d_r in self.d_r_search_space:
            d_r_count += 1
            d_f_count = -1
            #if d_r_count == 5:
             #   print('fuck d_r_count')
            for d_f in self.d_f_search_space:
                d_f_count += 1
                v_r_count = -1
                #if d_f_count == 5:
                 #   print('fuck d_f_count')
                for vel_r in self.vel_r_search_space:
                    v_r_count += 1
                    v_f_count = -1
                    #if v_r_count == 5:
                     #   print('fuck v_r_count')
                    for vel_f in self.vel_f_search_space:
                        v_f_count += 1
                        #if v_f_count == 5:
                          #  print('fuck v_f_count')
                        '''
                        Here we have the discretized kinematic model of the vehicle
                        
                        '''
                        v = ((vel_f * cos (d_f)) + (vel_r * cos (d_r))) / (self.l_f + self.l_r) #The velocity that we assum it equal for some time
                        beta = atan (((self.l_f * tan(d_r)) + (self.l_r * tan(d_f)) ) / (self.l_f + self.l_r))
                        #putting the initial point in the numpy array
                        x = self.x_init 
                        y = self.y_init
                        psi = self.psi_init
                        trajectory_point = TrajectoryPoint(x, y, psi, vel_r, vel_f, d_r, d_f, v, 'not_eval', 'not_eval')
                        self.list1[d_r_count][d_f_count][v_r_count][v_f_count][0] = trajectory_point
                        for i in range(1, self.np): #the counter of this for loop starts from 1 since node number 0 is initialized before this for loop.
                            x = self.list1[d_r_count][d_f_count][v_r_count][v_f_count][i-1].x + v * cos ( self.list1[d_r_count][d_f_count][v_r_count][v_f_count][i-1].psi + beta ) * (2/ (self.np-1)) # 1 / (self.np -1) is the time derivation for each step
                            y = self.list1[d_r_count][d_f_count][v_r_count][v_f_count][i-1].y + v * sin ( self.list1[d_r_count][d_f_count][v_r_count][v_f_count][i-1].psi + beta ) * (2/ (self.np-1)) # 1 / (self.np -1) is the time derivation for each step
                            psi = self.list1[d_r_count][d_f_count][v_r_count][v_f_count][i-1].psi + (v * cos (beta) * (tan (d_f) + tan (d_r))) / (self.l_f + self.l_r) * (2/ (self.np-1))
                            trajectory_point = TrajectoryPoint(x, y, psi, vel_r, vel_f, d_r, d_f, v, 'not_eval', 'not_eval')
                            self.list1[d_r_count][d_f_count][v_r_count][v_f_count][i] = trajectory_point
        

                                
    def Show(self):
        for d_r_count in range(self.nd):
            for d_f_count in range(self.nd):
                for v_r_count in range(self.nv):
                    for v_f_count in range(self.nv):
                        for i in range(self.np):
                            plt.plot(self.list1[d_r_count][d_f_count][v_r_count][v_f_count][i].x, self.list1[d_r_count][d_f_count][v_r_count][v_f_count][i].y, '.', color ='green')

'''

After defining sub-trajectories we should make another class as subfunction.
this class will evaluate each subtrajectory and put a cost for its trajectory points.

'''
class CostFunc:
    def __init__ (self, sub_traj_obj, obs_list, goal_pose):
        self.sub_traj_obj = sub_traj_obj
        self.obs_list = obs_list
        self.goal_pose = goal_pose
        self.min_obs_dist = 0.8 #minimum distance to the obstacle.
    # Evaluating distance sub_function:
    def EvalObsDist(self):
        #taking the sizes of sub_traj:
        self.num_d = self.sub_traj_obj.nd #number of steering angles
        self.num_v = self.sub_traj_obj.nv #number of velocities
        self.num_p = self.sub_traj_obj.np #number of points
        #taking the size of obstacle_list:
        num_obs = len(self.obs_list)
        
        #calculating the distances:

        # for loop for calling obstacle
        for obs_count in range(num_obs):
            obs_size = self.obs_list[obs_count].obs_size
            for obs_p_count in range(obs_size):
                #for loops for calling the sub_taj_points
                for d_r_count in range(self.num_d): 
                    for d_f_count in range(self.num_d):
                        for v_r_count in range(self.num_v):
                            for v_f_count in range(self.num_v):
                                for p_count in range(self.num_p): #points for loop
                                    #here the euclidean distance is being calculated automatically thanks to the method over riding on class node
                                    obs_dist_cost = self.obs_list[obs_count].points[obs_p_count] - self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count]  
                                    self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].SetObsDistList(obs_dist_cost)
                                    #Till here we have calculated the distance of different points with different points of all of the obstacles
        #now we should choose the minimum distance between all of the distances of each trajectory point and obstacle points
        #Since we have saved these distances in an atribute of each trajectory_point object now we can access them easier
        for d_r_count in range(self.num_d): 
            for d_f_count in range(self.num_d):
                for v_r_count in range(self.num_v):
                    for v_f_count in range(self.num_v):
                        for p_count in range(self.num_p):
                            min_dist = min(self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].obs_dist_list)
                            if min_dist <= self.min_obs_dist: #if there is a collision,
                                for p_count in range(self.num_p):       #put the cost of all points to 'f'
                                    self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].cost = 'f' #'f' as forbidden
                                break #break the points for loop to go to the next branch               
                            else:
                                self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].cost = - 1 * min_dist
    def EvalGoalDist(self):#sub_function for evaluating the distance and heading from the goal.
        for d_r_count in range(self.num_d): 
            for d_f_count in range(self.num_d):
                for v_r_count in range(self.num_v):
                    for v_f_count in range(self.num_v):
                        for p_count in range(self.num_p): #points for loop
                            self.goal_dist_cost = self.goal_pose - self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count]
                            #self.goal_head_cost = abs(self.goal_pose.psi - self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].psi)
                            self.eval_goal_cost =  self.goal_dist_cost #+  self.goal_head_cost
                            if self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].cost != 'f':
                                self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].cost += 0.8 * self.eval_goal_cost #Augumented sum since we should add this cost to other costs
                                self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].goal_dist = self.goal_dist_cost
        
    def EvalAdmissibleVel(self):#this method will be used to evaluate if the velocity
                                # of each branch is less than its admissible velocity.
        self.bracking_deceleration = 0.1 #braking deceleration is assumed as 0.1 m/s^2
        for d_r_count in range(self.num_d): 
            for d_f_count in range(self.num_d):
                for v_r_count in range(self.num_v):
                    for v_f_count in range(self.num_v):
                        for p_count in range(self.num_p):
                            min_dist = min(self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].obs_dist_list)
                            v_admis = sqrt(2 * min_dist * self.bracking_deceleration)
                            self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].SetAdmisVel(v_admis)
                            if v_admis < self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].v:
                                for p_count in range(self.num_p):
                                    self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].cost = 'f' #'f' as forbidden
                                    self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].SetAdmisVel(v_admis)
                                break
    def Eval(self):#This is the final evaluation function and it will run all the EvalSub_functions
        self.EvalObsDist()
        self.EvalGoalDist()
        self.EvalAdmissibleVel()
    
    #this method is for choosing the better subtrajectory    
    def Choose(self):
        'This part is for taking the costs of each branch'
        self.cost_list = [] #this is a list for saving all the costs
        for d_r_count in range(self.num_d): 
            for d_f_count in range(self.num_d):
                for v_r_count in range(self.num_v):
                    for v_f_count in range(self.num_v):
                        our_cost = self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][self.num_p -1].cost 
                        if our_cost != 'f':
                            self.cost_list.append(our_cost)
        #print(self.cost_list)
        'This part is for choosing the branch with the lowest cost'
        self.min_cost = min(self.cost_list)
        counter = 0 #this counter is for avoiding choosing several paths as optimum.
        for d_r_count in range(self.num_d): 
            if counter != 0:
                break
            for d_f_count in range(self.num_d):
                if counter != 0:
                    break
                for v_r_count in range(self.num_v):
                    if counter != 0:
                        break
                    for v_f_count in range(self.num_v):
                        if self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][self.num_p -1].cost == self.min_cost:
                            for p_count in range(self.num_p):
                                self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].cost = 'c' # 'c' as chosen
                            # Here I am setting these variables to return it and have a quick access to the chosen path.
                            self.chosen_d_r = d_r_count
                            self.chosen_d_f = d_f_count
                            self.chosen_v_r = v_r_count
                            self.chosen_v_f = v_f_count
                            counter +=1
                            break
        chosen_list = [self.chosen_d_r, self.chosen_d_f, self.chosen_v_r, self.chosen_v_f]
        return chosen_list
    
    
    def Show(self):#this method will help us to show the subtrajectories based on their costs
        for d_r_count in range(self.num_d):
            for d_f_count in range(self.num_d):
                for v_r_count in range(self.num_v):
                    for v_f_count in range(self.num_v):
                        for p_count in range(self.num_p):
                            #if the branch is (forbidden) draw it red
                            if self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].cost == 'f':
                                plt.plot(self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].x, self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].y, marker = '.', color ='red')
                            #if the branch is (allowed) draw it green
                            elif self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].cost == 'c':
                                plt.plot(self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].x, self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].y, marker = 'D', color ='k')
                                
                            else:
                                plt.plot(self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].x, self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].y, marker = '.', color ='green')
                                

class FullTrajectory:
    def __init__(self, start_pose, goal_pose, obs_list, l, num_d, num_v, num_p, v_r_init, v_f_init, d_r_init, d_f_init):
        self.start_pose = start_pose
        self.goal_pose = goal_pose
        self.obs_list = obs_list
        self.l = l
        self.num_d = num_d
        self.num_v = num_v
        self.num_p = num_p
        self.sub_traj_list = []
        self.state = 'start'
        self.v_r_init = v_r_init
        self.v_f_init = v_f_init
        self.d_r_init = d_r_init
        self.d_f_init = d_f_init
        self.chosen_reced_point = 1
        
    def Generate(self):
        
            i = 0 #this counter will be used to count the number of iterations
            #these lists will be used to save the chosen paths for faster path view
            self.chosen_d_r_list =[]
            self.chosen_d_f_list =[]
            self.chosen_v_r_list =[]
            self.chosen_v_f_list =[]
            
            while self.state == 'start':
                sub_traj = SubTrajectory(self.start_pose, self.v_r_init, self.v_f_init, self.d_r_init
                                         , self.d_f_init, self.l, self.num_d, self.num_v, self.num_p)
                cost_func = CostFunc(sub_traj, self.obs_list, self.goal_pose)
                cost_func.Eval()
                
                self.chosens = cost_func.Choose()
                self.sub_traj_list.append(cost_func)
                #now I should check whether I have reached the goal or not
                #c stansds for chosen, so, c_d_r is chosen rear wheel steering angle
                c_d_r = self.chosens[0] 
                c_d_f = self.chosens[1] 
                c_v_r = self.chosens[2] 
                c_v_f = self.chosens[3]
                #adding the chosen values to the lists
                self.chosen_d_r_list.append(c_d_r)
                self.chosen_d_f_list.append(c_d_f)
                self.chosen_v_r_list.append(c_v_r)
                self.chosen_v_f_list.append(c_v_f)
                #here I take the distance of the final point till the goal pose
                dist_to_goal = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].goal_dist
                dist_to_goal = round(dist_to_goal, ndigits = 3)
                print(f'{i} th iteration dist_to_goal {dist_to_goal} m')
                if dist_to_goal < 0.7:
                    self.state = 'finish'
                else:
                    x_init = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].x 
                    y_init = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].y
                    psi_init = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].psi
                    self.previous_start_pose = self.start_pose # I introduce this to be sure that robot is not stocked
                    self.start_pose = Pose(x_init, y_init, psi_init)
                    if round(self.previous_start_pose.x, ndigits = 3) == round(self.start_pose.x, ndigits = 3):
                        if round(self.previous_start_pose.y, ndigits = 3) == round(self.start_pose.y, ndigits = 3):
                            self.state = 'stocked'
                            print(f'at {i}th iteration robot got stocked')
                    self.v_r_init = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].v_r * 0.1
                    self.v_f_init = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].v_f * 0.1
                    self.d_r_init = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].d_r * 0.1 * 0.1 * 0.5
                    self.d_f_init = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].d_f * 0.1 * 0.1 * 0.5
                    i +=1
                #if dist_to_goal < 5.1:
                    #self.state = 'finish'
        
    
                    #to get the number of chosen points
            number_of_points = len(self.chosen_d_r_list)
            #creat a numpy array for chosen points coordinates
            self.chosen_x = np.zeros((number_of_points))
            self.chosen_y = np.zeros((number_of_points))
            self.chosen_psi = np.zeros((number_of_points))
            self.path_length_list = []
            #putting the chosen points coordinates in the arrays
            for i in range(number_of_points):
                
                c_d_r = self.chosen_d_r_list[i]
                c_d_f = self.chosen_d_f_list[i]
                c_v_r = self.chosen_v_r_list[i]
                c_v_f = self.chosen_v_f_list[i]
                self.chosen_x[i] = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].x
                self.chosen_y[i] = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].y
                self.chosen_psi[i] = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].psi
                #this part is for obtaining path length
                if i !=0:
                    self.path_length_list.append(self.path_length_list[i-1] + (self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point] - self.sub_traj_list[i-1].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point]))
                else:
                    self.path_length_list.append(0)




    def Show(self, first_sub_traj_number, last_sub_traj_number): #this method shows the whole trajectories even forbiddens and it takes a long time to render
        #because plotting the whole subtrajectory takes a lot of time
        #and at the end doesn't give us any valuable info
        #I put this first and last numbers
        #to plot a portion of full subtrajectory.
        self.first_sub_traj_number = first_sub_traj_number
        self.last_sub_traj_number = last_sub_traj_number
        
        count = len(self.sub_traj_list) #the total number of sub_trajectories
        if (self.last_sub_traj_number + 1) > count:
            self.last_sub_traj_number = (count - 1)
        for i in range(self.first_sub_traj_number, (self.last_sub_traj_number + 1)):
            print(i) # for test to see on which iteration of plotting are we?
            self.sub_traj_list[i].Show()
            
            
    def ShowChosen(self):#this method shows only chosen trajectories and it is faster

            
           
        #plotting the chosen points    
        show_chosen, = plt.plot(self.chosen_x, self.chosen_y, marker = 'D', color = 'green', ms = 5)
        return show_chosen,    
    def ShowProfile(self):
        self.overall_vel = np.zeros(len(self.chosen_d_r_list))
        self.v_r_list = np.zeros(len(self.chosen_d_r_list))
        self.v_f_list = np.zeros(len(self.chosen_d_r_list))
        for i in range(len(self.chosen_d_r_list)):
            c_d_r = self.chosen_d_r_list[i]
            c_d_f = self.chosen_d_f_list[i]
            c_v_r = self.chosen_v_r_list[i]
            c_v_f = self.chosen_v_f_list[i]
            self.overall_vel[i] = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].v * 0.1
            self.v_r_list[i] = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].v_r * 0.1
            self.v_f_list[i] = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].v_f * 0.1
        plt.figure(figsize = (8, 8))
        plt.plot(self.path_length_list, self.overall_vel)
        plt.xlabel('Path_Length (m)')
        plt.ylabel('Overall_Velocity (m/s)')
        plt.figure(figsize = (8, 8))
        plt.plot(self.path_length_list, self.v_r_list)
        plt.xlabel('Path_Length (m)')
        plt.ylabel('Rear_Wheel_Velocity (m/s)')
        plt.figure(figsize = (8, 8))
        plt.plot(self.path_length_list, self.v_f_list)
        plt.xlabel('Path_Length (m)')
        plt.ylabel('Front_Wheel_Velocity (m/s)')
  
    def GiveFinalPose(self):#this method will return the final pose of the full_trajectory
        #taking the last chosen velocities and steering angles
        c_d_r = self.chosen_d_r_list[-1]
        c_d_f = self.chosen_d_f_list[-1]
        c_v_r = self.chosen_v_r_list[-1]
        c_v_f = self.chosen_v_f_list[-1]
        #accessing the last trajectory point of the full trajectory (considering the receding horizon technique, so we choose the firs node of the final chosen sub_traj branch)
        final_x = self.sub_traj_list[-1].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].x
        final_y = self.sub_traj_list[-1].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].y
        final_psi = self.sub_traj_list[-1].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].psi
        self.final_pose = Pose(final_x, final_y, final_psi)
        return self.final_pose
    
    def GiveFinalSearch(self):#this method is for a quick access to final velocities and steering angles
        #this part is exactly the same as the part of setting initials in the for loop of full_trajectory generator
        #we multiply all of the values to 0.1 and 0.1 * 0.1 * 0.5 becaues we are taking the first step always for which we have less speed and steering angles than the final point of the sub_trajectory
        c_d_r = self.chosen_d_r_list[-1]
        c_d_f = self.chosen_d_f_list[-1]
        c_v_r = self.chosen_v_r_list[-1]
        c_v_f = self.chosen_v_f_list[-1] 
        v_r_init = self.sub_traj_list[-1].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].v_r * 0.1
        v_f_init = self.sub_traj_list[-1].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].v_f * 0.1
        d_r_init = self.sub_traj_list[-1].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].d_r * 0.1 * 0.1 * 0.5
        d_f_init = self.sub_traj_list[-1].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.chosen_reced_point].d_f * 0.1 * 0.1 * 0.5
        self.initial_list = [v_r_init, v_f_init, d_r_init, d_f_init]
        return self.initial_list
        
        
       
class GoalSeeker:
    def __init__(self, start_pose, goal_pose_list, obs_list, l, num_d, num_v, num_p, v_r_init, v_f_init, d_r_init, d_f_init):
       self.start_pose = start_pose
       self.goal_pose_list = goal_pose_list
       self.obs_list = obs_list
       self.l = l
       self.num_d = num_d
       self.num_v = num_v
       self.num_p = num_p
       self.full_traj_list = []
       self.state = 'start'
       self.v_r_init = v_r_init
       self.v_f_init = v_f_init
       self.d_r_init = d_r_init
       self.d_f_init = d_f_init
       self.chosen_reced_point = 1
       
    #seeking the goals   
    def Seek(self):
        number_of_goals = len(self.goal_pose_list)
        #seeking first goal
        goal_pose = self.goal_pose_list[0]
        full_trajectory = FullTrajectory(self.start_pose, goal_pose, self.obs_list, self.l, self.num_d, self.num_v, self.num_p, self.v_r_init, self.v_f_init, self.d_r_init, self.d_f_init)
        full_trajectory.Generate()
        self.full_traj_list.append(full_trajectory)
        print('**********************************')
        print('Robot reached the 1 st goal')
        print('**********************************')
        #for the situations that we have more than one goal
        if number_of_goals >= 2:
            for i in range(1, number_of_goals):
                self.start_pose = self.full_traj_list[i-1].GiveFinalPose()
                goal_pose = self.goal_pose_list[i]
                initial_list = self.full_traj_list[-1].GiveFinalSearch()
                self.v_r_init = initial_list[0]
                self.v_f_init = initial_list[1]
                self.d_r_init = initial_list[2]
                self.d_f_init = initial_list[3]
                full_trajectory = FullTrajectory(self.start_pose, goal_pose, self.obs_list, self.l, self.num_d, self.num_v, self.num_p, self.v_r_init, self.v_f_init, self.d_r_init, self.d_f_init)
                full_trajectory.Generate()
                self.full_traj_list.append(full_trajectory)
                number = i + 1
                print('**********************************')
                print(f'Robot reached the {number} nd goal')
                print('**********************************')
                
    def ShowChosen(self):
        for full_traj in self.full_traj_list:
            show_chosen, = plt.plot(full_traj.chosen_x, full_traj.chosen_y, marker = 'D', color = 'green', ms = 5)
        
        
        return show_chosen, 
       

    def ShowProfile(self):
        for full_traj in self.full_traj_list:
            full_traj.ShowProfile()
        
    #this method is responsible to generate a mat file out of all the points    
    def GenMat(self):
        #At fisrt I need to put all of the chosen coordinates into one concatenated array.
        number_of_full_trajs = len(self.full_traj_list)
        self.chosen_x = self.full_traj_list[0].chosen_x
        self.chosen_y = self.full_traj_list[0].chosen_y
        self.chosen_psi = self.full_traj_list[0].chosen_psi
        for i in range(1, number_of_full_trajs):
            self.chosen_x = np.concatenate((self.chosen_x, self.full_traj_list[i].chosen_x))
            self.chosen_y = np.concatenate((self.chosen_y, self.full_traj_list[i].chosen_y))
            self.chosen_psi = np.concatenate((self.chosen_psi, self.full_traj_list[i].chosen_psi))
        #Here I am creating a 3*n array in which n is the number of total chosen points
        #then I save each chosen array into each row of the new array
        #and then I save it as a mat file
        obj_arr = np.zeros((3,), dtype=np.object)
        obj_arr[0] = self.chosen_x
        obj_arr[1] = self.chosen_y
        obj_arr[2] = self.chosen_psi
        scio.savemat('results/path/path.mat', mdict = {'path': obj_arr})

          
    #%%
# This part is for testing and it goes step by step under each part of the code
     

#Starting pose
        
test_pose = Pose(18, 8, 3 * pi / 2)

#%%
#Defining goals manually
'''
goal_pose_list = []
goal_pose1 = Pose(15, 2, pi)
goal_pose_list.append(goal_pose1)
goal_pose2 = Pose(11, 8, pi/2)
goal_pose_list.append(goal_pose2)
goal_pose3 = Pose(7, 2, pi/2)
goal_pose_list.append(goal_pose3)
goal_pose4 = Pose(3, 8, pi/2)
goal_pose_list.append(goal_pose4)
goal_pose5 = Pose(1, 2, pi/2)
goal_pose_list.append(goal_pose5)
'''

#%%
'''Goal taker from the global planner'''
goal_pose_list = []
goal_list_x = np.load('C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\way_points\\way_points_x.npy')
goal_list_y = np.load('C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\way_points\\way_points_y.npy')

num_goal = len (goal_list_x)
for i in range(num_goal - 3):
    goal_pose = Pose(goal_list_x[i], goal_list_y[i]) 
    goal_pose_list.append(goal_pose)

goal_pose_list.reverse()
#%%
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

#%%
#generating trajectories

goal_seeker = GoalSeeker(test_pose, goal_pose_list, obstacle_list, 1, 3, 3, 17, 0, 0, 0, 0)
goal_seeker.Seek()

#goal_seeker.ShowProfile()

#%% creating the directories
#this part will creat different directories for me to make an order in my files
try:
    os.makedirs('C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\obs')
except FileExistsError:
    print('obs directory exists')   
try:
    os.makedirs('C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\goals')
except FileExistsError:
    print('goals directory exists')   
try:
    os.makedirs('C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\path')
except FileExistsError:
    print('path directory exists')
    
try:
    os.makedirs('C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\gif')
except FileExistsError:
    print('gif directory exists')
goal_seeker.GenMat()
#%%


colors = ['green', 'black' ,'red']
lines = [Line2D([0], [0], color = c ,linewidth = 3, linestyle = 'dotted') for c in colors]


plt.figure(figsize = (10, 5))
handles = []
labels= []
plt.xlabel('x [m]')
plt.ylabel('y [m]')
show_chosen = goal_seeker.ShowChosen()
start, = plt.plot(test_pose.x, test_pose.y, 'X', color = 'black', ms = 10)
start, = start,
count_obs = -1 #this counter will be used for saving the coordiantes of the obstacles
for test_obs in obstacle_list:
    count_obs += 1 #iterating the counter of obstacles
    obs = test_obs.Show()
    #this part is for saving the coordinates of obstacles
    np.save(f'C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\obs\\Obs{count_obs}_x', test_obs.coord_xs)
    np.save(f'C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\obs\\Obs{count_obs}_y', test_obs.coord_ys)   
count_goal = -1 #this counter will be used for saving the coordinates of the goals
for goal_pose in goal_pose_list:
    count_goal += 1
    pose = goal_pose.Show()
    #this part is for saving the coordinates of goals
    my_x = np.array(goal_pose.x) #to transform the single coordinate number to a numpy array
    my_y = np.array(goal_pose.y)
    np.save(f'C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\goals\\Goal{count_goal}_x', my_x) #we need a numpy array to be able to save a binary file
    np.save(f'C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\goals\\Goal{count_goal}_y', my_y)
#this part is for adding the legends
handles.append(start)
labels.append('Start_Point')
handles.append(obs)
labels.append('Obstacle')
handles.append(pose)
labels.append('Goal')
handles.append(show_chosen)
labels.append('Path')
plt.legend(handles, labels, loc = 'upper left')
#%% Save the data of the path as binary files

np.save('C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\x', goal_seeker.chosen_x)
np.save('C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\y', goal_seeker.chosen_y)
np.save('C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\psi', goal_seeker.chosen_psi)

'''
for loading them easily use np.load('Name.npy')
'''

#%% Saving the logfs
logfs_text =f'This is a path planner logfs file\nObs number = {count_obs}\nGoal number = {count_goal}\n'
my_logfs = open('my_logfs.txt', 'w+')
my_logfs.write(logfs_text)
my_logfs.close()




