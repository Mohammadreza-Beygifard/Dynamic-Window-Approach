'''
Object Oriented Programming (OOP) for the thesis:
    step1: Make a class as node and add distance method to this class. Done
    step2: Make a class as obstacle. This class should be inheritance of node.Done
    step3: Make a class as Trajectory. This will be also an inheritance of node. Done
    step4: for defining trajectories I relised I need another class as Pose.
        Since trajectory is a combination of poses which means, not only, we 
        have x and y coordinates as Node, but also, we have heading angle (psi) 
        as one of the coordinates. Done.
    step5: I have generated two other classes as SearchSpaceV and SearchSpaceD
        for making the search spaces of velocities and steering angles so I
        don't have to deal with 4 parameters each time. 
        Accelerations and the time for generating subtrajectories are editable 
        in these classes. Done
    step6: I should define a class as trajectory and put velocities and steering angles inside it
        so it has poses along with velocities and steerng angles. Done
    step6: I should take care about two things, at first abstraction of getting the velocities 
        and steering angles of each branch, then to define minus sign instead of EuDist for Node.
        I did it as this way, if the path is forbidden I put 'f' as the value of its score
        if it is not evaluated yet I put not_eval as the value of the cost.
        So in this way the numbers are only the costs of each subtrajectory and I 
        easily can evaluate the minimum cost between them.
    step7: I should take care about admissible velocity at the cost function, so those 
        branches that has the velocity more than the admissible one shoulb be baned.
'''
#%%
# Imports
import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, tan, pi, sqrt, atan


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
    def __init__(self, x, y, psi):
        self.x = x
        self.y = y
        self.psi = psi
        
    def HeadDist(self, other): #this method is for calculating the heading 
        return self.psi -other.psi #difference between two poses
    def Show(self):
        plt.plot(self.x, self.y, marker = 'D', color ='r')

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
        plt.plot(self.coord_xs, self.coord_ys, 'X', color = 'red')
        
        
    def GetCoords(self): #This method will return the list of nodes
        return self.points #of the obstacle
    
        

#This class returns the search space for the velocity       
class SearchSpaceV:
    def __init__(self, v):
        self.v = v
        self.v_dot = 0.1 #I assumed the acceleration for the robot as 0.1 m/s**2
        self.t = 1       #I assumed the time for generating subtrajectories is 1s
        self.search_space = []
        self.vel_min = self.v - self.v_dot * self.t
        self.vel_max = self.v + self.v_dot * self.t

        
class SearchSpaceD:
    def __init__(self, d):
        self.d = d
        self.d_dot = 0.05 #I assumed the acceleration for the robot as 0.05 rad/s**2
        self.t = 1       #I assumed the time for generating subtrajectories is 1s
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
                            x = self.list1[d_r_count][d_f_count][v_r_count][v_f_count][i-1].x + v * cos ( self.list1[d_r_count][d_f_count][v_r_count][v_f_count][i-1].psi + beta ) * (1/ (self.np-1)) # 1 / (self.np -1) is the time derivation for each step
                            y = self.list1[d_r_count][d_f_count][v_r_count][v_f_count][i-1].y + v * sin ( self.list1[d_r_count][d_f_count][v_r_count][v_f_count][i-1].psi + beta ) * (1/ (self.np-1)) # 1 / (self.np -1) is the time derivation for each step
                            psi = self.list1[d_r_count][d_f_count][v_r_count][v_f_count][i-1].psi + (v * cos (beta) * (tan (d_f) + tan (d_r))) / (self.l_f + self.l_r) 
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
        self.min_obs_dist = 1 #minimum distance to the obstacle.
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
                                self.sub_traj_obj.list1[d_r_count][d_f_count][v_r_count][v_f_count][p_count].cost = - 0.2 * min_dist
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
        #think about the admissible velocity
        #think about the situation that two sub_trajs have the same cost so you should break every thing when you reach to the minimum
    
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
        print(self.cost_list)
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
    def __init__(self, start_pose, goal_pose, obs_list, l, num_d, num_v, num_p):
        self.start_pose = start_pose
        self.goal_pose = goal_pose
        self.obs_list = obs_list
        self.l = l
        self.num_d = num_d
        self.num_v = num_v
        self.num_p = num_p
        self.sub_traj_list = []
        self.state = 'start'
        self.v_r_init = 0
        self.v_f_init = 0
        self.d_r_init = 0
        self.d_f_init = 0
        
    def Generate(self):
        i = 0
        while self.state == 'start':
            sub_traj = SubTrajectory(self.start_pose, self.v_r_init, self.v_f_init, self.d_r_init
                                     , self.d_f_init, self.l, self.num_d, self.num_v, self.num_p)
            cost_func = CostFunc(sub_traj, self.obs_list, self.goal_pose)
            cost_func.Eval()
            self.chosens = cost_func.Choose()
            self.sub_traj_list.append(cost_func)
            #now I should check whether I have reached the goal or not
            c_d_r = self.chosens[0] 
            c_d_f = self.chosens[1] 
            c_v_r = self.chosens[2] 
            c_v_f = self.chosens[3] 
            #here I take the distance of the final point till the goal pose
            dist_to_goal = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][int(self.num_p/2)].goal_dist
            dist_to_goal = round(dist_to_goal, ndigits = 3)
            print(f'{i} th attempt dist {dist_to_goal}')
            if dist_to_goal < 1:
                self.state = 'finish'
            else:
                x_init = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][int(self.num_p / 2)].x 
                y_init = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][int(self.num_p / 2)].y
                psi_init = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][int(self.num_p / 2)].psi
                self.previous_start_pose = self.start_pose # I introduce this to be sure that robot is not stocked
                self.start_pose = Pose(x_init, y_init, psi_init)
                if self.previous_start_pose.x == self.start_pose.x:
                    if self.previous_start_pose.y == self.start_pose.y:
                        self.state = 'stocked'
                        print(f'at {i}th attempt robot got stocked')
                self.v_r_init = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.num_p - 1].v_r * 0.5 
                self.v_f_init = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.num_p - 1].v_f * 0.5
                self.d_r_init = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.num_p - 1].d_r * 0.5
                self.d_f_init = self.sub_traj_list[i].sub_traj_obj.list1[c_d_r][c_d_f][c_v_r][c_v_f][self.num_p - 1].d_f * 0.5
                i +=1
            #if dist_to_goal < 5.1:
                #self.state = 'finish'
        
    def Show(self):
        count = len(self.sub_traj_list)
        for i in range(count):
            print(i) # for test to see on which iteration of plotting are we?
            self.sub_traj_list[i].Show()
   #%%
# This part is for testing and it goes step by step under each part of the code
     
        
'''      

test_list_1 = []
test_list_2 = []
test_list_3 = []
for i in range(3):
    test_list_2 = []
    for j in range(3):
        test_list_1 = []
        if j == 2:
            test_list_3.append(test_list_2)
        for k in range(3):
            test_list_1.append(0)
            if k == 2:
                test_list_2.append(test_list_1)

        
print(test_list_3)

'''
'''
test_list = [0, 0, 0]
test_list[1] = 1
'''













test_pose = Pose(0, 0, 0)
#test_subtraj = SubTrajectory(test_pose, 0, 0, 0, 0, 2, 3, 3, 11)
#print(test_subtraj.d_r_search_space)
#print(test_subtraj.d_f_search_space)
#print(test_subtraj.vel_r_search_space)
#print(test_subtraj.vel_f_search_space)

#ax = plt.gca()
#ax.axis([0, 10, 0, 10])

node1 = Node(2, 4)
node2 = Node(4, 6)
node3 = Node(10, 4)
node4 = Node(12, 6)
goal_pose = Pose(10, 10, 0)
obstacle_list = []
test_obs1 = Obstacle(node3, node4, 1)
test_obs2 = Obstacle(node1, node2, 2)
obstacle_list.append(test_obs1)
obstacle_list.append(test_obs2)
full_trajectory = FullTrajectory(test_pose, goal_pose, obstacle_list, 2, 3, 3, 11)
full_trajectory.Generate()
#cost_func = CostFunc (test_subtraj, obstacle_list, goal_pose)
#cost_func.EvalObsDist()
#cost_func.Eval()
#my_list = cost_func.Choose()

plt.figure(figsize = (8, 8))
full_trajectory.Show()
#full_trajectory.sub_traj_list[0].Show()
#test_subtraj.Show()
#cost_func.Show()
test_obs1.Show()
test_obs2.Show()
goal_pose.Show()

'''
CCC = 0
for a in range(test_subtraj.nv-1):
    for b in range(test_subtraj.nv-1):
        for c in range(test_subtraj.nd-1):
            for d in range(test_subtraj.nd-1):
                for i in range(test_subtraj.np-1):
                    CCC += 1
                    print(test_subtraj.sub_traj_list[a][b][c][d][i].v_r)
'''                   

#print(test_subtraj.sub_traj_list)


'''     
OObstacle = []

x = 0
y = 0
for i in range(10):
    x += 
    y += 1
    node = Node(x, y)
    OObstacle.append(node)
ss = OObstacle[0].EuDist(OObstacle[1])

'''

'''
test_search = SearchSpaceD(1)
print(test_search.search_space)
pose1 = Pose(0, 0, 0)
pose2 = Pose(1, 1, 1)
print(pose1.EuDist(pose2))
'''
'''
node1 = Node(0, 0)
node2 = Node(1, 1)
node3 = Node(5, 5)
node4 = Node(10, 10)
obstacle_list = []
test_obs1 = Obstacle(node1, node2, 5)
obstacle_list.append(test_obs1)
test_obs2 = Obstacle(node3, node4, 5)
obstacle_list.append(test_obs2)
print(len(obstacle_list))
obs_number = len(obstacle_list)



plt.figure(figsize = (8,8))
for i in range(obs_number):
    size_obs = obstacle_list[i].obs_size
    for j in range(size_obs):
        plt.plot(obstacle_list[i].points[j].x, obstacle_list[i].points[j].y, '.', color = 'green')
plt.margins(0.02)
'''

