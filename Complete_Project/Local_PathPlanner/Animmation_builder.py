import numpy as np
import matplotlib.pyplot as plt
from celluloid import Camera
import os
from math import degrees


#%% Map loader
class LogFS:
    def __init__(self):
        self.my_logfs = open('C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\my_logfs.txt', 'r')
        lines = self.my_logfs.readlines()
        self.my_logfs.close()

        char = []
        for chars in lines[1]:
            char.append(chars)
        char.reverse()
        print(char)
        char.remove(char[0])
        print(char)
        obs_num = ''
        for char1 in char:
            if (char1 != ' '):
                obs_num += char1
            else:
                break 
        str = obs_num 
        goal_num=''.join(reversed(str))
        self.obs_num = int(obs_num)

        char = []
        for chars in lines[2]:
            char.append(chars)
        
        char.reverse()
        char.remove(char[0])
        goal_num = ''
        for char1 in char:
            if (char1 != ' '):
                goal_num += char1
            else:
                break
        str = goal_num 
        goal_num=''.join(reversed(str))
        self.goal_num = int(goal_num)
        
class Obstacle:
    def __init__(self, logfs_name):
        self.my_log = logfs_name
        self.obs_x_list = []
        self.obs_y_list = []
        for i in range(self.my_log.obs_num + 1):
            obs_x = np.load(f'C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\obs\\Obs{i}_x.npy')
            self.obs_x_list.append(obs_x)
            obs_y = np.load(f'C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\obs\\Obs{i}_y.npy')
            self.obs_y_list.append(obs_y)
    def show(self):
        num = len(self.obs_x_list)
        for i in range(num):
            obs, = plt.plot(self.obs_x_list[i], self.obs_y_list[i], 'X', color = 'm')
        return obs,

class Goal:
    def __init__(self, logfs_name):
        self.my_log = logfs_name
        self.goal_x_list = []
        self.goal_y_list = []
        for i in range(self.my_log.goal_num + 1):
            goal_x = np.load(f'C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\goals\\Goal{i}_x.npy')
            self.goal_x_list.append(goal_x)
            goal_y = np.load(f'C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\goals\\Goal{i}_y.npy')
            self.goal_y_list.append(goal_y)
            
    def show(self):
       num = len(self.goal_x_list)
       for i in range(num):
           goal, = plt.plot(self.goal_x_list[i], self.goal_y_list[i], 'D', color = 'c')
    
       return goal,

#%%reading logfs module
my_log = LogFS()
my_obs = Obstacle(my_log)
my_obs.show()      
my_goal = Goal(my_log)
my_goal.show()





#%% Animation builder
#this part actually draw the graph several times and snap photos out of it and
#make an animation out of it.
fig = plt.figure(figsize = (10,5))

handles2 = []
labels2 = []
plt.xlabel('x [m]')
plt.ylabel('y [m]')

camera = Camera(fig)
x_data=[]
y_data=[]


#drawing obstacles
obs, = my_obs.show() 
handles2.append(obs)
labels2.append('Obstacle')
#drawing goal points
goal, = my_goal.show()
handles2.append(goal)
labels2.append('Goals')  

path_x_coords = np.load('C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\x.npy')
path_y_coords = np.load('C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\y.npy')
path_psi_coords = np.load('C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\psi.npy')

#drawing starting point
start, = plt.plot(path_x_coords[0], path_y_coords[0], 'X', color = 'black', ms = 10)
handles2.append(start)
labels2.append('Start_Point')

#%%
num = len(path_x_coords)
x = path_x_coords[0]
y = path_y_coords[0]
psi = degrees(path_psi_coords[0])
x_data.append(x)
y_data.append(y)
path, = plt.plot(x_data, y_data, marker = '.', color = 'blue')
robot, = plt.plot(x, y, marker = (3, 0,psi - 90 ), color = 'blue', markersize = 20)
handles2.append(path)
labels2.append('path')
handles2.append(robot)
labels2.append('robot')
x_data=[]
y_data=[]

for i in np.arange(0, num, 5):
    print(f'{i} of {num -1}')
    #drawing the start point
    x = path_x_coords[i]
    y = path_y_coords[i]
    psi = degrees(path_psi_coords[i])
    x_data.append(x)
    y_data.append(y)
    start, = plt.plot(path_x_coords[0], path_y_coords[0], 'X', color = 'black', ms = 10)
    obs, = my_obs.show()
    goal, = my_goal.show()
    path, = plt.plot(x_data, y_data, marker = '.', color = 'blue')
    robot, = plt.plot(x, y, marker = (3, 0,psi - 90 ), color = 'blue', markersize = 20)
    #managing the legend
    plt.legend(handles2, labels2, loc = 'upper left')
    camera.snap()

anim = camera.animate()
anim.save('C:\\Users\\user\\Downloads\\darsi\\2019_2020_1\\Thesis\\Prpf.Dabbene\\Python_2\\17.04.2020\\Local_Path_Planner\\results\\gif\\saman.gif', writer='imagemagick')