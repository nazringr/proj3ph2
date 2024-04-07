import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
from queue import PriorityQueue
import math
import time

boundry = []    
Pth = {}        #Stores the path for backtracking
UncheckedList = PriorityQueue()     #Used to store unvisited nodes
b_track = []            
CloseList = []
CheckedList = np.zeros((2000,6000),dtype='float64')     # Used to store the visited nodes
Robot_Radius = 11     #Needs to be changed to 10.5
Wheel_Radius = 3.3
Wheel_Length = 16
total_clearance = 5

startTime = time.time()

#Creating the Obstacle Space
#Obtacle with Obstacle, Obstacle Clearance and Robot Radius
def obstacle_space(space):
    h,w,_ = space.shape
    for l in range(h):
        for m in range(w):
            if ((2000-l) - 5 < 0) or ((m) - 5 < 0) or ((2000-l) -1995 > 0) or ((m) - 5995 > 0): #boundary
                space[l][m] = [0,0,255]
                
            if (m > ((1500-Obstacle_Clearance)-Robot_Radius)) and (m < (1750+Obstacle_Clearance+Robot_Radius)) and (2000-l < 2000) and (2000-l > ((1000-Obstacle_Clearance)-Robot_Radius)):   #rectangle1 with Obstacle clearance and Robot Radius
                space[l][m] = [255,0,0]
# 
            if (m > (1500-Obstacle_Clearance)) and (m < (1750+Obstacle_Clearance)) and (2000-l < 2000) and (2000-l > (1000-Obstacle_Clearance)):   #rectangle1 with Obstacle Clearance
                space[l][m] = [0,255,0]
                # 
            if (m > 1500) and (m < 1750) and (2000-l < 2000) and (2000-l > 1000):   #rectangle1
                space[l][m] = [0,0,255]
                # 
            if (m > ((2500-Obstacle_Clearance)-Robot_Radius)) and (m < (2750+Obstacle_Clearance+Robot_Radius)) and (2000-l >0) and (2000-l < (1000+Obstacle_Clearance+Robot_Radius)):  #rectangle2 with Obstacle clearance and Robot Radius
                space[l][m] = [255,0,0]
# 
            if (m > (2500-Obstacle_Clearance)) and (m < (2750+Obstacle_Clearance)) and (2000-l >0) and (2000-l < (1000+Obstacle_Clearance)):  #rectangle2 with Obstacle clearance
                space[l][m] = [0,255,0]
# 
            if (m > 2500) and (m < 2750) and (2000-l >0) and (2000-l < 1000):  #rectangle2
                space[l][m] = [0,0,255]
            # 
            if (math.pow((m-4200),2) + math.pow(((2000-l)-1200),2) - math.pow((600+Obstacle_Clearance+Robot_Radius),2)) < 0:    #Circle with Obstacle clearance and Robot Radius
                space[l][m] = [255,0,0]

            if (math.pow((m-4200),2) + math.pow(((2000-l)-1200),2) - math.pow((600+Obstacle_Clearance),2)) < 0: #Circle with Obstacle Clearance
                space[l][m] = [0,255,0]
                # 
            if (math.pow((m-4200),2) + math.pow(((2000-l)-1200),2) - math.pow(600,2)) < 0: #Circle
                space[l][m] = [0,0,255]                
    return 



#taking the obstacle coordinates into a list
def boundry_creation(space):
    h,w,_ = space.shape
    for l in range(h):
        for m in range(w):
            if space[l][m][2] == 255:
                boundry.append((m,2000-l))
            if space[l][m][1] == 255:
                boundry.append((m,2000-l))
            if space[l][m][0] == 255:
                boundry.append((m,2000-l))
    return boundry




#Getting User Inputs For the Start node from the user
def User_Inputs_Start(Obs_Coords):
    while True:
        x = int(input("Enter the Initial x node: "))
        y = int(input("Enter the Initial y node: "))
        z = int(input("Enter the orientation of robot at start pt.(in degrees): "))
        
        if((x>=0) and (x<=6000)) and (y>=0) and (y<=2000):
            if (x,y) not in Obs_Coords :
                start_node = (x,y,z)
                return start_node
            else:
                print("The Entered Start Node is in obstacle space")
#Getting User Input for the Goal Node from the user
def User_Inputs_Goal(Obs_Coords):
    while True:
        x = int(input("Enter the Goal x node: "))
        y = int(input("Enter the Goal y node: "))
        
        #goal_node = (x,y)
        if((x>=0) and (x<=6000)) and (y>=0) and (y<=2000):
            if (x,y) not in Obs_Coords :
                goal_node=(x,y)
                break
            else:
                print("The Entered Goal Node is in obstacle space")
    return goal_node

def User_Input_rpm():
    rpm1 = int(input("Enter the first RPM: "))
    rpm2 = int(input("Enter the second RPM: "))
    
    return (rpm1,rpm2)
def angle_conversion(theta):
    if theta > 360:
        theta = theta % 360
        return theta
    elif theta < -360:
        theta = (-theta % 360)*(-1) 
        return theta
    else:
        return theta
    
def a_star_function(pos,ul,ur):
    old_x = round(pos[0])
    old_y = round(pos[1])
    theta = round(pos[2])
    dt=0.1
    theta_n = (np.pi * (theta/180))
    Cost = 0
    Xn = old_x
    Yn = old_y
    
    Xn += 0.5*Wheel_Radius*(ul+ur)*np.cos(theta_n)*dt
    Yn += 0.5*Wheel_Radius*(ul+ur)*np.sin(theta_n)*dt
        # print(Xn)
        # print(Yn)
    theta_n += (Wheel_Radius/Wheel_Length)*(ur-ul)
    theta_n = (180*(theta_n))/np.pi
    theta_n = angle_conversion(theta_n)
    if (0<=round(Xn)<6000) and (0<=round(Yn)<2000):
        if (CheckedList[int(round(Yn))][int(round(Xn))] != 1) and ((round(Xn),round(Yn)) not in Obs_Coords):
            # plt.scatter(Xn,Yn,color="blue")
            Cost = Cost+math.sqrt(math.pow((0.5*Wheel_Radius * (ul + ur) * np.cos(theta_n) * dt),2)+math.pow((0.5*Wheel_Radius * (ul + ur) * np.sin(theta_n) * dt),2))
            CloseList.append((round(Xn),round(Yn),round(theta_n))) 
            Eucledian_dist = np.sqrt(((goal_pt[0] - Xn)**2)+((goal_pt[1] - Yn)**2))
            TotalCost = Cost + Eucledian_dist
            for m in range(UncheckedList.qsize()):
                if UncheckedList.queue[m][3] == (round(Xn),round(Yn),round(theta_n)):
                    if UncheckedList.queue[m][0] > TotalCost:
                        UncheckedList.queue[m] = (TotalCost,Eucledian_dist,Cost,(round(Xn),round(Yn),round(theta_n)))
                        Pth[(Xn,Yn,theta_n)] = (old_x,old_y,theta)
                        return
                    else:
                        return
            UncheckedList.put((TotalCost,Eucledian_dist,Cost,(round(Xn),round(Yn),round(theta_n))))
            Pth[(round(Xn),round(Yn),round(theta_n))] = (old_x,old_y,theta)
