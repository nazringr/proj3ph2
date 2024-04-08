import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
from queue import PriorityQueue
import math
import time

boundry = []    
path = {}      
unvisited = PriorityQueue()     
backtrack_list = []            
CloseList = []
visited = np.zeros((250,600),dtype='float64')     
r_radius = 11   
w_radius = 3.3
w_length = 16

startTime = time.time()

def canvas_space(space):
    h,w,_ = space.shape
    for l in range(h):
        for m in range(w):
            space[l][m] = [224,224,224]
            if ((200-l) - 5 < 0) or ((m) - 5 < 0) or ((200-l) -195 > 0) or ((m) - 595 > 0): #boundary
                space[l][m] = [102,255,178]
                
            if (m > ((150-obstacle_clearance)-r_radius)) and (m < (175+obstacle_clearance+r_radius)) and (200-l < 200) and (200-l > ((100-obstacle_clearance)-r_radius)):   #rectangle1 with Obstacle clearance and Robot Radius
                space[l][m] = [192,192,192]

            if (m > (150-obstacle_clearance)) and (m < (175+obstacle_clearance)) and (200-l < 200) and (200-l > (100-obstacle_clearance)):   #rectangle1 with Obstacle Clearance
                space[l][m] = [102,255,178]
                
            if (m > 150) and (m < 175) and (200-l < 200) and (200-l > 100):   #rectangle1
                space[l][m] = [255,204,153]
                
            if (m > ((250-obstacle_clearance)-r_radius)) and (m < (275+obstacle_clearance+r_radius)) and (200-l >0) and (200-l < (100+obstacle_clearance+r_radius)):  #rectangle2 with Obstacle clearance and Robot Radius
                space[l][m] = [192,192,192]

            if (m > (250-obstacle_clearance)) and (m < (275+obstacle_clearance)) and (200-l >0) and (200-l < (100+obstacle_clearance)):  #rectangle2 with Obstacle clearance
                space[l][m] = [102,255,178]

            if (m > 250) and (m < 275) and (200-l >0) and (200-l < 100):  #rectangle2
                space[l][m] = [255,204,153]
            
            if (math.pow((m-420),2) + math.pow(((200-l)-120),2) - math.pow((60+obstacle_clearance+r_radius),2)) < 0:    #Circle with Obstacle clearance and Robot Radius
                space[l][m] = [192,192,192]

            if (math.pow((m-420),2) + math.pow(((200-l)-120),2) - math.pow((60+obstacle_clearance),2)) < 0: #Circle with Obstacle Clearance
                space[l][m] = [102,255,178]
                
            if (math.pow((m-420),2) + math.pow(((200-l)-120),2) - math.pow(60,2)) < 0: #Circle
                space[l][m] = [255,204,153]                
    return 
def create_boundry(space):
    h,w,_ = space.shape
    for l in range(h):
        for m in range(w):
            if (space[l][m][0] != 224) and (space[l][m][2] > 20 or space[l][m][1] > 20 or space[l][m][0] > 20):
                boundry.append((m,200-l))
    return boundry

# 
def input_start(obs_coord):
    while True:
        x = int(input("Enter the Initial x node: "))
        y = int(input("Enter the Initial y node: "))
        z = int(input("Enter the orientation of robot (Start): "))
        
        if((x>=0) and (x<=600)) and (y>=0) and (y<=200):
            if (x,y) not in obs_coord :
                start_node = (x,y,z)
                return start_node
            else:
                print("The Entered Start Node is in obstacle space")
#Getting User Input for the Goal Node from the user
def input_goal(obs_coords):
    while True:
        x = int(input("Enter the Goal x node: "))
        y = int(input("Enter the Goal y node: "))
        
        #goal_node = (x,y)
        if((x>=0) and (x<=600)) and (y>=0) and (y<=200):
            if (x,y) not in obs_coords :
                goal_node=(x,y)
                break
            else:
                print("The Entered Goal Node is in obstacle space")
    return goal_node

def input_rpm():
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
    
def astar(pos,ul,ur):
    old_x = round(pos[0])
    old_y = round(pos[1])
    theta = round(pos[2])
    dt=0.1
    theta_n = (np.pi * (theta/180))
    Cost = 0
    Xn = old_x
    Yn = old_y
    
    Xn += 0.5*w_radius*(ul+ur)*np.cos(theta_n)*dt
    Yn += 0.5*w_radius*(ul+ur)*np.sin(theta_n)*dt

    theta_n += (w_radius/w_length)*(ur-ul)
    theta_n = (180*(theta_n))/np.pi
    theta_n = angle_conversion(theta_n)
    if (0<=round(Xn)<600) and (0<=round(Yn)<200):
        if (visited[int(round(Yn))][int(round(Xn))] != 1) and ((round(Xn),round(Yn)) not in obstacle_coordinates):

            Cost = Cost+math.sqrt(math.pow((0.5*w_radius * (ul + ur) * np.cos(theta_n) * dt),2)+math.pow((0.5*w_radius * (ul + ur) * np.sin(theta_n) * dt),2))
            CloseList.append((round(Xn),round(Yn),round(theta_n))) 
            Eucledian_dist = np.sqrt(((goal_pt[0] - Xn)**2)+((goal_pt[1] - Yn)**2))
            TotalCost = Cost + Eucledian_dist
            for m in range(unvisited.qsize()):
                if unvisited.queue[m][3] == (round(Xn),round(Yn),round(theta_n)):
                    if unvisited.queue[m][0] > TotalCost:
                        unvisited.queue[m] = (TotalCost,Eucledian_dist,Cost,(round(Xn),round(Yn),round(theta_n)))
                        path[(Xn,Yn,theta_n)] = (old_x,old_y,theta)
                        return
                    else:
                        return
            unvisited.put((TotalCost,Eucledian_dist,Cost,(round(Xn),round(Yn),round(theta_n))))
            path[(round(Xn),round(Yn),round(theta_n))] = (old_x,old_y,theta)
        

#Converting the rpm to velocity
def rpm_to_velocity(rpm1,rpm2):
    v1 = (2*np.pi*w_radius*rpm1)/60
    v2 = (2*np.pi*w_radius*rpm2)/60
    return v1,v2


def zero_n_rpm1(a):
    pos = a[3]
    astar(pos,0,vel_1)

def rpm1_n_zero(a):
    pos = a[3] 
    astar(pos,vel_1,0)    

def rpm1_n_rpm1(a):
    pos = a[3]
    astar(pos,vel_1,vel_1) 

def zero_n_rpm2(a):
    pos = a[3]  
    astar(pos,0,vel_2)

def rpm2_n_zero(a):
    pos = a[3]  
    astar(pos,vel_2,0)

def rpm2_n_rpm2(a):
    pos = a[3] 
    astar(pos,vel_2,vel_2)

def rpm1_n_rpm2(a):
    pos = a[3] 
    astar(pos,vel_1,vel_2)
    
def rpm2_n_rpm1(a):
    pos = a[3] 
    astar(pos,vel_2,vel_1)


#Defining the bactracking algorithm 
def backtrack(path,final_val,initial_pt):
    backtrack_ = []
    z = list(path)[-1]
    K = path.get(z)
    backtrack_.append(z)
    backtrack_.append(K)
    print(backtrack_)
    while K != (initial_pt):  
        K = path.get(K)
        backtrack_.append(K)
    backtrack_.reverse()
    return (backtrack_)

def predefined():
    print("Would you like to use predefined start, goal and rpms?")
    ans = input("y/n? ")
    if ans == "y":
        return True
    else:
        return False
         
space = np.ones((201,601,3),dtype='uint8')  

if predefined() == False:
    obstacle_clearance = int(input("Enter the Obstacle Clearance of the Robot: "))
    canvas_space(space)          
    obstacle_coordinates= create_boundry(space)

    init_pt = input_start(obstacle_coordinates)  
    goal_pt = input_goal(obstacle_coordinates)
    rpm_1,rpm_2 = input_rpm()
    vel_1,vel_2 = rpm_to_velocity(rpm_1,rpm_2)

    start = (0,0,0,init_pt)
    init_dist = np.sqrt(((goal_pt[0] - start[3][0])**2)+((goal_pt[1] - start[3][1])**2))  
    init_total_cost = init_dist   
    start = (init_total_cost,init_dist,0,init_pt)
else: 
    obstacle_clearance = 1
    canvas_space(space)
    obstacle_coordinates= create_boundry(space)

    init_pt = (100, 100, 0)
    goal_pt = (575, 100)
    rpm1 = 40
    rpm2 = 35

    vel_1, vel_2 = rpm_to_velocity(rpm1, rpm2)

    start = (0, 0, 0, init_pt)
    init_dist = np.sqrt(((goal_pt[0] - start[3][0])**2)+((goal_pt[1] - start[3][1])**2))
    init_total_cost = init_dist
    start = (init_total_cost, init_dist, 0, init_pt)
    print("start node: ", init_pt, "\ngoal node: ", goal_pt, "\nrpm1: ", rpm1, "rpm2: ", rpm2)


unvisited.put(start)

reached=0
while unvisited.qsize() != 0:
    a = unvisited.get()
    if visited[int(round(a[3][1])), int(round(a[3][0]))] != 1:
        visited[int(round(a[3][1])) , int(round(a[3][0]))] = 1
        if a[1] > 5:
            zero_n_rpm1(a)     
            rpm1_n_zero(a)
            rpm1_n_rpm1(a)
            zero_n_rpm2(a)
            rpm2_n_zero(a)
            rpm2_n_rpm2(a)
            rpm1_n_rpm2(a)
            rpm2_n_rpm1(a)
        else:
            print("Goal reached")
            reached=1
            break



if reached ==1:

    b = backtrack(path,a, init_pt)
    print(b)
    
    for i in CloseList:
        xi,yi,theta = i[0],i[1],i[2]
        j = path.get((xi,yi,theta))
        cv.line(space,(int(i[0]), 200-int(i[1])),(int(j[0]),200-int(j[1])),(0,0,255),1)
        scaled = cv.resize(space,(1202,402))
        cv.imshow("Map",scaled)
        if cv.waitKey(20) & 0xFF == ord('q'):
            break

    for j in b:
        space[200-int(j[1])][int(j[0])] = [0,255,0]
        plt.scatter(int(j[0]), int(j[1]),color="blue")
        cv.circle(space,(int(j[0]),200-int(j[1])), 1, (0,255,0), -1)
        space_scaled = cv.resize(space,(1202,402))
        cv.imshow("Space", space_scaled )
        if cv.waitKey(50) & 0xFF == ord('q'):
            break


else:
    print("Error")

cv.destroyAllWindows()
endTime = time.time()
print("\nrun time = ", endTime - startTime, "seconds")
