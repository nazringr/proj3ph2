#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from numpy import pi, cos, sin, sqrt
import time

import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
from queue import PriorityQueue
import math

startTime = time.time()

boundry = []
Pth = {}      
rpmDict = {}
UncheckedList = PriorityQueue()    
backtrack_ = []            
CloseList = []
CheckedList = np.zeros((250,600),dtype='uint8')    
r_radius = 22
w_radius = 3.3
w_length = 28.7

Obstacle_Clearance = 1 

def canvas_space(space):
    h,w,_ = space.shape
    for l in range(h):
        for m in range(w):
            space[l][m] = [224,224,224]
            if ((200-l) - 5 < 0) or ((m) - 5 < 0) or ((200-l) -195 > 0) or ((m) - 595 > 0): #boundary
                space[l][m] = [102,255,178]
                
            if (m > ((150-Obstacle_Clearance)-r_radius)) and (m < (175+Obstacle_Clearance+r_radius)) and (200-l < 200) and (200-l > ((100-Obstacle_Clearance)-r_radius)):   #rectangle1 with Obstacle clearance and Robot Radius
                space[l][m] = [192,192,192]

            if (m > (150-Obstacle_Clearance)) and (m < (175+Obstacle_Clearance)) and (200-l < 200) and (200-l > (100-Obstacle_Clearance)):   #rectangle1 with Obstacle Clearance
                space[l][m] = [102,255,178]
                
            if (m > 150) and (m < 175) and (200-l < 200) and (200-l > 100):   #rectangle1
                space[l][m] = [255,204,153]
                
            if (m > ((250-Obstacle_Clearance)-r_radius)) and (m < (275+Obstacle_Clearance+r_radius)) and (200-l >0) and (200-l < (100+Obstacle_Clearance+r_radius)):  #rectangle2 with Obstacle clearance and Robot Radius
                space[l][m] = [192,192,192]

            if (m > (250-Obstacle_Clearance)) and (m < (275+Obstacle_Clearance)) and (200-l >0) and (200-l < (100+Obstacle_Clearance)):  #rectangle2 with Obstacle clearance
                space[l][m] = [102,255,178]

            if (m > 250) and (m < 275) and (200-l >0) and (200-l < 100):  #rectangle2
                space[l][m] = [255,204,153]
            
            if (math.pow((m-420),2) + math.pow(((200-l)-120),2) - math.pow((60+Obstacle_Clearance+r_radius),2)) < 0:    #Circle with Obstacle clearance and Robot Radius
                space[l][m] = [192,192,192]

            if (math.pow((m-420),2) + math.pow(((200-l)-120),2) - math.pow((60+Obstacle_Clearance),2)) < 0: #Circle with Obstacle Clearance
                space[l][m] = [102,255,178]
                
            if (math.pow((m-420),2) + math.pow(((200-l)-120),2) - math.pow(60,2)) < 0: #Circle
                space[l][m] = [255,204,153]                
    return 

def boundry_creation(space):
    h,w,_ = space.shape
    for l in range(h):
        for m in range(w):
            if (space[l][m][0] != 224) and (space[l][m][2] > 20 or space[l][m][1] > 20 or space[l][m][0] > 20):
                boundry.append((m,200-l))
    return boundry

def PromptPredefined():
    print("Run predefined inputs")
    ans = input(" y/n? ")
    if ans == "y":
        return True
    else:
        return False

def User_Inputs_Start(Obs_Coords):
    while True:
        x = int(input("Enter the Initial x node: "))
        y = int(input("Enter the Initial y node: "))
        z = int(input("Enter the orientation of robot at start pt.(in degrees): "))
        
        if((x>=0) or (x<=600)) and (y>=0) or (y<=250):
            if (x,y) not in Obs_Coords :
                start_node = (x,y,z)
                return start_node
            else:
                print("The Entered Start Node is in obstacle space")

def User_Inputs_Goal(Obs_Coords):
    while True:
        x = int(input("Enter the Goal x node: "))
        y = int(input("Enter the Goal y node: "))
        
     
        if((x>=0) or (x<=600)) and (y>=0) or (y<=250):
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
    
fig, ax = plt.subplots()


def func_Cost(old_x,old_y,theta,ul,ur):
    rpmList = []
    t = 0
    dt = 0.1
    theta_n = (np.pi * (theta/180))
    D = 0
    Xn = old_x
    Yn = old_y
    while t<1:
        rpmLeft = (ul * 60) / (2 * np.pi*w_radius)
        rpmRight = (ur * 60) / (2 * np.pi*w_radius)
        rpmList += {(round(rpmLeft, 3), round(rpmRight, 3), round(theta_n, 3))}
    
        t = t+dt
        Xn += 0.5*w_radius*(ul+ur)*np.cos(theta_n)*dt
        Yn += 0.5*w_radius*(ul+ur)*np.sin(theta_n)*dt
     
        theta_n += (w_radius/w_length)*(ur-ul)
        if (round(Xn),round(Yn)) not in Obs_Coords:
            D = D+math.sqrt(math.pow((0.5*w_radius * (ul + ur) * np.cos(theta_n) * dt),2) + math.pow((0.5*w_radius * (ul + ur) * np.sin(theta_n) * dt),2))
            
        else:
            D = 0
            Xn = old_x
            Yn = old_y
            return (D,Xn,Yn,theta_n)
    theta_n = (180*(theta_n))/np.pi
    theta_n = angle_conversion(theta_n)
    return (D, Xn, Yn, theta_n, rpmList)

def a_star_function(pos,vel1,vel2):
    Costfn = func_Cost(pos[0],pos[1],pos[2],vel1,vel2)
    if Costfn[0] !=0:
        newPos = (Costfn[1],Costfn[2],Costfn[3])
        childrpmList = Costfn[4]
        Cost = Costfn[0]
        x,y,_ = newPos
        if (0<=round(x)<600) and (0<=round(y)<250):
            if (CheckedList[int(round(y))][int(round(x))] != 1) and ((int(round(x)),int(round(y))) not in Obs_Coords):
                    CloseList.append((round(x),round(y),newPos[2]))
                    Eucledian_dist = np.sqrt(((goal_pt[0] - newPos[0])**2)+((goal_pt[1] - newPos[1])**2))
                    TotalCost = Cost + Eucledian_dist
                    for m in range(UncheckedList.qsize()):
                        if UncheckedList.queue[m][3] == newPos:
                            if UncheckedList.queue[m][0] > TotalCost:
                                UncheckedList.queue[m] = (TotalCost,Eucledian_dist,Cost,newPos)
                                Pth[(round(newPos[0]),round(newPos[1]),round(newPos[2]))] = (round(pos[0]),round(pos[1]),round(pos[2]))
                                rpmDict[(round(newPos[0]),round(newPos[1]),round(newPos[2]))] = childrpmList
                                return
                            else:
                                return
                    UncheckedList.put((TotalCost,Eucledian_dist,Cost,newPos))
                    Pth[(round(newPos[0]),round(newPos[1]),round(newPos[2]))] = (round(pos[0]),round(pos[1]),round(pos[2]))
                    rpmDict[(round(newPos[0]),round(newPos[1]),round(newPos[2]))] = childrpmList
                    

#left wheel rpm = 0 and right wheel rpm = rpm1
def zero_n_rpm1(a):
    pos = a[3]
    a_star_function(pos,0,vel_1)

#left wheel rpm = rpm1 and right wheel rpm = 0
def rpm1_n_zero(a):
    pos = a[3] 
    a_star_function(pos,vel_1,0)    
    
#left wheel rpm = rpm1 and right wheel rpm = rpm1
def rpm1_n_rpm1(a):
    pos = a[3]
    a_star_function(pos,vel_1,vel_1) 

# left wheel rpm = zero and right wheel rpm = rpm2
def zero_n_rpm2(a):
    pos = a[3]  
    a_star_function(pos,0,vel_2)
# left wheel rpm = rpm2 and right wheel rpm = zero
def rpm2_n_zero(a):
    pos = a[3]  
    a_star_function(pos,vel_2,0)

#left wheel rpm = rpm2 and right wheel rpm = rpm2
def rpm2_n_rpm2(a):
    pos = a[3] 
    a_star_function(pos,vel_2,vel_2)

#left wheel rpm = rpm1 and right wheel rpm = rpm2
def rpm1_n_rpm2(a):
    pos = a[3] 
    a_star_function(pos,vel_1,vel_2)
    
def rpm2_n_rpm1(a):
    pos = a[3] 
    a_star_function(pos,vel_2,vel_1)

#Converting the rpm to velocity
def rpm_to_velocity(rpm1,rpm2):
    v1 = (2*np.pi*w_radius*rpm1)/60
    v2 = (2*np.pi*w_radius*rpm2)/60
    return v1,v2


def backtrack_(Pth, initial_pt):
    b_track = []
    a = list(Pth)[-1]
    K = Pth.get(a)
    b_track.append(a)
    b_track.append(K)
    
    while K != (initial_pt):  
        K = Pth.get(K)
        
        b_track.append(K)
    b_track.reverse()
    return (b_track)

def generateVelocityPath(path, rpmDict):
    velocityPath = []
    for i, key in enumerate(path):
        if i !=0:
            velocityPath.append(rpmDict[key])
    
    return velocityPath
         
space = np.ones((201,601,3),dtype='uint8') 

if PromptPredefined() == False:
    Obstacle_Clearance = int(input("Enter the Obstacle Clearance of the Robot: "))
    canvas_space(space)       
    Obs_Coords= boundry_creation(space)

    initial_pt = User_Inputs_Start(Obs_Coords)  
    goal_pt = User_Inputs_Goal(Obs_Coords)
    rpm_1,rpm_2 = User_Input_rpm()
    vel_1,vel_2 = rpm_to_velocity(rpm_1,rpm_2)

    start = (0,0,0,initial_pt)
    InitialEucledian_dist = np.sqrt(((goal_pt[0] - start[3][0])**2)+((goal_pt[1] - start[3][1])**2))  
    InitialTotalCost = InitialEucledian_dist   
    start = (InitialTotalCost,InitialEucledian_dist,0,initial_pt)

else: 
    Obstacle_Clearance = 1
    canvas_space(space)
    Obs_Coords= boundry_creation(space)

    initial_pt = (100, 100, 0)
    # goal_pt = (225, 150)
    goal_pt = (575, 100)
    rpm1 = 5
    rpm2 = 10

    vel_1, vel_2 = rpm_to_velocity(rpm1, rpm2)

    start = (0, 0, 0, initial_pt)
    InitialEucledian_dist = np.sqrt(((goal_pt[0] - start[3][0])**2)+((goal_pt[1] - start[3][1])**2))
    InitialTotalCost = InitialEucledian_dist
    start = (InitialTotalCost, InitialEucledian_dist, 0, initial_pt)
    print("start node: ", initial_pt, "\ngoal node: ", goal_pt, "\nrpm1: ", rpm1, "rpm2: ", rpm2)

UncheckedList.put(start)

reached=0
while UncheckedList.qsize() != 0:
    a = UncheckedList.get()
    # print(a)
    #print(goal_pt)
    if CheckedList[int(round(a[3][1])), int(round(a[3][0]))] != 1:
        CheckedList[int(round(a[3][1])), int(round(a[3][0]))] = 1
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
    b = backtrack_(Pth, initial_pt)
   
    velPath = generateVelocityPath(b, rpmDict)
    
    for i in CloseList:
        xi,yi,teta = i[0],i[1],round(i[2])
        j = Pth.get((xi,yi,teta))
        cv.line(space,(int(i[0]),200-int(i[1])),(int(j[0]),200-int(j[1])),(0,0,255),1)
        
        # cv.imshow("Space",space)
        ScaledUp = cv.resize(space, (1202, 402))
        cv.imshow("ScaledUp", ScaledUp)
        if cv.waitKey(20) & 0xFF == ord('q'):
            break

    for j in b:
        space[200-int(j[1])][int(j[0])] = [0,255,0]
        cv.circle(space,(int(j[0]),200-int(j[1])), 1, (0,255,0), -1)
        # cv.imshow("SPACE", space )
        SPACEscaledUp = cv.resize(space, (1202, 402))
        cv.imshow("Space", SPACEscaledUp)
        if cv.waitKey(50) & 0xFF == ord('q'):
            break

else:
    print("Error")


def publishVelocities(velocityList):

    msg = Twist()
    rclpy.init(args=None)
    node = rclpy.create_node('robot_talker')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    t = 0
    dt = .1 # seconds
    r = .033 # meters
    L = .16 # meters
    i = 0
    rowNumber = 1
    numberOfRows = len(velocityList)
    linearFactorStraight = 1.4459225 * 2.7
    linearFactorCurved = 2
    angularFactor = 4.3190914 * 2.3
 # subscribe to odom topic, at each point create an x, y and theta

    while rclpy.ok():

        for row in velocityList:
            print("\nrow: ", row)

            for velocities in row:
                UL = velocities[0]
                UR = velocities[1]
                theta = velocities[2]
                thetaN = pi * theta / 180

                if t <= 1:
                    thetaN += (r / L) * (UR - UL) * 2*pi / 60 * dt # rad
                    Vx = (r / 2) * (UL + UR) * 2*pi/60 * cos(thetaN) # m/s check units
                    Vy = (r / 2) * (UL + UR) * 2*pi/60 * sin(thetaN)
                    if UL == UR:
                 
                        msg.linear.x = sqrt(Vx**2 + Vy**2) * linearFactorStraight
                        msg.angular.z = 0.0
                    
                    if UL != UR:
                  
                        msg.linear.x = sqrt(Vx**2 + Vy**2) * linearFactorCurved
                        msg.angular.z = (r / L) * (UR - UL) * 2*pi/60 * angularFactor

                    pub.publish(msg)
                    rclpy.spin_once(node, timeout_sec=0.08);
                
                
                t = round(t + dt, 2)

                if t >= .9:
                    t = 0
                    print("\nt > 1, stopping robot, moving onto next row")
                    msg.angular.z = 0.0
                    msg.linear.x = 0.0
                    pub.publish(msg)
                    break
                
                i += 1

            if int(rowNumber) >= numberOfRows:
                print("\nending script")
                return
            rowNumber += 1



publishVelocities(velPath)
rclpy.shutdown();
cv.destroyAllWindows()
endTime = time.time()
print("\nrun time = ", endTime - startTime, "seconds")
