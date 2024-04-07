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
    v1 = (2*np.pi*Wheel_Radius*rpm1)/60
    v2 = (2*np.pi*Wheel_Radius*rpm2)/60
    return v1,v2


#Defining the bactracking algorithm 
def B_tracking(Pth,final_val,initial_pt):
    b_track = []
    z = list(Pth)[-1]
    K = Pth.get(z)
    b_track.append(z)
    b_track.append(K)
    print(b_track)
    while K != (initial_pt):  
        K = Pth.get(K)
        b_track.append(K)
    b_track.reverse()
    return (b_track)

def PromptPredefined():
    print("Want to use predefined start, goal and rpms?")
    ans = input("y/n? ")
    if ans == "y":
        return True
    else:
        return False
         
space = np.ones((2001,6001,3),dtype='uint8')  #Creating an matrix with ones, of the shape of boundry shape

# if PromptPredefined() == False:
#     Obstacle_Clearance = int(input("Enter the Obstacle Clearance of the Robot: "))
#     obstacle_space(space)           #Creating the obstacle boundries
#     Obs_Coords= boundry_creation(space)

#     initial_pt = User_Inputs_Start(Obs_Coords)  
#     goal_pt = User_Inputs_Goal(Obs_Coords)
#     rpm_1,rpm_2 = User_Input_rpm()
#     vel_1,vel_2 = rpm_to_velocity(rpm_1,rpm_2)

#     start = (0,0,0,initial_pt)
#     InitialEucledian_dist = np.sqrt(((goal_pt[0] - start[3][0])*2)+((goal_pt[1] - start[3][1])*2))  
#     InitialTotalCost = InitialEucledian_dist   
#     start = (InitialTotalCost,InitialEucledian_dist,0,initial_pt)
# else: 
Obstacle_Clearance = 5
obstacle_space(space)
Obs_Coords= boundry_creation(space)




# cv.imshow("SPACE", space )
# cv.waitKey()
# cv.destroyAllWindows()

initial_pt = (500,1000, 0)
# # goal_pt = (225, 150)
goal_pt = (5750, 1000)
rpm1 = 150
rpm2 = 180
vel_1, vel_2 = rpm_to_velocity(rpm1, rpm2)
start = (0, 0, 0, initial_pt)
print(goal_pt)
InitialEucledian_dist = np.sqrt(((goal_pt[0] - start[3][0])**2)+((goal_pt[1] - start[3][1])**2))
print(InitialEucledian_dist)
InitialTotalCost = InitialEucledian_dist
start = (InitialTotalCost, InitialEucledian_dist, 0, initial_pt)
# print("start node: ", initial_pt, "\ngoal node: ", goal_pt, "\nrpm1: ", rpm1, "rpm2: ", rpm2)
print("start")
print(start)

UncheckedList.put(start)
#print(UncheckedList)
reached=0
while UncheckedList.qsize() != 0:
    a = UncheckedList.get()
    print(a)
    
    #print(goal_pt)
    if CheckedList[int(round(a[3][1])), int(round(a[3][0]))] != 1:
        CheckedList[int(round(a[3][1])) , int(round(a[3][0]))] = 1
        if a[1] > 5:
            zero_n_rpm1(a)     
            rpm1_n_zero(a)
            rpm1_n_rpm1(a)
            zero_n_rpm2(a)
            rpm2_n_zero(a)
            rpm2_n_rpm2(a)
            rpm1_n_rpm2(a)
            rpm2_n_rpm1(a)
            # for i in CloseList:
            #     j = Pth.get(i)
            #     cv.line(space,(i[0],250-i[1]),(j[0],250-j[1]),(0,0,255),1)
                
            #     cv.imshow("Space",space)
            #     if cv.waitKey(50) & 0xFF == ord('q'):
            #         break

        else:
            
            print("success")
            reached=1
            break


# cv.imshow("SPACE", space )
# cv.waitKey()

# cv.destroyAllWindows()
if reached ==1:
    print("hello")
    b = B_tracking(Pth,a, initial_pt)
    
    
    print("path")
    print(b)
    
    
    # for i in CheckedList:
    #     space[250-i[1]][i[0]] = [255,0,0]

    #     cv.imshow("SPACE", space )
    # # cv.waitKey(0)
    
    #     if cv.waitKey(1) & 0xFF == ord('q'):
    #         break
    # print("Closed list")
    for i in CloseList:
        xi,yi,teta = i[0],i[1],i[2]
        j = Pth.get((xi,yi,teta))
        cv.line(space,(int(i[0]), 2000-int(i[1])),(int(j[0]),2000-int(j[1])),(0,0,255),1)
        
        #cv.imshow("Space",space)
        SPACEscaledUp = cv.resize(space,(1202,402))
        cv.imshow("Map",SPACEscaledUp)
        if cv.waitKey(20) & 0xFF == ord('q'):
            break

    for j in b:
        space[2000-int(j[1])][int(j[0])] = [0,255,0]
        plt.scatter(int(j[0]), int(j[1]),color="blue")
        cv.circle(space,(int(j[0]),2000-int(j[1])), 5, (0,255,0), -1)
        SPACEscaledUp = cv.resize(space,(1202,402))
        cv.imshow("SPACE", SPACEscaledUp)
        if cv.waitKey(50) & 0xFF == ord('q'):
            break


else:
    print("The Goal node cannot be reached")


cv.destroyAllWindows()
endTime = time.time()
print("\nrun time = ", endTime - startTime, "seconds")