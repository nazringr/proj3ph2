#!/usr/bin/env python3

import math
import time
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import rclpy
from rclpy.node import Node
from heapq import heappush, heappop
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt

class vel_publisher(Node):
    def __init__(self):
        super().__init__('vel_publisher')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 100)

        self.msg = Twist()
        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = 0.0

        self.vel_pub.publish(self.msg)

        self.th_dist = 0.01
        self.ndist = int(1 / self.th_dist)
        self.sizeofx = 10 * self.ndist
        self.thrang = 10
        self.nare = int(360 / self.thrang)

        self.total = 0.18

        self.actions = [
            [2, 0], [0, 2], [2, 2],
            [0, 3], [3, 0], [3, 3],
            [2, 3], [3, 2]
        ]

        self.start = None
        self.goal = None
        self.parent = {}

    def cal_vel(self, xi, yi, the_i, vel_l, vel_r):
        radius = 0.033 #in metres
        length = 0.287 #in metres
        delt = 10
        vel_l = vel_l*2*math.pi/60
        vel_r = vel_r*2*math.pi/60
        thetan = 3.14 * the_i / 180
        angular_vel = (radius / length) * (vel_r - vel_l) 
        del_theta = angular_vel + thetan
        vel_x = (radius / 2) * (vel_l + vel_r) * math.cos(del_theta) 
        vel_y = (radius / 2) * (vel_l + vel_r) * math.sin(del_theta) 
        vel_tot = math.sqrt(vel_x** 2 + vel_y** 2)
        thet_f = (180*del_theta/ 3.14)   
        return vel_tot, angular_vel, thet_f

    def distance(self, start_coordinate, goal_coordinate):
        sx,sy = start_coordinate[0],start_coordinate[1]
        gx,gy = goal_coordinate[0],goal_coordinate[1]
        return math.sqrt((gx-sx)**2 + (gy-sy)**2)#instead of using scipy.spatial importing distance
    
    def norm_node(self, Node):
        x,y,t = Node[0],Node[1],Node[2]
        x = round(x/self.th_dist)* self.th_dist
        y = round(y/self.th_dist)* self.th_dist
        t = round(t/self.thrang) * self.thrang
        x=round(x,4)
        y=round(y,4)
        n=t//360
        t=t-(360*n)
        t=(t/self.thrang)
        return [x,y,int(t)]

    def string_from_list(self, s):  
            str1 = ""  
            for ele in s:  
                str1 += str(ele)  
            return str1  

    def check_bound(self, i, j):
        if (i<self.total or j>=6-self.total or j<self.total or i>=6-self.total):
            return 0
        else:
            return 1


    def check_obs(self, x, y):
        circle_obs = ((np.square(x-4.2))+ (np.square(y-1.2)) <=np.square(0.6+self.total))
        square_obs1=(x>=1.5-self.total) and (x<=1.75+self.total) and (y>=1-self.total) and (y <= 2)
        square_obs2=(x>=2.5-self.total) and (x<=2.75+self.total) and (y>=0) and (y <= 1+self.total)
        
        boundary=(x<=self.total) or (x>=6-self.total) or (y <= self.total) or (y>=2-self.total)
        if circle_obs or square_obs1 or square_obs2 or boundary:
            object_value =0
        else:
            object_value = 1
    
        return object_value

    def draw_motion(self, xi, yi, the_i, vel_l, vel_r):
        t = 0
        radius = 0.033
        length = 0.287
        delt = 1
        Xn=xi
        Yn=yi
        Thetan = math.radians(the_i)
        while t<10:
            t = t + delt
            Xs = Xn
            Ys = Yn
            Xn += (0.5*radius) * (vel_l + vel_r) * math.cos(Thetan) * delt
            Yn += (0.5*radius) * (vel_l + vel_r) * math.sin(Thetan) * delt
            Thetan += (radius / length) * (vel_r - vel_l) * delt
            plt.plot([Xs, Xn], [Ys, Yn], color="red")
        Thetan = math.degrees(Thetan)
        return [Xn, Yn, Thetan]


    def exec_robot_motion(self, xi, yi, the_i, vel_l, vel_r, s, n):
        t = 0
        radius = 0.033
        length = 0.287
        delt = 1
        Xn=xi
        Yn=yi
        length=0
        Thetan = math.radians(the_i*self.thrang)
        while t<10:
            t = t + delt
            Xs = Xn
            Ys = Yn
            Xn += (0.5*radius) * (vel_l + vel_r) * math.cos(Thetan) * delt
            Yn += (0.5*radius) * (vel_l + vel_r) * math.sin(Thetan) * delt
            length+=self.distance([Xs,Ys],[Xn,Yn])
            status=self.check_bound(Xn,Yn)
            flag=self.check_obs(Xn,Yn)
            if (status!=1) or (flag != 1):
                return None
            Thetan += (radius / length) * (vel_r - vel_l) * delt
            s.append((Xs,Ys))
            n.append((Xn,Yn))
        Thetan = math.degrees(Thetan)
    
        return [Xn, Yn, Thetan,length]

    def goal_reached(self, a, final_node):
        if((np.square(a[0]-final_node[0]))+ (np.square(a[1]-final_node[1])) <=np.square(0.05)) :
                        return 0
        else:
            return 1

    def run(self):
        # enter clearance 
        self.total=float(input("Enter clearance for the robot")) 
        
        l_rpm=int(input("Enter the L-rpm:"))
        r_rpm=int(input("Enter the R-rpm:"))
        actions=[[l_rpm,0],[0,l_rpm],[l_rpm,l_rpm],[0,r_rpm],[r_rpm,0],[r_rpm,r_rpm],[l_rpm,r_rpm],[r_rpm,l_rpm]]
        
        startx = float(input("Enter start point x coordinate:"))
        
        starty = float(input("Enter start point y coordinate:"))
    
        starting_ang = int(input("Enter start orientation in degrees:"))
        obstacle_st = self.check_obs(startx,starty)
        bound_st = self.check_bound(startx,starty)
        
        while(obstacle_st!=1 or bound_st!=1):
            print("Incorrect start point! Enter a valid start point:")
            startx = float(input("Enter start point x coordinate:"))
        
            starty = float(input("Enter start point y coordinate:"))
            
            starting_ang = int(input("Enter start orientation in degrees:"))
            obstacle_st = self.check_obs(startx,starty)
            bound_st = self.check_bound(startx,starty)
        
        start=self.norm_node([startx,starty,starting_ang])
        norm_current_node=start
        goalx=float(input("Enter goal point x coordinate:"))
    
        goaly=float(input("Enter goal point y coordinate:"))
    
        goal_ang=0
        final_obs=self.check_obs(goalx,goaly)
        final_bound=self.check_bound(goalx,goaly)
        
        
        while(final_obs!=1 or final_bound!=1):
            print("Incorrect goal point! Enter a valid goal point:")
            goalx=float(input("Enter another goal point x coordinate:"))
        
            goaly=float(input("Enter another goal point y coordinate:"))
        
            goal_ang=0
            final_obs=self.check_obs(goalx,goaly)
            final_bound=self.check_bound(goalx,goaly)
        goal=self.norm_node([goalx,goaly,goal_ang])
        final_node=[goal[0],goal[1],goal[2]]
        
        array_for_cost=np.array(np.ones((self.sizeofx,self.sizeofx,self.nare)) * np.inf)

        visited=np.array(np.zeros((self.sizeofx,self.sizeofx,self.nare)))

        array_total_cost=np.array(np.ones((self.sizeofx,self.sizeofx,self.nare)) * np.inf)

        parent={}
        Q=[]
        heappush(Q,(0,start))
        array_for_cost[int(self.ndist*start[0])][int(self.ndist*start[1])][start[2]]=0
        array_total_cost[int(self.ndist*start[0])][int(self.ndist*start[1])][start[2]]=0
        nodes_explored=[]
        start_time=time.time()    
        flag_break=0
        s=[]
        n=[]
        ps=[]
        pn=[]

        while self.goal_reached(norm_current_node,final_node):
            if flag_break==1:
                break
            norm_current_node=heappop(Q)[1]
            if self.goal_reached(norm_current_node,final_node)==0:
                goalfound=[norm_current_node,action]
                print("found")
                print(norm_current_node)
                break
            for action in actions:
                curr_node=self.exec_robot_motion(norm_current_node[0], norm_current_node[1], norm_current_node[2], action[0], action[1],s,n)
                if(curr_node==None):
                    continue
                length=curr_node[3]
                curr_node=curr_node[0:3]
                norm_curr_node=self.norm_node(curr_node)
                if self.goal_reached(norm_curr_node,final_node)==0:
                    print("found")
                    parent[self.string_from_list(norm_curr_node)]=[norm_current_node,action]
                    goalfound=[norm_curr_node,action]
                    print(norm_curr_node)
                    flag_break=1
                    break
                status=self.check_bound(norm_curr_node[0],norm_curr_node[1])
                flag=self.check_obs(norm_curr_node[0],norm_curr_node[1])
                if (status and flag == 1):
                    if visited[int(self.ndist*norm_curr_node[0]),int(self.ndist*norm_curr_node[1]),norm_curr_node[2]]==0:
                        visited[int(self.ndist*norm_curr_node[0]),int(self.ndist*norm_curr_node[1]),norm_curr_node[2]]=1
                        nodes_explored.append([norm_current_node,action,norm_curr_node])
                        parent[self.string_from_list(norm_curr_node)]=[norm_current_node,action]
                        array_for_cost[int(self.ndist*norm_curr_node[0]),int(self.ndist*norm_curr_node[1]),norm_curr_node[2]]=length+array_for_cost[int(self.ndist*norm_current_node[0]),int(self.ndist*norm_current_node[1]),norm_current_node[2]]
                        array_total_cost[int(self.ndist*norm_curr_node[0]),int(self.ndist*norm_curr_node[1]),norm_curr_node[2]]=array_for_cost[int(self.ndist*norm_curr_node[0]),int(self.ndist*norm_curr_node[1]),norm_curr_node[2]] + self.distance(norm_curr_node, final_node)
                        heappush(Q,( array_total_cost[int(self.ndist*norm_curr_node[0]),int(self.ndist*norm_curr_node[1]),norm_curr_node[2]] ,norm_curr_node ))
                    else:
                        if array_total_cost[int(self.ndist*norm_curr_node[0]),int(self.ndist*norm_curr_node[1]),norm_curr_node[2]]>(array_total_cost[int(self.ndist*norm_current_node[0]),int(self.ndist*norm_current_node[1]),norm_current_node[2]]+sum(action)):
                            array_total_cost[int(self.ndist*norm_curr_node[0]),int(self.ndist*norm_curr_node[1]),norm_curr_node[2]]=(array_total_cost[int(self.ndist*norm_current_node[0]),int(self.ndist*norm_current_node[1]),norm_current_node[2]]+sum(action))
                            nodes_explored.append([norm_current_node,action,norm_curr_node])
                            parent[self.string_from_list(norm_curr_node)]=[norm_current_node,action]

        optimal_path=[]
        def generate_path(goal,start):
            optimal_path.append(goal)
            GN=parent[self.string_from_list(goal[0])]
            optimal_path.append(GN)
            while (GN[0]!=start):
                GN=parent[self.string_from_list(GN[0])]
                optimal_path.append(GN)
            return optimal_path


        optimal_path=generate_path(goalfound,start)
        optimal_path.reverse()
        print("Time taken")
        print(time.time()-start_time)  

        sx=startx
        sy=starty
        sz=starting_ang
        fig, ax = plt.subplots()
        ax.set(xlim=(0, 6), ylim=(0, 2))

        plt.plot(norm_curr_node[0], norm_curr_node[1], color='green', marker='o', linestyle='dashed', linewidth=1,markersize=4)
        plt.plot(start[0], start[1], color='yellow', marker='o', linestyle='dashed', linewidth=1,markersize=4)
        
        c1 = plt.Circle((4.2, 1.2), 0.6, fill=None)
        currentAxis = plt.gca()
        currentAxis.add_patch(Rectangle((1.5, 1), 0.25, 1, fill=None, alpha=1))
        currentAxis.add_patch(Rectangle((2.5, 0), 0.25, 1, fill=None, alpha=1))
        # currentAxis.add_patch(Rectangle((2.25, 7.25), 1.5, 1.5, fill=None, alpha=1))
        currentAxis.add_patch(Rectangle((0, 0), 6, 2, fill=None, alpha=1))
        
        ax.add_artist(c1)
        ax.set_aspect('equal')
        plt.grid()
        for action in optimal_path:
            x1= self.draw_motion(sx,sy,sz, action[1][0],action[1][1])
            sx=x1[0]
            sy=x1[1]
            sz=x1[2]
        plt.show()
        plt.pause(1)
        plt.close()
        ros=[]
        for i in range (len(optimal_path)):
            ros.append(optimal_path[i][1])
        print(ros)
        c =0
        completion=0
        for action in ros:
            print("action", action)
            print("completion: ", int((completion/len(ros))*100), "%")
            completion+=1
            while rclpy.ok():
                if c== 101:
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
                    self.vel_pub.publish(self.msg)
                    break
                else:
                    vel , th, thet_f = self.cal_vel(0.5,1,0,action[0],action[1])
                    self.msg.linear.x = vel*10
                    self.msg.angular.z =  th*10
                    self.vel_pub.publish(self.msg)
                    c=c+1
                    time.sleep(0.1)
            c=0
        print("REACHED GOAL !")
        time.sleep(2)

        sys.exit()

def main(args=None):
    rclpy.init(args=args)
    vel_publisher = vel_publisher()

    vel_publisher.run()

    rclpy.spin(vel_publisher)
    vel_publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()