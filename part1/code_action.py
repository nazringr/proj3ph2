import matplotlib.pyplot as plt
import numpy as np
import math

def cost(Xi,Yi,Thetai,UL,UR):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180


# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes
    D=0


    x = [Xi]
    y = [Yi]
    
    while t<1:
        t = t + dt
        # Xs = Xn
        # Ys = Yn
        Delta_Xn = 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Delta_Yn = 0.5*r * (UL + UR) * math.sin(Thetan) * dt

        Xn += Delta_Xn # update the positions each iteration
        Yn += Delta_Yn
        x.append(Xn)
        y.append(Yn)

        Thetan += (r / L) * (UR - UL) * dt
        D=D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
      
    print("Thetan: ", Thetan, "for [", UL, UR, "]")
    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan, D, x, y
    


rpm1 = 5
rpm2 = 10
actions = [[0, rpm1], [rpm1, 0], [rpm1, rpm1], [rpm2, rpm2],[0, rpm2], [rpm2, 0],  [rpm1, rpm2], [rpm2, rpm1]]
   
x_coords = []
y_coords = []
for action in actions:
     k=cost(0,0,45, action[0],action[1])      # (0,0,45) hypothetical start configuration, this dosn't matter for calucating the edges'costs
     print("Distance: ", k[3])
     x_coords.append(k[0])
     y_coords.append(k[1])
  
k1 = cost(0, 0, 0, 0, rpm1)
plt.plot
k2 = cost(0, 0, 0, rpm1, 0)
k3 = cost(0, 0, 0, rpm1, rpm1)
k4 = cost(0, 0, 0, rpm2, rpm2)
k5 = cost(0, 0, 0, 0, rpm2)
k6 = cost(0, 0, 0, rpm2, 0)
k7 = cost(0, 0, 0, rpm1, rpm2)
k8 = cost(0, 0, 0, rpm2, rpm1)

plt.title("Path Taken")
plt.plot(k1[4], k1[5], 'd', label = '[0, rpm1]')
plt.plot(k2[4], k2[5], 'H', label = '[rpm1, 0]')
plt.plot(k3[4], k3[5], 'p', label = '[rpm1, rpm1]')
plt.plot(k4[4], k4[5], '*', label = '[rpm2, rpm2]')
plt.plot(k5[4], k5[5], 'o', label = '[0, rpm2]')
plt.plot(k6[4], k6[5], 'x', label = '[rpm2, 0]')
plt.plot(k7[4], k7[5], 'v', label = '[rpm1, rpm2]')
plt.plot(k8[4], k8[5], 's', label = '[rpm2, rpm1]')

plt.legend()


fig = plt.figure()
plt.plot(x_coords, y_coords, 'x')
plt.legend()
plt.title("Final Positions")
plt.show()
