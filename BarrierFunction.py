# This script is used to visualize the barrier functions


from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
from numpy import sqrt
from numpy import exp


b = 100.0 #argument of exponential
c = 2.0 #cost at obstacle diameter

def B(x, y, obst_tup):
    xc, yc, r_obst = obst_tup
    B = r_obst**8*(c + r_obst**4*exp(-b*r_obst**2))/((x - xc)**2 + (y - yc)**2)**4 - ((x - xc)**2 + (y - yc)**2)**2*exp(-100*(x - xc)**2 - 100*(y - yc)**2)
    return B

fig = plt.figure()
ax = plt.axes(projection='3d')

x = np.linspace(0,10,100, float)
y = np.linspace(0,10,100, float)
XX,YY = np.meshgrid(x,y)
COST_FIELD = np.zeros_like(XX)


obst_field = ((5.0, 3.0, 1.0),(3.0, 4.0, 0.5),(6.0, 8.0, 1.0))   # xc, yc, r


for obst in obst_field:
    xc, yc, r = obst
    # traces out the obstacle
    theta = np.linspace(0,2*np.pi, 100)
    circle = xc + 1j*yc + np.exp(-1j*theta)
    obst_zone = xc + 1j*yc + 2*np.exp(-1j*theta)
    COST_FIELD += B(XX, YY, obst)

    # plt.figure(1)
    # ax.plot3D(np.real(circle), np.imag(circle), H_obst, color="red")
    # ax.plot3D(np.real(obst_zone), np.imag(obst_zone), 0, color="red")

    plt.figure(2)
    plt.plot(np.real(circle), np.imag(circle))
    plt.plot(np.real(obst_zone), np.imag(obst_zone), '--')

# set COST_FIELD[i,j]=0 where cost>c
for i in range(len(x)):
    for j in range(len(y)):
        if COST_FIELD[i,j] > c:
            COST_FIELD[i,j] = 0

plt.figure(1)
ax.contour3D(XX, YY, COST_FIELD, 100)
plt.show()
