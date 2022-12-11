import numpy as np
from numpy import sqrt, exp

# Global variables
# a: coefficient used in development, but not present in code (function of obstacle radius)
b = 100   # argument of the exponential term
c = 50     # cost at obstacle diameter

# Cost-landscape made by the obstacle field
def B(x, y, obst_field):
    # 6x1 State z = [x, y, theta, x_dot, y_dot, theta_dot].T
    x = z[0]
    y = z[1]
    B = 0.0
    for obst in obst_field:
        xc, yc, r_obst = obst
        B += r_obst**8*(c + r_obst**4*exp(-b*r_obst**2))/((x - xc)**2 + (y - yc)**2)**4 - ((x - xc)**2 + (y - yc)**2)**2*exp(-b*((x - xc)**2 + (y - yc)**2))
    return B

def grad_obst_cost(z, uk, obst_field):
    '''
    param: zk = 6x1 state vector 
    param: uk = 2x1 control
    param: obst_field: list of obst tuples
    param: H: height (cost) of all obstacles
    return: 8x1 gradient vector (keep uk in here for full generality)
    '''
    grad = np.zeros((8,))
    grad[0] = B_x(z, obst_field)
    grad[1] = B_y(z, obst_field)
    
    return grad

def hess_obst_cost(z, uk, obst_field):
    '''
    param: zk = 6x1 state vector 
    param: uk = 2x1 control
    param: obst_field: list of obst tuples
    return: 8x8 (keep uk in here for full generality incase we need it)
    '''
    hessB = np.zeros((8,8))
    hessB[0,0] = B_xx(z, obst_field)
    hessB[1,1] = B_yy(z, obst_field)
    hessB[0,1] = hessB[1,0] = B_xy(z, obst_field)
    return hessB

# maybe pick another letter for the barrier function
def B_x(z, obst_field):
    # 6x1 State z = [x, y, theta, x_dot, y_dot, theta_dot].T
    x = z[0]
    y = z[1]
    B_x = 0.0
    for obst in obst_field:
        xc, yc, r_obst = obst
        B_x += b*(2*x - 2*xc)*((x - xc)**2 + (y - yc)**2)**2*exp(-b*((x - xc)**2 + (y - yc)**2)) + r_obst**8*(c + r_obst**4*exp(-b*r_obst**2))*(-8*x + 8*xc)/((x - xc)**2 + (y - yc)**2)**5 - (4*x - 4*xc)*((x - xc)**2 + (y - yc)**2)*exp(-b*((x - xc)**2 + (y - yc)**2))
    return B_x

def B_y(z, obst_field):
    # State z = [x, y, theta, x_dot, y_dot, theta_dot].T
    x = z[0]
    y = z[1]
    B_y = 0.0
    for obst in obst_field:
        xc, yc, r_obst = obst
        B_y += b*(2*y - 2*yc)*((x - xc)**2 + (y - yc)**2)**2*exp(-b*((x - xc)**2 + (y - yc)**2)) + r_obst**8*(c + r_obst**4*exp(-b*r_obst**2))*(-8*y + 8*yc)/((x - xc)**2 + (y - yc)**2)**5 - (4*y - 4*yc)*((x - xc)**2 + (y - yc)**2)*exp(-b*((x - xc)**2 + (y - yc)**2))
    return B_y
 
def B_xx(z, obst_field):
    # State z = [x, y, theta, x_dot, y_dot, theta_dot].T
    x = z[0]
    y = z[1]
    B_xx = 0.0
    for obst in obst_field:
        xc, yc, r_obst = obst
        B_xx += -b**2*(2*x - 2*xc)**2*((x - xc)**2 + (y - yc)**2)**2*exp(-b*((x - xc)**2 + (y - yc)**2)) - b*(2*x - 2*xc)*(4*x - 4*xc)*(-(x - xc)**2 - (y - yc)**2)*exp(-b*((x - xc)**2 + (y - yc)**2)) + b*(2*x - 2*xc)*(4*x - 4*xc)*((x - xc)**2 + (y - yc)**2)*exp(-b*((x - xc)**2 + (y - yc)**2)) + 2*b*((x - xc)**2 + (y - yc)**2)**2*exp(-b*((x - xc)**2 + (y - yc)**2)) + r_obst**8*(c + r_obst**4*exp(-b*r_obst**2))*(-10*x + 10*xc)*(-8*x + 8*xc)/((x - xc)**2 + (y - yc)**2)**6 - 8*r_obst**8*(c + r_obst**4*exp(-b*r_obst**2))/((x - xc)**2 + (y - yc)**2)**5 + (-2*x + 2*xc)*(4*x - 4*xc)*exp(-b*((x - xc)**2 + (y - yc)**2)) + (-4*(x - xc)**2 - 4*(y - yc)**2)*exp(-b*((x - xc)**2 + (y - yc)**2))
    return B_xx

def B_yy(z, obst_field):
    # State z = [x, y, theta, x_dot, y_dot, theta_dot].T
    x = z[0]
    y = z[1]
    B_yy = 0.0
    for obst in obst_field:
        xc, yc, r_obst = obst
        B_yy += -b**2*(2*y - 2*yc)**2*((x - xc)**2 + (y - yc)**2)**2*exp(-b*((x - xc)**2 + (y - yc)**2)) - b*(2*y - 2*yc)*(4*y - 4*yc)*(-(x - xc)**2 - (y - yc)**2)*exp(-b*((x - xc)**2 + (y - yc)**2)) + b*(2*y - 2*yc)*(4*y - 4*yc)*((x - xc)**2 + (y - yc)**2)*exp(-b*((x - xc)**2 + (y - yc)**2)) + 2*b*((x - xc)**2 + (y - yc)**2)**2*exp(-b*((x - xc)**2 + (y - yc)**2)) + r_obst**8*(c + r_obst**4*exp(-b*r_obst**2))*(-10*y + 10*yc)*(-8*y + 8*yc)/((x - xc)**2 + (y - yc)**2)**6 - 8*r_obst**8*(c + r_obst**4*exp(-b*r_obst**2))/((x - xc)**2 + (y - yc)**2)**5 + (-2*y + 2*yc)*(4*y - 4*yc)*exp(-b*((x - xc)**2 + (y - yc)**2)) + (-4*(x - xc)**2 - 4*(y - yc)**2)*exp(-b*((x - xc)**2 + (y - yc)**2))
    return B_yy

def B_xy(z, obst_field):
    # State z = [x, y, theta, x_dot, y_dot, theta_dot].T
    x = z[0]
    y = z[1]
    B_xy = 0.0
    for obst in obst_field:
        xc, yc, r_obst = obst
        B_xy += -b**2*(2*x - 2*xc)*(2*y - 2*yc)*((x - xc)**2 + (y - yc)**2)**2*exp(-b*((x - xc)**2 + (y - yc)**2)) + b*(2*x - 2*xc)*(4*y - 4*yc)*((x - xc)**2 + (y - yc)**2)*exp(-b*((x - xc)**2 + (y - yc)**2)) - b*(4*x - 4*xc)*(2*y - 2*yc)*(-(x - xc)**2 - (y - yc)**2)*exp(-b*((x - xc)**2 + (y - yc)**2)) + r_obst**8*(c + r_obst**4*exp(-b*r_obst**2))*(-8*x + 8*xc)*(-10*y + 10*yc)/((x - xc)**2 + (y - yc)**2)**6 + (4*x - 4*xc)*(-2*y + 2*yc)*exp(-b*((x - xc)**2 + (y - yc)**2))
    return B_yy
