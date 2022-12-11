import numpy as np
from math import sin, cos, pi
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Dynamics for the quadrotor
def _f(x, u):
  g = 9.81
  m = 1
  a = 0.25
  I = 0.0625

  theta = x[2]
  ydot = x[3]
  zdot = x[4]
  thetadot = x[5]
  u0 = u[0]
  u1 = u[1]

  xdot = np.array([ydot,
                   zdot,
                   thetadot,
                   -sin(theta) * (u0 + u1) / m,
                   -g + cos(theta) * (u0 + u1) / m,
                   a * (u0 - u1) / I])

  return xdot

def F(xc, uc, dt):
    '''
    INPUTS:
    Outputs for streamline + differential flatness:
    xc: 6x1 state at a givent time step
    uc: 2x1 control at a given time step
    OUTPUTS:
    x_k+1: 6X1 state at next time step
    '''
    # Simulate the open loop quadrotor for one step
    def f(_, x):
        return _f(x, uc)
    sol = solve_ivp(f, (0, dt), xc, first_step=dt)
    return sol.y[:, -1].ravel()

def simulate_traj(init, uc, dt):
    # uc: 2xN array control sequence during the trajectory
    z = np.zeros((np.shape(uc)[1], 6))
    z[0,:] = init   # initial state at t0
    for i in range(np.shape(uc)[1]-1):
        z[i+1,:] = F(z[i,:], uc[:,i], dt)
        
    return z