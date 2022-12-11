import numpy as np
from scipy.interpolate import CubicSpline

# This function uses differential flatness to determine an initial input control, u=(uL, uR), which corresponds
# to the initial streamline solution, (xs, ys)
def diff_flat(xs, ys, tf):
    '''
    INPUTS:
    xs: np.ndarray, Streamline solution
    ys: np.ndarray, Streamline solution
    tf: scalar, Final time
    '''
    
    t = np.linspace(0, tf, len(xs))
    g = 9.81 # m/s^2
    x_spline = CubicSpline(t, xs)
    y_spline = CubicSpline(t, ys)
    
    samples = 100
    t_interp = np.linspace(0, tf, samples)

    x_dot = x_spline.derivative(1)
    y_dot = y_spline.derivative(1)
    
    x_ddot = x_spline.derivative(2)
    y_ddot = y_spline.derivative(2)
    theta = -np.arctan2(x_ddot(t_interp),y_ddot(t_interp)+g)   # array of thetas, in terms of flat outputs, x and y
    theta_spline = CubicSpline(t_interp, theta)
    theta_dot = theta_spline.derivative(1)
    theta_ddot = theta_spline.derivative(2)   # Spline

    m = 1
    a = 0.25
    I = 0.0625
    uL = 0.5*((theta_ddot(t_interp)*I/a)-(x_ddot(t_interp)*m/np.sin(theta)))
    uR = -x_ddot(t_interp)*m/np.sin(theta) - uL
    u = np.vstack((uL, uR))

    z = np.vstack((x_spline(t_interp), y_spline(t_interp), theta, x_dot(t_interp), y_dot(t_interp), theta_dot(t_interp)))   # input state vector for all times
    return z, u