import numpy as np
from scipy.signal import cont2discrete
from typing import List, Tuple
import quad_sim
from barriers import grad_obst_cost, hess_obst_cost

# This script was orgininally written by the MEAM5170 Teaching Assistants. Our group filled in much of the code for homework and added the barrier functions as part of the final project.

class iLQR(object):

    def __init__(self, x_goal: np.ndarray, N: int, dt: float, Q: np.ndarray, R: np.ndarray, Qf: np.ndarray, obst_field):
        """
        Constructor for the iLQR solver
        :param N: iLQR horizon
        :param dt: timestep
        :param Q: weights for running cost on state
        :param R: weights for running cost on input
        :param Qf: weights for terminal cost on input
        """

        # Quadrotor dynamics parameters
        self.m = 1
        self.a = 0.25
        self.I = 0.0625
        self.nx = 6
        self.nu = 2

        # iLQR constants
        self.N = N
        self.dt = dt

        # Solver parameters
        self.alpha = 2e-2      
        self.max_iter = 2500
        self.tol = 25

        # target state
        self.x_goal = x_goal
        self.u_goal = 0.5 * 9.81 * np.ones((2,))

        # Cost terms
        self.Q = Q
        self.R = R
        self.Qf = Qf

        # Obstacle field
        self.obst_field = obst_field

    def total_cost(self, xx, uu):
        J = sum([self.running_cost(xx[k], uu[k]) for k in range(self.N - 1)])
        return J + self.terminal_cost(xx[-1])

    def get_linearized_dynamics(self, x: np.ndarray, u: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        :param x: quadrotor state
        :param u: input
        :return: A and B, the linearized continuous quadrotor dynamics about some state x
        """
        m = self.m
        a = self.a
        I = self.I
        A = np.array([[0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1],
                      [0, 0, -np.cos(x[2]) * (u[0] + u[1]) / m, 0, 0, 0],
                      [0, 0, -np.sin(x[2]) * (u[0] + u[1]) / m, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0]])
        B = np.array([[0, 0],
                      [0, 0],
                      [0, 0],
                      [-np.sin(x[2]) / m, -np.sin(x[2]) / m],
                      [np.cos(x[2]) / m, np.cos(x[2]) / m],
                      [a / I, -a / I]])

        return A, B

    def get_linearized_discrete_dynamics(self, x: np.ndarray, u: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        :param x: state
        :param u: input
        :return: the discrete linearized dynamics matrices, A, B as a tuple
        """
        A, B = self.get_linearized_dynamics(x, u)
        C = np.eye(A.shape[0])
        D = np.zeros((A.shape[0],))
        [Ad, Bd, _, _, _] = cont2discrete((A, B, C, D), self.dt)
        return Ad, Bd

    def running_cost(self, xk: np.ndarray, uk: np.ndarray) -> float:
        """
        :param xk: state
        :param uk: input
        :return: l(xk, uk), the running cost incurred by xk, uk
        """

        # Standard LQR cost on the goal state
        lqr_cost = 0.5 * ((xk - self.x_goal).T @ self.Q @ (xk - self.x_goal) +
                          (uk - self.u_goal).T @ self.R @ (uk - self.u_goal))

        return lqr_cost

    def grad_running_cost(self, xk: np.ndarray, uk: np.ndarray) -> np.ndarray:
        """
        :param xk: state
        :param uk: input
        :return: [???l/???x???, ???l/???u???]???, evaluated at xk, uk
        """
        grad = np.zeros((8,))

        #TODO: Compute the gradient
        grad = np.hstack((self.Q @ (xk - self.x_goal), self.R @ (uk - self.u_goal))).T
        gradBr = grad_obst_cost(xk, uk, self.obst_field).T   # Obstacle field goes here

        grad += gradBr
        return grad.reshape((8,))

    def hess_running_cost(self, xk: np.ndarray, uk: np.ndarray) -> np.ndarray:
        """
        :param xk: state
        :param uk: input
        :return: The hessian of the running cost
        [[?????l/???x??, ?????l/???x???u],
         [?????l/???u???x, ?????l/???u??]], evaluated at xk, uk
        """
        H = np.zeros((self.nx + self.nu, self.nx + self.nu))

        # TODO: Compute the hessian
        H[0:6,0:6] = self.Q
        H[6:8,6:8] = self.R
        hessBr = hess_obst_cost(xk, uk, self.obst_field)   # obstacle field
        
        return H + hessBr

    def terminal_cost(self, xf: np.ndarray) -> float:
        """
        :param xf: state
        :return: Lf(xf), the running cost incurred by xf
        """
        return 0.5*(xf - self.x_goal).T @ self.Qf @ (xf - self.x_goal)

    def grad_terminal_cost(self, xf: np.ndarray) -> np.ndarray:
        """
        :param xf: final state
        :return: ???Lf/???xf
        """

        grad = np.zeros((self.nx))

        # TODO: Compute the gradient
        grad = self.Qf @ (xf - self.x_goal)
        
        return grad
        
    def hess_terminal_cost(self, xf: np.ndarray) -> np.ndarray:
        """
        :param xf: final state
        :return: ?????Lf/???xf??
        """ 

        H = np.zeros((self.nx, self.nx))

        # TODO: Compute H
        H = self.Qf

        return H

    def forward_pass(self, xx: List[np.ndarray], uu: List[np.ndarray], dd: List[np.ndarray], KK: List[np.ndarray]) -> \
            Tuple[List[np.ndarray], List[np.ndarray]]:
        """
        :param xx: list of states, should be length N
        :param uu: list of inputs, should be length N-1
        :param dd: list of "feed-forward" components of iLQR update, should be length N-1
        :param KK: list of "Feedback" LQR gain components of iLQR update, should be length N-1
        :return: A tuple (xtraj, utraj) containing the updated state and input
                 trajectories after applying the iLQR forward pass
        """

        xtraj = [np.zeros((self.nx,))] * self.N
        utraj = [np.zeros((self.nu,))] * (self.N - 1)
        xtraj[0] = xx[0]

        # TODO: compute forward pass
        for i in range(self.N-1):
            utraj[i] = uu[i] + KK[i] @ (xtraj[i] - xx[i]) + self.alpha * dd[i]
            xtraj[i+1] = quad_sim.F(xtraj[i], utraj[i], self.dt)

        return xtraj, utraj

    def backward_pass(self,  xx: List[np.ndarray], uu: List[np.ndarray]) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """
        :param xx: state trajectory guess, should be length N
        :param uu: input trajectory guess, should be length N-1
        :return: KK and dd, the feedback and feedforward components of the iLQR update
        """
        dd = [np.zeros((self.nu,))] * (self.N - 1)
        KK = [np.zeros((self.nu, self.nx))] * (self.N - 1)
        
        # TODO: compute backward pass
        Hk1 = self.hess_terminal_cost(xx[-1])
        gk1 = self.grad_terminal_cost(xx[-1])

        for i in reversed(range(self.N-1)):
            grad = self.grad_running_cost(xx[i], uu[i])

            lx = grad[0:6]
            lu = grad[6:8]
            lxx = self.Q
            luu = self.R
            Bruu = np.zeros((2,2))
            lxu = np.zeros((6,2))
            Brxu = np.zeros((6,2))
            lux = np.zeros((2,6))
            Brux = np.zeros((2,6))
            Ad, Bd = self.get_linearized_discrete_dynamics(xx[i],uu[i])
            
            Qx = lx + Ad.T @ gk1
            Qu = lu + Bd.T @ gk1
            Qxx = lxx + Ad.T @ Hk1 @ Ad
            Quu = luu + Bd.T @ Hk1 @ Bd
            Qux = lux + Bd.T @ Hk1 @ Ad
            Qxu = Qux.T

            dd[i] = -np.linalg.inv(Quu) @ Qu
            KK[i] = -np.linalg.inv(Quu) @ Qux

            Hk1 = Qxx - KK[i].T @ Quu @ KK[i] 
            gk1 = Qx - KK[i].T @ Quu @ dd[i]

        return dd, KK

    def calculate_optimal_trajectory(self, x: List[np.ndarray], uu_guess: List[np.ndarray]) -> \
            Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray]]:

        """
        Calculate the optimal trajectory using iLQR from a given initial condition x,
        with an initial input sequence guess uu
        :param x: initial state
        :param uu_guess: initial guess at input trajectory
        :return: xx, uu, KK, the input and state trajectory and associated sequence of LQR gains
        """
        #assert (len(uu_guess) == self.N - 1)

        # Get an initial, dynamically consistent guess for xx by simulating the quadrotor
        xx = x

        Jprev = np.inf
        Jnext = self.total_cost(xx, uu_guess)
        uu = uu_guess
        KK = None

        i = 0
        print('  ' + f'cost: {Jnext}')
        while np.abs(Jprev - Jnext) > self.tol and i < self.max_iter:
            dd, KK = self.backward_pass(xx, uu);
            xx, uu = self.forward_pass(xx, uu, dd, KK)

            Jprev = Jnext
            Jnext = self.total_cost(xx, uu)
            print('  ' + f'cost: {Jnext}')
            i += 1
        print('  ' + f'Converged to cost {Jnext}')
        return xx, uu, KK, Jnext
