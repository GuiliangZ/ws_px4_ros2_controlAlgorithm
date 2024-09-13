import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import time

centre = [1.5, 1.5, 1.5]
a = 0.5  # Radius for x


# Define theta and phi angles
theta = np.linspace(0, 2 * np.pi, 100)
phi = np.linspace(0, np.pi, 100)

# Create the grid of angles
theta, phi = np.meshgrid(theta, phi)

# Sphere equations
xe = centre[0] + a * np.sin(phi) * np.cos(theta)
ye = centre[1] + a * np.sin(phi) * np.sin(theta)
ze = centre[2] + a * np.cos(phi)

class Scvx():
    def __init__(self,destination) -> None:
        # Parameters
        self.T = 5  # Prediction horizon
        self.dt = 0.1  # Time step
        self.nx = 3  # Number of state variables (adjust as per your system)
        self.nu = 3  # Number of control inputs (adjust as per your system)
        self.Q = np.diag([1,1,1])#0*np.eye(nx)  # State weighting
        self.R = 0.1*np.eye(self.nu)  # Control weighting
        self.max_velocity = 0.5

        self.n=self.nx
        self.m=self.nu
        #T=len(states)
        self.N = self.m*(self.T - 1) + self.n*self.T
        self.Ncon = self.n*(self.T-1) + self.T+self.T-1

        self.A=np.eye(3)
        self.B=self.dt*np.eye(3)

        self.centre = [1.5, 1.5, 1.5]
        self.a = 0.5  # Radius for x


        # Define theta and phi angles
        theta = np.linspace(0, 2 * np.pi, 100)
        phi = np.linspace(0, np.pi, 100)

        # Create the grid of angles
        theta, phi = np.meshgrid(theta, phi)

        # Sphere equations
        self.xe = self.centre[0] + self.a * np.sin(phi) * np.cos(theta)
        self.ye = self.centre[1] + self.a * np.sin(phi) * np.sin(theta)
        self.ze = self.centre[2] + self.a * np.cos(phi)

        self.destination = destination  # Starting at the origin

    def z_bar(self, z, j):
        y = cp.Variable(shape=z.shape)
        x = y[:self.n*self.T]
        u = y[self.n*self.T:]
        

        objective = cp.Minimize(cp.norm(y - z, 2))
        
        if j < self.n * (self.T - 1):
            t = int(j / self.n)
            x_t = x[self.n*t:self.n*(t + 1)]
            x_tp1 = x[self.n*(t + 1):self.n*(t + 2)]
            u_t = u[self.m*t:self.m*(t + 1)]
            gv = self.A @ x_t + self.B @ u_t  - x_tp1 #<---------------------Removed +x_t from here, wasnot making sense
            const = [gv[np.mod(j,self.n)] <= 0]
            
        elif j>=self.n*(self.T-1) and j < self.n*(self.T-1) + self.T:
            #print('hihihi')
            t = j - self.n*(self.T - 1)
            x = y[:self.n*self.T]
            x_t = x[self.n*t:self.n*(t + 1)]
            const = [(x_t[0] - self.centre[0]) ** 2 + (x_t[1] - self.centre[1]) ** 2 + (x_t[2] - self.centre[2]) ** 2 - self.a**2 <= 0]

        else:
            t = j - self.n*(self.T - 1) - self.T
            u = y[self.n*self.T:]
            u_t = u[self.n*t:self.n*(t + 1)]
            const = [self.max_velocity**2 - (u_t[0] ) ** 2 - (u_t[1]) ** 2 - (u_t[2]) ** 2  >= 0]
        
        prb = cp.Problem(objective, const)
        prb.solve()
        #print(prb.value)
        
        return y.value


    def gfun(self, y, A, B, T, n, m):
        x = y[:n*T]
        u = y[n*T:]

        out=0
        for t in range(T-1):
            x_t = x[n*t:n*(t + 1)]
            x_tp1 = x[n*(t + 1):n*(t + 2)]
            u_t = u[m*t:m*(t + 1)]
            gi = A @ x_t + B @ u_t  - x_tp1#<---------------------Removed +x_t from here, wasnot making sense, because my A is already discrete
            out+=cp.norm(gi,1)
        return out

    def grad_g_t(self, y, t):
        out = np.zeros((self.n,self.N))
        out[:, self.n*t:self.n*(t + 1)] = self.A 
        out[:, self.n*(t + 1):self.n*(t + 2)] = -np.eye(self.n)
        out[:, self.n*self.T + self.m*t: self.n*self.T + self.m*(t + 1)] = self.B
        return out

    def grad_h1_t(self,y, t):
        out = np.zeros(self.N)
        x = y[:self.n*self.T]
        x_t = x[self.n*t:self.n*t+3]
        out[self.n*t:self.n*t+3] = 2*(x_t - self.centre)
        return out

    def grad_h2_t(self, y, t):
        out = np.zeros(self.N)
        u = y[self.n*self.T:]
        u_t = u[self.n*t:self.n*t+3]
        out[self.n*self.T + self.m*t: self.n*self.T + self.m*(t + 1)] = -2*u_t
        return out

    def grad_q_j(self, y, j):
        if j < self.n*(self.T - 1):
            t = int(j/self.n) 
            out = self.grad_g_t(y, t)
            out = out[np.mod(j,self.n), :]
        elif j>=self.n*(self.T-1) and j < self.n*(self.T-1) + self.T:
            t = j - self.n*(self.T - 1)
            out = self.grad_h1_t(y, t)
        else:
            t = j - self.n*(self.T - 1) - self.T
            out = self.grad_h2_t(y,t)
        return out


    

    def mpc_controller(self, current_state):
        # MPC controller to compute optimal control input
        optimal_control = -current_state / np.linalg.norm(current_state)
        
        states = np.zeros((self.T, 3))  # Adjusted for 3 dimensions
        states[0] = current_state
        control = np.zeros_like(optimal_control)
        control[2] = -0.5  # Specific control behavior
        for k in range(1, self.T):
            states[k] = states[k-1] + control * self.dt
        flattened_states = states.flatten()
        repeated_controls = np.tile(optimal_control, (self.T-1, 1)).flatten()
        z = np.concatenate([flattened_states, repeated_controls])

        y2 = cp.Variable(self.N)
        lbda = 1
        gn = self.gfun(y2, self.A, self.B, self.T, self.nx, self.nu)
        Objective2 = cp.Minimize(cp.norm(y2[self.nx*self.T-3:self.nx*self.T], 2) + lbda * gn + 0.05 * cp.norm(y2[self.nx*self.T:], 2))
        constraint2 = [y2[0] == z[0], y2[1] == z[1], y2[2] == z[2]]

        for j in range(self.Ncon):
            z_bar_j = self.z_bar(z, j)
            dq_j = self.grad_q_j(z_bar_j, j)
            constraint2.append(dq_j @ (y2 - z_bar_j) >= 0)

        problem2 = cp.Problem(Objective2, constraint2)
        problem2.solve()

        z_new = y2.value
        optimal_control = z_new[self.nx*self.T:self.nx*self.T+self.nu]
        
        return optimal_control
        