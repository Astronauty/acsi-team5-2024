import logging
import sys
import time
import osqp
import numpy as np
import scipy as sp

from threading import Event


from scipy import signal
from scipy import sparse


N_MPC_HORIZON = 3 # number of steps to consider in MPC horizon
N_STATES = 12
N_CONTROLS = 4


class QuadrotorLQR():
    """Class that solves the following quadratic program to stabilize the drone about a reference setpoint:
    
    min 0.5x'Px + q'x
    s.t. l <= Ax <= u
    """
    def __init__(self, dt, verbose=True):

        
        self.umax = 0.15 # max thrust per motor in newtons

        params = dict()
        params['g'] = 9.81
        params['m'] = 0.027
        
        g = params['g']
        m = params['m']
        
        Ixx = 2.3951E-5
        Iyy = 2.3951E-5
        Izz = 3.2347E-5

        # Construct the state space system for the quadrotor (based on https://arxiv.org/pdf/1908.07401)
        A = np.zeros([12,12])
        A[0:3,3:6] = np.eye(3)
        A[3:6,6:9] = np.array([[0, -g, 0], 
                               [g, 0, 0],
                               [0, 0, 0 ]])
        A[6:9,9:12] = np.eye(3)

        B = np.zeros([12,4])
        #B[4,0] = -1/m
        B[5,0] = -1/m
        B[9:12, 1:4] = np.array([[1/Ixx, 0, 0], 
                                 [0, 1/Iyy, 0],
                                 [0, 0, 1/Izz]])
        
        C = np.zeros([6,12])
        C[0:3,0:3] = np.eye(3)
        C[3:6, 6:9] = np.eye(3)
        
        D = np.zeros([6,4])

        self.sys = signal.StateSpace(A, B, C, D)
        self.discrete_sys = self.sys.to_discrete(dt)
        
            
        # Define LQR costs
        #Q = np.eye(N_STATES) #x,y,z,vx,vy,vz,roll,pitch,yaw,vroll,vpitch,vyaw
        #Q_1 = 1*np.array([4000,4000,1,1,1,0.1,0.0001,0.0001,0.0001,0.0001,0.0001,0.0001])##################
        #Q_1 = 1*np.array([4,4,1,1,1,1,10000000,10000000,10000000,100,100,100])
        #Q_1 = 1*np.array([4000,4000,1,1,1,1,1,1,1,1,1,1])######12/7
        Q_1 = 1*np.array([10000,10000,10000,500,500,500,1000,1000,1000,50,50,50])#32ms
        #Q_1 = 1*np.array([40000,40000,70000,1600,1600,1600,1500,1500,1500,50,50,50])#10ms
        Q = np.diag(Q_1)
        R = np.eye(N_CONTROLS)
        
        self.Qbar = np.kron(np.eye(N_MPC_HORIZON), Q)
        self.Rbar = 100*np.kron(np.eye(N_MPC_HORIZON-1), R)
        
        # Precomputes
        self.S_hat = self.compute_S_hat(self.discrete_sys)
        self.T_hat = self.compute_T_hat(self.discrete_sys)
        
        print("S_hat Shape:", self.S_hat.shape)
        print("T_hat Shape:", self.T_hat.shape)
        print("Qbar Shape:", self.Qbar.shape)
        print("Rbar Shape:", self.Rbar.shape)
        
        
        self.P = sparse.csc_matrix(2*self.Rbar + 2*self.S_hat.T @ self.Qbar @ self.S_hat) # Quadratic cost term
        print("P",self.P)
        # Set up the OSQP solver
        self.prob = osqp.OSQP()
        
        self.x0 = np.zeros((N_STATES, 1))
        self.q = (2*self.x0.T @ self.T_hat.T @ self.Qbar @ self.S_hat).T # Recompute linear cost term at each timestep
        
        self.A = sparse.vstack([sparse.eye(N_CONTROLS * (N_MPC_HORIZON-1)), -sparse.eye(N_CONTROLS * (N_MPC_HORIZON-1))], format='csc')
        self.l = -np.inf * np.ones((2*N_CONTROLS*(N_MPC_HORIZON-1), 1))
        
        self.u = np.inf * np.ones((2*N_CONTROLS*(N_MPC_HORIZON-1), 1))
        
        # self.u = np.vstack([np.vstack([np.ones((N_CONTROLS,1))] * (N_MPC_HORIZON-1)), -np.vstack([np.ones((N_CONTROLS,1))]* (N_MPC_HORIZON-1))])        

        
        print()
        print("P_shape: ", self.P.shape)
        print("q_shape: ", self.q.shape)
        print("A_shape: ", self.A.shape)
        print("l_shape: ", self.l.shape)
        print("u_shape: ", self.u.shape)
        
        print("P_type: ", type(self.P))
        print("q_type: ", type(self.q))
        print("A_type: ", type(self.A))
        print("l_type: ", type(self.l))
        print("u_type: ", type(self.u))
        
        # print("P: ", self.P)
        # print("q: ", self.q)
        # print("A: ", self.A)
        # print("l: ", self.l)
        # print("u: ", self.u)
        
        
    
        # prob.setup(self.P, self.q, self.A, self.l, self.u, warm_starting=True, verbose=False)
        self.prob.setup(self.P, self.q, self.A, self.l, self.u, verbose=True)
        
        
        # Enable motor power override via param setting
        #self.scf.cf.param.set_value('motorPowerSet.enable', 1)
        #self.set_motor_thrusts(np.zeros(4))

        #print(self.thrust_to_pwm(0.01))
        #print(self.pwm_to_thrust(np.iinfo(np.uint16).max))

        # self.set_motor_thrusts(np.array([0.01, 0.01, 0.01, 0.01]))

        pass
    
    
    
    def update_x0(self,x,y,z,vx,vy,vz,roll,pitch,yaw,vroll,vpitch,vyaw):
        self.x0[0:6] = np.array([x, y, z, vx, vy, vz]).reshape(6,1)
        self.x0[6:12] = np.array([roll, pitch, yaw, vroll, vpitch, vyaw]).reshape(6,1)
        #self.x0[np.abs(self.x0) < 0.001] = 0
        #print(self.x0)
        return None

    def solve_linear_mpc(self, x_ref):
        e = x_ref - self.x0
        #e[2] = -self.x0[2] + x_ref[2]
        #e[5] = -self.x0[5] + x_ref[5]
        #e = np.zeros((12,1))
        #e[0:6] = -x_ref[0:6] + self.x0[0:6]
        #e[6:12] = self.x0[6:12] - x_ref[6:12]
        #e[np.abs(e) < 0.0000001] = 0
        print("e",e)
        self.q = (2*e.T @ self.T_hat.T @ self.Qbar @ self.S_hat).T # Recompute linear cost term at each timestep
        self.prob.update(q=self.q)
        res = self.prob.solve()
        
        if res.info.status != 'solved':
            raise ValueError("OSQP did not solve the problem.")
            
        u = res.x
        #self.set_motor_thrusts(0.1*u)
        print("u: ", u)
        
        return res
        
    def compute_S_hat(self, discrete_state_space_sys):
        """
        Computes the matrix mapping control inputs
        
        Args:
            discrete_state_space_sys (StateSpace): The discrete time state space system used to rollout the future dynamics.

        Returns:
            S_hat (ndarray): test
        """
        A = discrete_state_space_sys.A
        B = discrete_state_space_sys.B
        C = discrete_state_space_sys.C
        D = discrete_state_space_sys.D
        
        S_hat = np.zeros([N_STATES*N_MPC_HORIZON, N_CONTROLS*(N_MPC_HORIZON-1)])
        for k in range(N_MPC_HORIZON): # state prediction timestep
            for j in range(N_MPC_HORIZON-1): # control timestep
                if(k-j >= 0):
                    # print(k," ", j)
                    # print(B.shape)
                    S_hat[k*N_STATES:(k+1)*N_STATES, j*N_CONTROLS:(j+1)*N_CONTROLS] = np.linalg.matrix_power(A, k-j) @ B
                    # print()
                    # print((np.linalg.matrix_power(A, k-j) @ B).shape)
                    # print(S_hat[k*N_STATES:(k+1)*N_STATES, j*N_CONTROLS:(j+1)*N_CONTROLS].shape)
                
        return S_hat
    
    def compute_T_hat(self, discrete_state_space_sys):
        """_summary_

        Args:
            discrete_state_space_sys (StateSpace): The discrete time state space system used to rollout the future dynamics.

        Returns:
            T_hat (N_STATES * N_MPC_HORIZON, N_STATES): T_hat, maps the 
        """
        A = discrete_state_space_sys.A
        B = discrete_state_space_sys.B
        C = discrete_state_space_sys.C
        D = discrete_state_space_sys.D
        
        T_hat = np.zeros([N_STATES*N_MPC_HORIZON, N_STATES])
        
        # T_hat = [A, A^2, ..., A^N]';
        for k in range(N_MPC_HORIZON):
            T_hat[k*N_STATES:(k+1)*N_STATES, :] = np.linalg.matrix_power(A, k+1)
            
        return T_hat
    
    def quadratic_cost_matrix(self, S_hat, T_hat, Q_bar, S_bar):
        """
        The quadratic (P) and linear cost (q) matrices for a quadratic program with cost function of the form J = 0.5x'Px + q'x.
        Returns:
            P (ndarray):
            q 
        """
        # self.compute_
        return NotImplementedError

        
    
    def state_prediction(self, S_hat, T_hat, z, x0):
        """_summary_

        Returns:
            _type_: X = S_hat*z + T_hat*x0
        """
        X = S_hat @ z + T_hat @ x0
        return X





def quadrotor_lqr_cost(x, u, Q, R):
    return x @ Q @ x + u @ R @ u