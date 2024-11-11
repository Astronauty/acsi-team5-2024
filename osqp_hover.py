import logging
import sys
import time
import osqp
import numpy as np
import scipy as sp
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from scipy import signal
# from scipy import linalg

URI = dict()
URI['cf'] = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E701')
URI['tb'] = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E702')

N_MPC_HORIZON = 5 # number of steps to consider in MPC horizon
N_STATES = 12
N_CONTROLS = 4


class QuadrotorLQR():
    """Class that solves a quadratic program to stabilize the drone about a setpoint. 
    """
    def __init__(self, dt, verbose=True):
        params = dict()
        params['g'] = 9.81
        params['m'] = 0.027
        
        g = params['g']
        m = params['m']
        
        Ixx = 2.3951E-5
        Iyy = 2.3951E-5
        Izz = 3.2347E-5

        # Construct the state space system for the quadrotor (based on https://arxiv.org/pdf/1908.07401)
        print("Continuous State Space Model")
        print("============================")
        
        A = np.zeros([12,12])
        A[0:3,3:6] = np.eye(3)
        A[3:6,6:9] = np.array([[0, -g, 0], 
                               [g, 0, 0],
                               [0, 0, 0 ]])
        A[6:9,9:12] = np.eye(3)
        print("A: \n", A)

        B = np.zeros([12,4])
        B[4,0] = -1/m
        B[9:12, 1:4] = np.array([[1/Ixx, 0, 0], 
                                 [0, 1/Iyy, 0],
                                 [0, 0, 1/Izz]])
        print("B: \n", B)
        
        C = np.zeros([6,12])
        C[0:3,0:3] = np.eye(3)
        C[3:6, 6:9] = np.eye(3)
        print("C: \n", C)
        
        D = np.zeros([6,4])

        self.sys = signal.StateSpace(A, B, C, D)
        self.discrete_sys = self.sys.to_discrete(dt)
        
        # Precomputes
        self.S_hat = self.compute_S_hat(self.discrete_sys)
        self.T_hat = self.compute_T_hat(self.discrete_sys)
        
        print("\nT Hat")
        print("============================")
        print(self.T_hat)
    
        print("\nS Hat")
        print("============================")
        print(self.S_hat)

        print(self.discrete_sys.A)
        pass
    

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

def set_motor_power(u):
    return NotImplemented



# Callback to log position and orientation
def state_est_log_callback(timestamp, data, logconf):
    x = data['stateEstimate.x']
    y = data['stateEstimate.y']
    z = data['stateEstimate.z']
    yaw = data['stateEstimate.yaw']
    pitch = data['stateEstimate.pitch']
    roll = data['stateEstimate.roll']
    print(f"[{logconf.name}][{timestamp}] Pose: x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={yaw:.2f}, pitch={pitch:.2f}, roll={roll:.2f}")
    

def main():
    # cflib.crtp.init_drivers()

    # # Define a log configuration to get position and orientation data
    # log_conf = LogConfig(name='cf', period_in_ms=100)
    # log_conf.add_variable('stateEstimate.x', 'float')
    # log_conf.add_variable('stateEstimate.y', 'float')
    # log_conf.add_variable('stateEstimate.z', 'float')

    # log_conf.add_variable('stateEstimate.yaw', 'float')
    # log_conf.add_variable('stateEstimate.pitch', 'float')
    # log_conf.add_variable('stateEstimate.roll', 'float')

    # with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    #     print("Connected to Crazyflie.")
        
    #     # Add the log configuration to the Crazyflie
    #     scf.cf.log.add_config(log_conf)

        # Enable motor power override via param setting
        # scf.cf.param.set_value('motorPowerSet.enable', 1)
        # set_motor_power(np.zeros(4))


    #     # Start logging if the configuration is added successfully
    #     if log_conf.valid:
    #         log_conf.data_received_cb.add_callback(state_est_log_callback)
    #         log_conf.start()
    #         print("Logging position and orientation data...")

    #         # Keep the connection open and outputting data
    #         try:
    #             while True:
    #                 time.sleep(1)
    #         except KeyboardInterrupt:
    #             print("Logging stopped.")

    #         log_conf.stop()
    #     else:
            # print("Invalid logging configuration.")

    # Define LQR parameters
    paa
    quadrotor_lqr = QuadrotorLQR(dt=0.001)
    
            

if __name__ == '__main__':
    main()