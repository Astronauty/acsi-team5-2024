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
from scipy import sparse
# from scipy import linalg

URI = dict()
URI['cf'] = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E702')
#URI['tb'] = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E701')

N_MPC_HORIZON = 3 # number of steps to consider in MPC horizon
N_STATES = 12
N_CONTROLS = 4


class QuadrotorLQR():
    """Class that solves the following quadratic program to stabilize the drone about a reference setpoint:
    
    min 0.5x'Px + q'x
    s.t. l <= Ax <= u
    """
    def __init__(self, dt, scf, verbose=True):
        self.scf = scf
        self.pwm_to_thrust_a = float(self.scf.cf.param.get_value('quadSysId.pwmToThrustA'))
        self.pwm_to_thrust_b = float(self.scf.cf.param.get_value('quadSysId.pwmToThrustB'))
        
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
        B[4,0] = -1/m
        B[9:12, 1:4] = np.array([[1/Ixx, 0, 0], 
                                 [0, 1/Iyy, 0],
                                 [0, 0, 1/Izz]])
        
        C = np.zeros([6,12])
        C[0:3,0:3] = np.eye(3)
        C[3:6, 6:9] = np.eye(3)
        
        D = np.zeros([6,4])

        self.sys = signal.StateSpace(A, B, C, D)
        self.discrete_sys = self.sys.to_discrete(dt)
        
        if verbose:
            print("Continuous State Space Model")
            print("============================")
            print("A: \n", A)
            print("B: \n", B)
            print("C: \n", C)


            print("\nT Hat")
            print("============================")
            print(self.T_hat)
        
            print("\nS Hat")
            print("============================")
            print(self.S_hat)
            
        # Define LQR costs
        Q = np.eye(N_STATES)
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
        self.scf.cf.param.set_value('motorPowerSet.enable', 1)
        self.set_motor_thrusts(np.zeros(4))

        print(self.thrust_to_pwm(0.01))
        print(self.pwm_to_thrust(np.iinfo(np.uint16).max))

        # self.set_motor_thrusts(np.array([0.01, 0.01, 0.01, 0.01]))

        pass
    
    def translation_state_est_callback(self, timestamp, data, logconf):
        
        x = data['stateEstimate.x']
        y = data['stateEstimate.y']
        z = data['stateEstimate.z']
        vx = data['stateEstimate.vx']
        vy = data['stateEstimate.vy']
        vz = data['stateEstimate.vz']
        # roll = data['stateEstimate.roll']
        # pitch = data['stateEstimate.pitch']
        # yaw = data['stateEstimate.yaw']
        
        # print("x:", self.x0)
        # self.x0 = np.array([x, y, z, vx, vy, vz, yaw, pitch, roll, 0, 0, 0])
        self.x0[0:6] = np.array([x, y, z, vx, vy, vz]).reshape(6,1)
        
        return None
    
    def orientation_state_est_callback(self, timestamp, data, logconf):
        # roll = data['stateEstimate.roll']
        # pitch = data['stateEstimate.pitch']
        # yaw = data['stateEstimate.yaw']
        
        roll = data['stabilizer.roll']
        pitch = data['stabilizer.pitch']
        yaw = data['stabilizer.yaw']
        
        vroll = data['stateEstimateZ.rateRoll']
        vpitch = data['stateEstimateZ.ratePitch']
        vyaw = data['stateEstimateZ.rateYaw']
        
        # print("x:", self.x0)
        # self.x0 = np.array([x, y, z, vx, vy, vz, yaw, pitch, roll, 0, 0, 0])
        self.x0[6:12] = np.array([roll, pitch, yaw, vroll, vpitch, vyaw]).reshape(6,1)
        
        return None
    
    
    def solve_linear_mpc(self, x_ref):
        self.q = (2*self.x0.T @ self.T_hat.T @ self.Qbar @ self.S_hat).T # Recompute linear cost term at each timestep
        self.prob.update(q=self.q)
        res = self.prob.solve()
        
        if res.info.status != 'solved':
            raise ValueError("OSQP did not solve the problem.")
            
        u = res.x
        self.set_motor_thrusts(0.1*u)
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

    def pwm_to_thrust(self, pwm):
        """
        Receives a PWM value from 0-UINT16_MAX and converts it to the thrust force in newtons based on the parameters defined in the quadSysId parameter group of crazyflie.
        Args:
            pwm (uint16): PWM value commanded to crazyflie motor.
        Returns:
            thrust: Force in newtons produced by the rotor at the provided PWM value.
        """
        pwm = pwm/np.uint16(65535) # normalize pwm to val from 0-1
        thrust = self.pwm_to_thrust_a*pwm*pwm + self.pwm_to_thrust_b*pwm
        return thrust

    def thrust_to_pwm(self, thrust):
        """
        Receives a thrust value in newtons and converts it to the equivalent PWM command.
        Args:
            thrust: Force in newtons produced by the rotor at the provided PWM value.
        Returns:
            pwm (uint16): PWM value commanded to crazyflie motor.
        """
        
        pwm = (-self.pwm_to_thrust_b + np.sqrt(self.pwm_to_thrust_b**2 + 4*self.pwm_to_thrust_a*thrust))/(2*self.pwm_to_thrust_a)
        pwm = pwm*np.uint16(65535) # convert normalized pwm to value that ranges from 0-UINT16_MAX
        return pwm

    
    def set_motor_thrusts(self, u):
        """
        Args:
            u (ndarray): Control input to the quadrotor in the form of a 4x1 vector where each element is the thrust in newtons.
        Returns:
            None
        """
        # assert len(u) == N_CONTROLS, "Control input must be a 4x1 vector."

        for i in range(4):
            pwm_value = self.thrust_to_pwm(u[i])
            pwm_value = max(0, min(65535, pwm_value))
            self.scf.cf.param.set_value(f'motorPowerSet.m{i+1}', pwm_value)
        
        return None

    


def quadrotor_lqr_cost(x, u, Q, R):
    return x @ Q @ x + u @ R @ u




def main():
    cflib.crtp.init_drivers()

    # Define a log configuration to get position and orientation data
    t_log_conf = LogConfig(name='t_cf', period_in_ms=100)
    t_log_conf.add_variable('stateEstimate.x', 'float')
    t_log_conf.add_variable('stateEstimate.y', 'float')
    t_log_conf.add_variable('stateEstimate.z', 'float')

    t_log_conf.add_variable('stateEstimate.vx', 'float')
    t_log_conf.add_variable('stateEstimate.vy', 'float')
    t_log_conf.add_variable('stateEstimate.vz', 'float')

    o_log_conf = LogConfig(name='o_cf', period_in_ms=100)
    # o_log_conf.add_variable('stateEstimate.roll', 'float')
    # o_log_conf.add_variable('stateEstimate.pitch', 'float')
    # o_log_conf.add_variable('stateEstimate.yaw', 'float')
    o_log_conf.add_variable('stabilizer.roll', 'float')
    o_log_conf.add_variable('stabilizer.pitch', 'float')
    o_log_conf.add_variable('stabilizer.yaw', 'float')
    o_log_conf.add_variable('stateEstimateZ.rateRoll', 'int16_t')
    o_log_conf.add_variable('stateEstimateZ.ratePitch', 'int16_t')
    o_log_conf.add_variable('stateEstimateZ.rateYaw', 'int16_t')
    

    # try: 
    with SyncCrazyflie(URI['cf'], cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Connected to Crazyflie.")
        
        # Add the log configuration to the Crazyflie
        scf.cf.log.add_config(t_log_conf)
        scf.cf.log.add_config(o_log_conf)
        


        # Define LQR parameters
        quadrotor_lqr = QuadrotorLQR(dt=0.05, scf=scf, verbose=False)

        # Start logging if the configuration is added successfully
        if t_log_conf.valid and o_log_conf.valid:
            t_log_conf.data_received_cb.add_callback(quadrotor_lqr.translation_state_est_callback)
            o_log_conf.data_received_cb.add_callback(quadrotor_lqr.orientation_state_est_callback)
            
            t_log_conf.start()
            o_log_conf.start()
            print("Logging position and orientation data...")

            # Keep the connection open and outputting data
            try:
                while True:
                    time.sleep(0.1)
                    quadrotor_lqr.solve_linear_mpc(np.zeros((N_STATES,1)))
                    print()
                    print("x0:", quadrotor_lqr.x0)
            except KeyboardInterrupt:
                scf.cf.param.set_value('motorPowerSet.enable', 0)
                print("Logging stopped.")

            t_log_conf.stop()
            o_log_conf.stop()
        else:
            print("Invalid logging configuration.")
    # except Exception as e:
        # print(f"Could not connect to Crazyflie: {e}")

 
    
            

if __name__ == '__main__':
    main()