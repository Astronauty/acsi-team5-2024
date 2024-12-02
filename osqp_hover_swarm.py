import sys
import select
import os
import time
import osqp
import numpy as np
#import keyboard
from threading import Event, Lock, Thread
from scipy import signal, sparse
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Ensure local cflib is used
#current_dir = os.path.dirname(os.path.abspath(__file__))
#cflib_path = os.path.join(current_dir, "cflib", "cflib")  # Adjust this if your cflib is elsewhere
#sys.path.insert(0, cflib_path)

# Optional: Verify it worked by logger.infoing which cflib is being imported
#logger.info(f"Using cflib from:{cflib_path}")

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI_C = 'radio://0/20/2M/E7E7E7E702' # Flying crazyflie
URI_T = 'radio://0/20/2M/E7E7E7E701' # Stationary crazyflie (Tumbller)

# Set to True to use the stationary crazyflie as the reference
USE_T_CRAZYFLIE = False

uris = [URI_C]
if USE_T_CRAZYFLIE:
    uris.append(URI_T)

# Shared state variable and lock for the stationary crazyflie
t_crazyflie_state = {'x':0, 'y':0, 'z':0}
t_state_lock = Lock()
stop_event = Event()  # Global event to signal threads to stop

# MPC parameters
N_MPC_HORIZON = 3  # number of steps to consider in MPC horizon
N_STATES = 12
N_CONTROLS = 4

# Function to check for keyboard input
def check_keyboard_input():
    if select.select([sys.stdin], [], [], 0)[0]:  # Non-blocking check
        return sys.stdin.read(1)  # Read one character
    return None

def check_connection(uri):
    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            logger.info(f"Connected to {uri}. Verifying communication...")
            if scf.cf.param.get_value('stateEstimate.x') is not None:
                logger.info(f"Crazyflie {uri} communication verified!")
                return True
            else:
                logger.info(f"Crazyflie {uri} connected but not responding.")
                return False
    except Exception as e:
        logger.info(f"Failed to connect to {uri}: {e}")
        return False

def t_crazyflie_state_callback(timestamp, data, logconf):
    with t_state_lock:
        t_crazyflie_state['x'] = data['stateEstimate.x']
        t_crazyflie_state['y'] = data['stateEstimate.y']
        t_crazyflie_state['z'] = data['stateEstimate.z']

# Function for starting the stationary crazyflie logging
def run_t_crazyflie(scf):
    # Set up logging
    log_conf = LogConfig(name='stateEstimate', period_in_ms=100)
    log_conf.add_variable('stateEstimate.x', 'float')
    log_conf.add_variable('stateEstimate.y', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')
    log_conf.add_variable('stateEstimate.vx', 'float')
    log_conf.add_variable('stateEstimate.vy', 'float')
    log_conf.add_variable('stateEstimate.vz', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(t_crazyflie_state_callback)
    log_conf.start()
    logger.info(f"Started logging for {scf.cf.uri}.")

    # Keep logging running until the stop event is set
    try:
        while not stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info(f"Keyboard interrupt detected for {scf.cf.uri}.")
    finally:
        log_conf.stop()
        logger.info(f"Stopped logging for {scf.cf.uri}.")


class QuadrotorLQR():
    """Class that solves the following quadratic program to stabilize the drone about a reference setpoint:

    min 0.5x'Px + q'x
    s.t. l <= Ax <= u
    """

    def __init__(self, dt, scf, uri, Q=None, R=None, verbose=True):
        self.dt = dt
        self.scf = scf
        self.uri = uri
        self.pwm_to_thrust_a = float(self.scf.cf.param.get_value('quadSysId.pwmToThrustA'))
        self.pwm_to_thrust_b = float(self.scf.cf.param.get_value('quadSysId.pwmToThrustB'))

        self.umax = 0.15  # max thrust per motor in newtons

        params = dict()
        params['g'] = 9.81
        params['m'] = 0.027

        g = params['g']
        m = params['m']

        Ixx = 2.3951E-5
        Iyy = 2.3951E-5
        Izz = 3.2347E-5

        # Construct the state space system for the quadrotor (based on https://arxiv.org/pdf/1908.07401)
        A = np.zeros([12, 12])
        A[0:3, 3:6] = np.eye(3)
        A[3:6, 6:9] = np.array([[0, -g, 0],
                                [g, 0, 0],
                                [0, 0, 0]])
        A[6:9, 9:12] = np.eye(3)

        B = np.zeros([12, 4])
        B[4, 0] = -1 / m
        B[9:12, 1:4] = np.array([[1 / Ixx, 0, 0],
                                 [0, 1 / Iyy, 0],
                                 [0, 0, 1 / Izz]])

        C = np.zeros([6, 12])
        C[0:3, 0:3] = np.eye(3)
        C[3:6, 6:9] = np.eye(3)

        D = np.zeros([6, 4])

        self.sys = signal.StateSpace(A, B, C, D)
        self.discrete_sys = self.sys.to_discrete(dt)

        if verbose:
            logger.info("Continuous State Space Model")
            logger.info("============================")
            logger.info("A: \n", A)
            logger.info("B: \n", B)
            logger.info("C: \n", C)

            logger.info("\nT Hat")
            logger.info("============================")
            logger.info(self.T_hat)

            logger.info("\nS Hat")
            logger.info("============================")
            logger.info(self.S_hat)

        # Define LQR costs
        if Q is None:
            Q = np.eye(N_STATES)
        if R is None:
            R = np.eye(N_CONTROLS)

        self.Q = Q
        self.R = R

        self.Qbar = 10 * np.kron(np.eye(N_MPC_HORIZON), self.Q)
        self.Rbar = 1 * np.kron(np.eye(N_MPC_HORIZON - 1), self.R)

        # Precomputes
        self.S_hat = self.compute_S_hat(self.discrete_sys)
        self.T_hat = self.compute_T_hat(self.discrete_sys)

        logger.info(f"S_hat Shape: {self.S_hat.shape}")
        logger.info(f"T_hat Shape: {self.T_hat.shape}")
        logger.info(f"Qbar Shape: {self.Qbar.shape}")
        logger.info(f"Rbar Shape: {self.Rbar.shape}")

        self.P = sparse.csc_matrix(2 * self.Rbar + 2 * self.S_hat.T @ self.Qbar @ self.S_hat)  # Quadratic cost term

        # Set up the OSQP solver
        self.prob = osqp.OSQP()

        self.x0 = np.zeros((N_STATES, 1))
        self.q = (
                    2 * self.x0.T @ self.T_hat.T @ self.Qbar @ self.S_hat).T  # Recompute linear cost term at each timestep

        self.A = sparse.vstack(
            [sparse.eye(N_CONTROLS * (N_MPC_HORIZON - 1)), -sparse.eye(N_CONTROLS * (N_MPC_HORIZON - 1))], format='csc')
        self.l = -np.inf * np.ones((2 * N_CONTROLS * (N_MPC_HORIZON - 1), 1))

        self.u = np.inf * np.ones((2 * N_CONTROLS * (N_MPC_HORIZON - 1), 1))

        # self.u = np.vstack([np.vstack([np.ones((N_CONTROLS,1))] * (N_MPC_HORIZON-1)), -np.vstack([np.ones((N_CONTROLS,1))]* (N_MPC_HORIZON-1))])

        #logger.info()
        logger.info(f"P_shape: {self.P.shape}")
        logger.info(f"q_shape: {self.q.shape}")
        logger.info(f"A_shape: {self.A.shape}")
        logger.info(f"l_shape: {self.l.shape}")
        logger.info(f"u_shape: {self.u.shape}")

        logger.info(f"P_type: {type(self.P)}")
        logger.info(f"q_type: {type(self.q)}")
        logger.info(f"A_type: {type(self.A)}")
        logger.info(f"l_type: {type(self.l)}")
        logger.info(f"u_type: {type(self.u)}")

        # logger.info("P: ", self.P)
        # logger.info("q: ", self.q)
        # logger.info("A: ", self.A)
        # logger.info("l: ", self.l)
        # logger.info("u: ", self.u)

        # prob.setup(self.P, self.q, self.A, self.l, self.u, warm_starting=True, verbose=False)
        self.prob.setup(self.P, self.q, self.A, self.l, self.u, verbose=True)

        # Enable motor power override via param setting
        self.scf.cf.param.set_value('motorPowerSet.enable', 1)
        self.set_motor_thrusts(np.zeros(4))
        logger.info(self.thrust_to_pwm(0.01))
        logger.info(self.pwm_to_thrust(np.iinfo(np.uint16).max))
        # self.set_motor_thrusts(np.array([0.01, 0.01, 0.01, 0.01]))


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

        # logger.info("x:", self.x0)
        # self.x0 = np.array([x, y, z, vx, vy, vz, yaw, pitch, roll, 0, 0, 0])
        self.x0[0:6] = np.array([x, y, z, vx, vy, vz]).reshape(6, 1)

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

        # logger.info("x:", self.x0)
        # self.x0 = np.array([x, y, z, vx, vy, vz, yaw, pitch, roll, 0, 0, 0])
        self.x0[6:12] = np.array([roll, pitch, yaw, vroll, vpitch, vyaw]).reshape(6, 1)

    def solve_linear_mpc(self, x_ref):

        # Update the linear cost term with the current state and reference
        e = self.x0 - x_ref
        #self.q = (2 * self.x0.T @ self.T_hat.T @ self.Qbar @ self.S_hat).T
        self.q = (2 * e.T @ self.T_hat.T @ self.Qbar @ self.S_hat).T

        self.prob.update(q=self.q)
        res = self.prob.solve()
        if res.info.status != 'solved':
            raise ValueError("OSQP did not solve the problem.")

        u = res.x[:N_CONTROLS]
        self.set_motor_thrusts(0.1 * u)
        #logger.info(f"u: {u}")  # Corrected logging

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

        S_hat = np.zeros([N_STATES * N_MPC_HORIZON, N_CONTROLS * (N_MPC_HORIZON - 1)])
        for k in range(N_MPC_HORIZON):  # state prediction timestep
            for j in range(N_MPC_HORIZON - 1):  # control timestep
                if (k - j >= 0):
                    # logger.info(k," ", j)
                    # logger.info(B.shape)
                    S_hat[k * N_STATES:(k + 1) * N_STATES, j * N_CONTROLS:(j + 1) * N_CONTROLS] = np.linalg.matrix_power(A, k - j) @ B
                    # logger.info()
                    # logger.info((np.linalg.matrix_power(A, k-j) @ B).shape)
                    # logger.info(S_hat[k*N_STATES:(k+1)*N_STATES, j*N_CONTROLS:(j+1)*N_CONTROLS].shape)

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

        T_hat = np.zeros([N_STATES * N_MPC_HORIZON, N_STATES])

        # T_hat = [A, A^2, ..., A^N]';
        for k in range(N_MPC_HORIZON):
            T_hat[k * N_STATES:(k + 1) * N_STATES, :] = np.linalg.matrix_power(A, k + 1)

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
        pwm = pwm / np.uint16(65535)  # normalize pwm to val from 0-1
        thrust = self.pwm_to_thrust_a * pwm * pwm + self.pwm_to_thrust_b * pwm
        return thrust

    def thrust_to_pwm(self, thrust):
        """
        Receives a thrust value in newtons and converts it to the equivalent PWM command.
        Args:
            thrust: Force in newtons produced by the rotor at the provided PWM value.
        Returns:
            pwm (uint16): PWM value commanded to crazyflie motor.
        """

        pwm = (-self.pwm_to_thrust_b + np.sqrt(self.pwm_to_thrust_b ** 2 + 4 * self.pwm_to_thrust_a * thrust)) / (
                    2 * self.pwm_to_thrust_a)
        pwm = pwm * np.uint16(65535)  # convert normalized pwm to value that ranges from 0-UINT16_MAX
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
            self.scf.cf.param.set_value(f'motorPowerSet.m{i + 1}', pwm_value)

        return None

def quadrotor_lqr_cost(x, u, Q, R):
    return x @ Q @ x + u @ R @ u

# Function for starting the flying crazyflie logging
def run_c_crazyflie(scf):

    # Define LQR parameters
    Q = np.eye(N_STATES)
    #Q = np.diag([10.0, 10.0, 10.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])  # Higher values for position states

    R = np.eye(N_CONTROLS)
    #R = np.diag([1.0, 1.0, 1.0, 1.0])  # Reduce these values to allow larger control efforts

    quadrotor_lqr = QuadrotorLQR(dt=0.05, scf=scf, uri=scf.cf.link_uri, Q=Q, R=R, verbose=False)

    # Define a log configuration to get position and orientation data
    p_log_conf = LogConfig(name='t_cf', period_in_ms=100)
    p_log_conf.add_variable('stateEstimate.x', 'float')
    p_log_conf.add_variable('stateEstimate.y', 'float')
    p_log_conf.add_variable('stateEstimate.z', 'float')
    p_log_conf.add_variable('stateEstimate.vx', 'float')
    p_log_conf.add_variable('stateEstimate.vy', 'float')
    p_log_conf.add_variable('stateEstimate.vz', 'float')
    scf.cf.log.add_config(p_log_conf)
    p_log_conf.data_received_cb.add_callback(quadrotor_lqr.translation_state_est_callback)
    p_log_conf.start()

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
    scf.cf.log.add_config(o_log_conf)
    o_log_conf.data_received_cb.add_callback(quadrotor_lqr.orientation_state_est_callback)
    o_log_conf.start()
    logger.info("Logging orientation data...")

    # Control loop
    try:
        while not stop_event.is_set():
            time.sleep(0.1)

            # Check for keyboard input
            #key = check_keyboard_input()
            #if key == 'q':  # Quit if 'q' is pressed
            #    logger.info("Key 'q' pressed. Stopping control loop.")
            #    stop_event.set()
            #    break

            if USE_T_CRAZYFLIE:
                with t_state_lock:
                    x_ref = np.zeros((N_STATES, 1))
                    x_ref[0, 0] = t_crazyflie_state['x']
                    x_ref[1, 0] = t_crazyflie_state['y']
                    x_ref[2, 0] = t_crazyflie_state['z']
            else:
                # Use hardcoded reference position
                x_ref = np.zeros((N_STATES, 1))
                x_ref[0, 0] = 0.0  # Desired X position
                x_ref[1, 0] = 0.0  # Desired Y position
                x_ref[2, 0] = 0.5  # Desired Z position (e.g., hover at 0.5 meters)

            res = quadrotor_lqr.solve_linear_mpc(x_ref)
            u = res.x[:N_CONTROLS]
            logger.info(f"u: {u.round(4)}")

            # Print out the current state and reference
            logger.info(f"x: {quadrotor_lqr.x0[0].round(4)}, {quadrotor_lqr.x0[1].round(4)}, {quadrotor_lqr.x0[2].round(4)}")
            logger.info(f"x_ref: {x_ref[0].round(4)}, {x_ref[1].round(4)}, {x_ref[2].round(4)}")

            # Print out the error between the current state and reference
            e = quadrotor_lqr.x0 - x_ref
            logger.info(f"e: {e[0].round(4)}, {e[1].round(4)}, {e[2].round(4)}")



    except KeyboardInterrupt:
        scf.cf.param.set_value('motorPowerSet.enable', 0)
        p_log_conf.stop()
        o_log_conf.stop()
        logger.info("Control loop stopped.")
    except Exception as e:
        logger.info(f"Error in {scf.cf.link_uri}: {e}")
    finally:
        scf.cf.param.set_value('motorPowerSet.enable', 0)
        p_log_conf.stop()
        o_log_conf.stop()
        logger.info(f"Control loop stopped for {scf.cf.link_uri}")

def start_crazyflie_thread(scf, target):
    def thread_wrapper(scf):
        try:
            target(scf)
        except Exception as e:
            logger.info(f"Error in thread for {scf.cf.link_uri}: {e}")
        finally:
            logger.info(f"Thread for {scf.cf.link_uri} finished.")

    t = Thread(target=thread_wrapper, args=(scf,))
    t.daemon = True
    t.start()
    return t

def run_swarm():

    # Starting background threads for each Crazyflie
    threads = []
    try:
        #connected_uris = [uri for uri in uris if check_connection(uri)]
        #if not connected_uris:
        #    logger.info("No Crazyflies connected. Exiting...")
        #    return

        with Swarm(uris) as swarm:
            scfs = swarm._cfs # Crazyflie instances

            if URI_C in scfs:
                threads.append(start_crazyflie_thread(scfs[URI_C], run_c_crazyflie))
            if USE_T_CRAZYFLIE and URI_T in scfs:
                threads.append(start_crazyflie_thread(scfs[URI_T], run_t_crazyflie))

            try:
                # Wait for all threads to finish
                for thread in threads:
                    thread.join()
                    #thread.join(timeout=5)
                    if thread.is_alive():
                        logger.info("Thread did not finish in time (5 seconds).")
            except KeyboardInterrupt:
                logger.info("Keyboard interrupt detected. Stopping threads")
                stop_event.set()  # Signal all threads to stop

    finally:
        #for scf in scfs.values():
        #    scf.cf.param.set_value('motorPowerSet.enable', 0)
        for uri in uris:
            try:
                logger.info(f"Disconnecting {uri}...")
                with SyncCrazyflie(uri) as scf:
                    scf.close_link()
            except Exception as e:
                logger.info(f"Error while disconnecting {uri}: {e}")
        logger.info("Crazyflie threads stopped.")
        # If any threads are still alive, force terminate the process
        for thread in threads:
            if thread.is_alive():
                logger.error("Forcefully terminating the script as threads are still running.")
                os._exit(1)

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    run_swarm()