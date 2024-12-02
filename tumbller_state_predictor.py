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

URI = dict()
# URI['cf'] = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E702')
URI['tb'] = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E701')

class TumbllerStatePredictor():
    def __init__(self, prediciton_horizon_time):
        # Initialize tumbller state
        self.tb_state = np.zeros(6)
        self.prediction_horizon_time = prediciton_horizon_time
        
        # Define a log configuration to get position and orientation data
        t_log_conf = LogConfig(name='t_cf', period_in_ms=50)
        t_log_conf.add_variable('stateEstimate.x', 'float')
        t_log_conf.add_variable('stateEstimate.y', 'float')
        t_log_conf.add_variable('stateEstimate.z', 'float')
        t_log_conf.add_variable('stateEstimate.vx', 'float')
        t_log_conf.add_variable('stateEstimate.vy', 'float')
        t_log_conf.add_variable('stateEstimate.vz', 'float')

        o_log_conf = LogConfig(name='o_cf', period_in_ms=50)
        # o_log_conf.add_variable('stateEstimate.roll', 'float')
        # o_log_conf.add_variable('stateEstimate.pitch', 'float')
        # o_log_conf.add_variable('stateEstimate.yaw', 'float')
        o_log_conf.add_variable('stabilizer.roll', 'float')
        o_log_conf.add_variable('stabilizer.pitch', 'float')
        o_log_conf.add_variable('stabilizer.yaw', 'float')
        o_log_conf.add_variable('stateEstimateZ.rateRoll', 'int16_t')
        o_log_conf.add_variable('stateEstimateZ.ratePitch', 'int16_t')
        o_log_conf.add_variable('stateEstimateZ.rateYaw', 'int16_t')

        with SyncCrazyflie(URI['cf'], cf=Crazyflie(rw_cache='./cache')) as scf:
            print("Connected to Crazyflie.")
            
            # Add the log configuration to the Crazyflie
            scf.cf.log.add_config(t_log_conf)
            scf.cf.log.add_config(o_log_conf)
            
            # Start logging if the configuration is added successfully
            if t_log_conf.valid and o_log_conf.valid:
                t_log_conf.data_received_cb.add_callback(self.translation_state_est_callback)
                # o_log_conf.data_received_cb.add_callback(self.orientation_state_est_callback)
                
                t_log_conf.start()
                # o_log_conf.start()
                print("Logging position and orientation data...")
        return None
        
    def translation_state_est_callback(self, timestamp, data, logconf):
        x = data['stateEstimate.x']
        y = data['stateEstimate.y']
        z = data['stateEstimate.z']
        vx = data['stateEstimate.vx']
        vy = data['stateEstimate.vy']
        vz = data['stateEstimate.vz']
        self.tb_state[0:6] = np.array([x, y, z, vx, vy, vz])    
        return None
    
    def orientation_state_est_callback(self, timestamp, data, logconf):
        roll = data['stabilizer.roll']
        pitch = data['stabilizer.pitch']
        yaw = data['stabilizer.yaw']
        vroll = data['stateEstimateZ.rateRoll']
        vpitch = data['stateEstimateZ.ratePitch']
        vyaw = data['stateEstimateZ.rateYaw']
        self.tb_state[6:12] = np.array([roll, pitch, yaw, vroll, vpitch, vyaw])
        return None
        

    def tumbller_state_prediction(self, t_pred):
        trans_pos = self.tb_state[0:3]
        trans_vel = self.tb_state[3:6]
        orient_pos = self.tb_state[6:9]
        orient_vel = self.tb_state[9:12]
        
        future_trans_pos = self.tb_state[0:3] + t_pred*trans_vel
        future_orient_pos = self.tb_state[6:9] + t_pred*orient_vel
        return np.concatenate(future_trans_pos, np.zeros(3), future_orient_pos, np.zeros(3))

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    tumbller_state_predictor = TumbllerStatePredictor()
    
