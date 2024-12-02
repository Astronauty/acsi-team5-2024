# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

import logging
import time
import math
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E701')

DEFAULT_HEIGHT = 1.5

logging.basicConfig(level=logging.ERROR)

position_estimate = [0,0,0]
position_log = []


def run_sequence(scf, hover_time=10, ref_point=(0, 0, 0.5, 0)):
    cf = scf.cf

    x = ref_point[0]
    y = ref_point[1]
    z = ref_point[2]
    yaw = ref_point[3]

    print(f"Hovering at position {ref_point[:3]} for {hover_time} seconds...")
    start_time = time.time()

    while time.time() - start_time < hover_time:
        cf.commander.send_position_setpoint(x, y, z, yaw)
        time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

def log_pos_callback(timestamp, data, logconf):
    print(data)
    global position_estimate, position_log
    position_estimate = [data['stateEstimate.x'],
                         data['stateEstimate.y'],
                         data['stateEstimate.z'],
                         data['stateEstimate.yaw']]
    position_log.append(position_estimate)

def reset_estimator(scf):
    """Reset and wait for the position estimator to initialize."""
    print('Resetting position estimator...')
    scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    scf.cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(scf)

def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

def wait_for_position_estimator(scf):
    """Wait for the Kalman filter to stabilize."""
    print('Waiting for position estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_history = {'x': [1000] * 10, 'y': [1000] * 10, 'z': [1000] * 10}
    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]
            var_history['x'].append(data['kalman.varPX'])
            var_history['y'].append(data['kalman.varPY'])
            var_history['z'].append(data['kalman.varPZ'])

            var_history['x'].pop(0)
            var_history['y'].pop(0)
            var_history['z'].pop(0)

            if all(
                max(var_history[axis]) - min(var_history[axis]) < threshold
                for axis in var_history
            ):
                print('Position estimator ready.')
                break

def init_pos_log(scf):
    logconf = LogConfig(name='Position', period_in_ms=10)
    logconf.add_variable('stateEstimate.x', 'float')
    logconf.add_variable('stateEstimate.y', 'float')
    logconf.add_variable('stateEstimate.z', 'float')
    logconf.add_variable('stateEstimate.yaw', 'float')
    scf.cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_pos_callback)
    logconf.start()

    return logconf

if __name__ == '__main__':

    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        time.sleep(1)

        logconf = init_pos_log(scf)
        set_initial_position(scf, 1, 0, 0, 0)
        reset_estimator(scf)

        run_sequence(scf, hover_time=10)
        logconf.stop()

    # Write the xyz data to a csv file
    np.savetxt('position_log.csv', np.array(position_log), delimiter=',')
    print('Data saved to position_estimate.csv')
