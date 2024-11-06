# -*- coding: utf-8 -*-
#
# This script connects to the Crazyflie and continuously outputs
# the position and orientation of the drone.

import logging
import time
import numpy as np

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Define the URI for the Crazyflie
radio = 'radio://0/20/2M/E7E7E7E702'
uri = uri_helper.uri_from_env(default=radio)


# Callback to log position and orientation
def position_log_callback(timestamp, data, logconf):
    x = data['stateEstimate.x']
    y = data['stateEstimate.y']
    z = data['stateEstimate.z']
    yaw = data['stateEstimate.yaw']
    pitch = data['stateEstimate.pitch']
    roll = data['stateEstimate.roll']
    print(f"[{logconf.name}][{timestamp}] Pose: x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={yaw:.2f}, pitch={pitch:.2f}, roll={roll:.2f}")


def main():
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # Define a log configuration to get position and orientation data
    log_conf = LogConfig(name='Tumbller', period_in_ms=100)
    log_conf.add_variable('stateEstimate.x', 'float')
    log_conf.add_variable('stateEstimate.y', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')
    log_conf.add_variable('stateEstimate.yaw', 'float')
    log_conf.add_variable('stateEstimate.pitch', 'float')
    log_conf.add_variable('stateEstimate.roll', 'float')

    # Connect to the Crazyflie and start logging
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Connected to Crazyflie.")
        
        # Add the log configuration to the Crazyflie
        scf.cf.log.add_config(log_conf)
        
        # Start logging if the configuration is added successfully
        if log_conf.valid:
            log_conf.data_received_cb.add_callback(position_log_callback)
            log_conf.start()
            print("Logging position and orientation data...")

            # Keep the connection open and outputting data
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                print("Logging stopped.")

            log_conf.stop()
        else:
            print("The log configuration is not supported by the Crazyflie firmware.")

if __name__ == '__main__':
    main()
