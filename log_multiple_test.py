import time
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

uri_dir = {'tumbller_uri': 'radio://0/20/2M/E7E7E7E702',
           'crazyflie_uri': 'radio://0/20/2M/E7E7E7E701'}

uris = [
    #'radio://0/20/2M/E7E7E7E701', # Crazyflie
    'radio://0/20/2M/E7E7E7E702', # Tumbller
]

tumbller_position = {'x':0, 'y':0, 'z':0, 'roll':0, 'pitch':0, 'yaw':0}
crazyflie_position = {'x':0, 'y':0, 'z':0, 'roll':0, 'pitch':0, 'yaw':0}

trans_t = {'last_time': time.time(),
           'avg_100_samples': [0] * 100  # Initialize as a list with 100 elements
           }
trans_c = {'last_time': time.time(),
           'avg_100_samples': [0] * 100  # Initialize as a list with 100 elements
           }

def log_trans_callback(uri, timestamp, data, log_conf):
    """Callback function that logs the translational data and updates the global position variables with the matching uri
    """
    x = round(float(data['stateEstimate.x']), 4) if 'stateEstimate.x' in data else 0.0
    y = round(float(data['stateEstimate.y']), 4) if 'stateEstimate.y' in data else 0.0
    z = round(float(data['stateEstimate.z']), 4) if 'stateEstimate.z' in data else 0.0

    if uri==uri_dir['tumbller_uri']:
        tumbller_position['x'] = x
        tumbller_position['y'] = y
        tumbller_position['z'] = z
        rate = round(1/(time.time() - trans_t['last_time']), 2)
        print(f" Tumbller Rate: {rate} ")
        trans_t['last_time'] = time.time()

        # Calculate average of past 100 samples
        trans_t['avg_100_samples'].pop(0)
        trans_t['avg_100_samples'].append(rate)
        print(f"Average Rate: {np.mean(trans_t['avg_100_samples'])}")


    elif uri==uri_dir['crazyflie_uri']:
        crazyflie_position['x'] = x
        crazyflie_position['y'] = y
        crazyflie_position['z'] = z
        rate = round(1/(time.time() - trans_c['last_time']), 2)
        print(f" Crazyflie Rate: {rate} ")
        trans_c['last_time'] = time.time()

        # Calculate average of past 100 samples
        trans_c['avg_100_samples'].pop(0)
        trans_c['avg_100_samples'].append(rate)
        print(f"Average Rate: {np.mean(trans_c['avg_100_samples'])}")


def log_orient_callback(uri, timestamp, data, log_conf):
    """Callback function that logs teh data and updates the global position variables with the matching uri
    """
    roll = round(float(data['stabilizer.roll']), 4) if 'stabilizer.roll' in data else 0.0
    pitch = round(float(data['stabilizer.pitch']), 4) if 'stabilizer.pitch' in data else 0.0
    yaw = round(float(data['stabilizer.yaw']), 4) if 'stabilizer.yaw' in data else 0.0

    if uri==uri_dir['tumbller_uri']:
        tumbller_position['roll'] = roll
        tumbller_position['pitch'] = pitch
        tumbller_position['yaw'] = yaw

    elif uri==uri_dir['crazyflie_uri']:
        crazyflie_position['roll'] = roll
        crazyflie_position['pitch'] = pitch
        crazyflie_position['yaw'] = yaw

def log_async(scf):

    # Define the logging variables
    t_lg_vars = {
        'stateEstimate.x': 'float',
        'stateEstimate.y': 'float',
        'stateEstimate.z': 'float',
        'stateEstimate.vx': 'float',
        'stateEstimate.vy': 'float',
        'stateEstimate.vz': 'float',
    }
    o_lg_vars = {
        'stabilizer.roll': 'float',
        'stabilizer.pitch': 'float',
        'stabilizer.yaw': 'float',
        'stateEstimateZ.rateRoll': 'float',
        'stateEstimateZ.ratePitch': 'float',
        'stateEstimateZ.rateYaw': 'float',
    }

    # Define log configuration for translation
    t_log_conf = LogConfig(name="Translation", period_in_ms=10)
    o_log_conf = LogConfig(name="Orientation", period_in_ms=10)

    for key in t_lg_vars:
        t_log_conf.add_variable(key, t_lg_vars[key])
    for key in o_lg_vars:
        o_log_conf.add_variable(key, o_lg_vars[key])

    # Add log configuration to Crazyflie
    scf.cf.log.add_config(t_log_conf)
    scf.cf.log.add_config(o_log_conf)

    if t_log_conf.valid:
        # Start logging translation data
        t_log_conf.data_received_cb.add_callback(lambda t, d, l: log_trans_callback(scf.cf.link_uri, t, d, l))
        t_log_conf.start()
        print("Logging translation data...")

    if o_log_conf.valid:
        # Start logging orientation data
        o_log_conf.data_received_cb.add_callback(lambda t, d, l: log_orient_callback(scf.cf.link_uri, t, d, l))
        o_log_conf.start()
        print("Logging orientation data...")


if __name__ == '__main__':

    cflib.crtp.init_drivers(enable_debug_driver=False)  # initialize drivers
    factory = CachedCfFactory(rw_cache='./cache')

    try:
        with Swarm(uris, factory=factory) as swarm:
            swarm.parallel_safe(log_async)

            while True:
                time.sleep(0.5)
                print("Tumbller position: ", tumbller_position)
                print("Crazyflie position: ", crazyflie_position)


                #var = input("insert command:\n")

    except KeyboardInterrupt:
        print("Interrupted by user.")

