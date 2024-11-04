import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

# Set up logging
logging.basicConfig(level=logging.ERROR)

# URIs of the Crazyflies
uris = {
    #'radio://0/20/2M/E7E7E7E701',
    'radio://0/20/2M/E7E7E7E702',
}

# Callback for logging IMU data
def imu_data_callback(timestamp, data, logconf):
    print(f'[{timestamp}][{logconf.name}]: {data}')

def log_imu_data(scf):
    cf = scf.cf
    lg_imu = LogConfig(name='IMU', period_in_ms=100)
    #lg_imu.add_variable('acc.x', 'float')        # acceleration
    #lg_imu.add_variable('acc.y', 'float')
    #lg_imu.add_variable('acc.z', 'float')
    #lg_imu.add_variable('gyro.x', 'float')       # gyro
    #lg_imu.add_variable('gyro.y', 'float')
    #lg_imu.add_variable('gyro.z', 'float')
    #logconf = LogConfig(name='Position', period_in_ms=10)
    #lg_imu.add_variable('stateEstimate.x', 'float') # position (throws error if not using positioning system)
    #lg_imu.add_variable('stateEstimate.y', 'float')
    #lg_imu.add_variable('stateEstimate.z', 'float')
    lg_imu.add_variable('stateEstimate.roll', 'float') # orientation
    lg_imu.add_variable('stateEstimate.pitch', 'float')
    lg_imu.add_variable('stateEstimate.yaw', 'float')
    lg_imu.add_variable('stabilizer.thrust', 'float') # current thrust
        

    cf.log.add_config(lg_imu)
    lg_imu.data_received_cb.add_callback(imu_data_callback)
    lg_imu.start()
    return lg_imu

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    with Swarm(uris, factory=factory) as swarm:
        print('Connected to Crazyflies')
        
        # Log IMU data for each drone
        swarm.parallel_safe(lambda scf: log_imu_data(scf))
        
        # Let the logging run for a specified duration
        time.sleep(10)  # Change duration as needed

        # Optionally stop logging after the duration
        swarm.parallel_safe(lambda scf: scf.cf.log.stop_all())

        time.sleep(5)


