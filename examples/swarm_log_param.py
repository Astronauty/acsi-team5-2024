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
    #print(f'[{timestamp}][{logconf.name}]: {data}')
    #x = data['stateEstimate.x']
    x = 0.00
    y = data['stateEstimate.y']
    z = data['stateEstimate.z']
    yaw = data['stateEstimate.yaw']
    pitch = data['stateEstimate.pitch']
    roll = data['stateEstimate.roll']
    print(f"[{logconf.name}][{timestamp}] Pose: x={x:.3f}, y={y:.3f}, z={z:.3f}, roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")

def log_imu_data(scf):
    cf = scf.cf
    lg_imu = LogConfig(name='Position', period_in_ms=100)
    #lg_imu.add_variable('stateEstimate.x', 'float') # position
    lg_imu.add_variable('stateEstimate.y', 'float')
    lg_imu.add_variable('stateEstimate.z', 'float')
    lg_imu.add_variable('stateEstimate.roll', 'float') # orientation
    lg_imu.add_variable('stateEstimate.pitch', 'float')
    lg_imu.add_variable('stateEstimate.yaw', 'float')
    lg_imu.add_variable('stabilizer.thrust', 'float') # current thrust
        

    cf.log.add_config(lg_imu)
    lg_imu.data_received_cb.add_callback(imu_data_callback)
    lg_imu.start()
    return lg_imu

def take_off(scf):
    commander = scf.cf.high_level_commander
    commander.takeoff(1.0, 2.0)
    time.sleep(3)

def land(scf):
    commander = scf.cf.high_level_commander
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()

def activate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 255)

def deactivate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 0)

def light_check(scf):
    activate_led_bit_mask(scf)
    time.sleep(2)
    deactivate_led_bit_mask(scf)
    time.sleep(2)


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    with Swarm(uris, factory=factory) as swarm:
        print('Connected to Crazyflies')
        
        # Log IMU data for each drone
        swarm.parallel_safe(lambda scf: log_imu_data(scf))
        
        # Run the flight sequence with light checks and maneuvers
        #swarm.parallel_safe(light_check)
        #swarm.parallel_safe(take_off)
        #swarm.parallel_safe(run_square_sequence)
        #swarm.parallel_safe(land)

        # Let the logging run for a specified duration
        #time.sleep(10)  # Change duration as needed

        # Optionally stop logging after the duration
        swarm.parallel_safe(lambda scf: scf.cf.log.stop_all())

        time.sleep(5)


