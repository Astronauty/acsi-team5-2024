import time

from cflib.crtp import init_drivers
#from cflib.crazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm


class CrazyflieDevice:
    """ Class to store data and functionality for single Crazyflie device"""
    def __init__(self, name, uri, sampling_rate=50):
        self.name = name
        self.uri = uri
        self.sample_rate = sampling_rate
        self.position = {'x':0, 'y':0, 'z':0}
        self.orientation = {'roll':0, 'pitch':0, 'yaw':0}
        self.last_log_t = time.time()
        self.avg_100_samples = [0] * 100  # Initialize as a list with 100 elements

    def get_position(self):
        """ Returns the device's position as a list [x, y, z]"""
        return [self.position['x'], self.position['y'], self.position['z']]

    def update_position(self, x, y, z):
        self.position['x'] = x
        self.position['y'] = y
        self.position['z'] = z

    def update_orientation(self, roll, pitch, yaw):
        self.orientation['roll'] = roll
        self.orientation['pitch'] = pitch
        self.orientation['yaw'] = yaw

    def update_rate(self):
        current_time = time.time()
        rate = round(1 / (current_time - self.last_log_t), 2)
        self.last_log_t = current_time

        # Calculate average of past 100 samples
        self.avg_100_samples.pop(0)
        self.avg_100_samples.append(rate)
        return rate, sum(self.avg_100_samples) / len(self.avg_100_samples)

# Making a dictionary of crazyflie devices
devices = {'tumbller': CrazyflieDevice('Tumbller', 'radio://0/20/2M/E7E7E7E702'),
           'crazyflie': CrazyflieDevice('Crazyflie', 'radio://0/20/2M/E7E7E7E701')}

# Also need a separate list of URIs for the Swarm class
uris = [
    devices['tumbller'].uri, # Tumbller crazyflie
#    uri_dir['crazyflie_uri'], # Aerial crazyflie
]

# Parameters for the takeoff and land functions for specific crazyflies
params = {devices['tumbller'].uri: [{'device': 'tumbller'}]}

# Callback function to update the global position variables with the matching uri, No need to change
def log_trans_callback(uri, timestamp, data, log_conf):
    """Callback function that logs the translational data and updates the global position variables with the matching uri
    """
    x = round(float(data.get('stateEstimate.x', 0.0)), 4)
    y = round(float(data.get('stateEstimate.y', 0.0)), 4)
    z = round(float(data.get('stateEstimate.z', 0.0)), 4)
    # Find the device associated with this URI
    for device in devices.values():
        if device.uri == uri:
            device.update_position(x, y, z)
            rate, avg_rate = device.update_rate()
            print(f'{device.name} position: {device.position}')
            # print(f'{devices[uri].name} Rate: {rate}')
            break

# Callback function to update the global orientation variables with the matching uri, No need to change
def log_orient_callback(uri, timestamp, data, log_conf):
    """Callback function that logs teh data and updates the global position variables with the matching uri
    """
    roll = round(float(data.get('stabilizer.roll', 0.0)), 4)
    pitch = round(float(data.get('stabilizer.pitch', 0.0)), 4)
    yaw = round(float(data.get('stabilizer.yaw', 0.0)), 4)
    for device in devices.values():
        if device.uri == uri:
            device.update_orientation(roll, pitch, yaw)
            break

# Function to setup the logging configurations for the crazyflie devices. No need to change
def setup_async_logging(scf):

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

    # Define log configuration for translation and orientation for all crazyflies
    t_log_conf = LogConfig(name="Translation", period_in_ms=20)
    for key in t_lg_vars:
        t_log_conf.add_variable(key, t_lg_vars[key])

    o_log_conf = LogConfig(name="Orientation", period_in_ms=20)
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


def takeoff(scf, params):
    if 'device' in params:
        if params['device'] == 'tumbller':
            print("Sending command to Tumbller...")
            scf.cf.commander.send_position_setpoint(0, 0, 1.6, 0)
            #time.sleep(3)


def land(scf, params):
    if 'device' in params:
        if params['device'] == 'tumbller':
            print("Landing Tumbller...")
            scf.cf.commander.send_position_setpoint(0, 0, 0, 0)
            time.sleep(3)

            scf.cf.commander.send_stop_setpoint()
            # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
            scf.cf.commander.send_notify_setpoint_stop()


if __name__ == '__main__':

    init_drivers(enable_debug_driver=False)  # initialize drivers
    factory = CachedCfFactory(rw_cache='./cache') # For reducing connection time

    try:
        with Swarm(uris, factory=factory) as swarm:
            swarm.parallel_safe(setup_async_logging)
            swarm.parallel_safe(takeoff, args_dict=params)

            # Configuration should be done at this point. Motion commander can now be called for the specific crazyflie
            while True:
                [x, y, z] = devices['tumbller'].get_position()
                print(f'Tumbller position: {x}, {y}, {z}')

                # send command to land if it reaches its height
                if z >= 1.5:
                    print("Landing...")
                    swarm.parallel_safe(land, args_dict=params)
                    break
                else:
                    print("climb!")

                time.sleep(0.05)

            print("Done")

    except KeyboardInterrupt:
        print("Interrupted by user.")

