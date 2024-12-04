import time
import numpy as np

from cflib.crtp import init_drivers
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncLogger import SyncLogger
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
#   devices['crazyflie'].uri, # Aerial crazyflie
]


# Callback function to update the global position variables with the matching uri
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
            break

# Callback function to update the global orientation variables with the matching uri
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

# Function to set up the logging configurations for the crazyflie devices
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
    t_log_conf = LogConfig(name="Translation", period_in_ms=100)
    for key in t_lg_vars:
        t_log_conf.add_variable(key, t_lg_vars[key])

    o_log_conf = LogConfig(name="Orientation", period_in_ms=100)
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

# Resets the state estimator on the crazyflie
def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

# Works in-tandem with the reset_estimator function to wait for the position estimator to find the position
def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

# Function to takeoff the crazyflie
def takeoff(scf, params):
    # Params no longer has the URI as the key, the dictionary in the value replaces params
    if params['enable']:
        print("Taking off")
        z_values = np.linspace(0, 1.6, 50)
        for z in z_values:
            scf.cf.commander.send_position_setpoint(0, 0, z, 0)
            time.sleep(0.1)

    print("Exiting takeoff function")

# Function to land the crazyflie
def land(scf, params):
    if params['land']:
        print("Landing")
        z_values = np.linspace(1.6, 0, 50)
        for z in z_values:
            scf.cf.commander.send_position_setpoint(0, 0, z, 0)
            time.sleep(0.1)
    print("Exiting land function")

# Function to stop all crazyflies
def stop_all(scf):
    scf.cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    scf.cf.commander.send_notify_setpoint_stop()
    time.sleep(0.1)



if __name__ == '__main__':

    init_drivers(enable_debug_driver=False)  # initialize drivers
    factory = CachedCfFactory(rw_cache='./cache') # For reducing connection time

    try:
        with Swarm(uris, factory=factory) as swarm:
            # Set up the logging configurations for the crazyflie devices
            swarm.parallel_safe(setup_async_logging)
            swarm.parallel_safe(reset_estimator)

            # We will pass in our 'param' dictionary to contain our arbitrary parameters. In this case, the "takeoff"
            # function is looking for the 'enable' key and if it is set to True, it will takeoff. The key has to be the
            # URI value, and teh value is another dictionary with the key 'enable' and the value
            params = {devices['tumbller'].uri: [{'enable': True}],
                      devices['crazyflie'].uri: [{'enable': False}]}
            swarm.parallel_safe(takeoff, args_dict=params) # Executes the function

            # Configuration should be done at this point.
            while True:

                # Sample the current position of the crazyflie
                [x, y, z] = devices['tumbller'].get_position()

                # If it reaches its height, exit out of the loop and land the crazyflie
                if z >= 1.6:
                    break
                else:
                    print(f'Tumbller height: {z}')

                time.sleep(0.05)

            print("Landing...")
            # Set up the parameters so the crazyflie is the only one doing the landing sequence. The "land" funtion only
            # looks for the 'land' key, and if it is set to True, it will land
            params = {devices['tumbller'].uri: [{'land': True}],
                      devices['crazyflie'].uri: [{'land': False}]}
            swarm.parallel_safe(land, args_dict=params)
            print("Done")

    except KeyboardInterrupt:
        print("Interrupted by user.")

