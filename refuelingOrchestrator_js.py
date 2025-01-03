import logging
import os
import time
import datetime
import numpy as np
from threading import Event, Thread
from pynput import keyboard
from scipy.spatial.transform import Rotation as R

from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
from cflib.crtp import init_drivers
from cflib.crazyflie.swarm import Swarm, CachedCfFactory

logging.basicConfig(level=logging.WARNING, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
stop_event = Event()

class RefuelingOrchestrator():
    def __init__(self, URI, max_scenario_time, tether_length=0.2, log_pos=False, log_rate=20, simulate=False, verbose=False):
        # Check if URIs are defined
        #if 'tb' not in URI:
        #    logger.error("Warning: CrazyFlie tumbller URI not defined")

        ## Params
        self.URI = URI
        self.max_scenario_time = max_scenario_time
        self.tether_length = tether_length
        self.log_pos = log_pos
        self.log_rate = log_rate
        self.simulate = simulate
        self.verbose = verbose
        self.mode_select = 0  # Phase 0 - Init & No Controls, 1 - Dwell, 2 - Rendesvous & Follow, 3 - Detach
        self.new_state = False
        self.cf_state = np.zeros(12)
        self.tb_state = np.zeros(12)
        self.cf_reference_pos = np.zeros(12)
        self.platform_offset = [-0.1, 0, 0] # Offset of the platform from the tumbller in the local frame [x, y, z]

        # Create the position and reference log file
        data = "time, x, y, z, vx, vy, vz, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate"
        self.write_to_pos_log('cf_pos_log.txt', data, new_file=True)
        self.write_to_pos_log('cf_ref_log.txt', data, new_file=True)

        # initialize drivers and factory for reducing connection time
        init_drivers(enable_debug_driver=False)
        factory = CachedCfFactory(rw_cache='./cache')

        # Initialize key listener in separate thread
        listener_thread = Thread(target=self.start_key_listener)
        listener_thread.daemon = True
        listener_thread.start()

        with Swarm(list(URI.values()), factory=factory) as swarm:
            scfs = swarm._cfs  # Crazyflie instances
            self.cf_translation_log_conf, self.cf_orientation_log_conf, self.tb_translation_log_conf, self.tb_orientation_log_conf = self.initialize_async_logs()
            try:
                self.start_time = time.time()
                self.end_time = self.start_time + self.max_scenario_time

                threads = []
                if 'cf' in URI:
                    threads.append(self.start_crazyflie_thread(scfs[URI['cf']], self.run_cf))
                    print(f"Successfully started Crazyflie thread.")
                if 'tb' in URI:
                    threads.append(self.start_crazyflie_thread(scfs[URI['tb']], self.run_tb))
                    print(f"Successfully started Tumbller thread.")

                # Wait for all threads to finish
                for thread in threads:
                    thread.join()
                    if thread.is_alive():
                        logger.info("Thread did not finish in time (5 seconds).")
                print("Threads done!")

            except KeyboardInterrupt:
                logger.info("Keyboard interrupt detected. Stopping threads")
                stop_event.set()  # Signal all threads to stop
            finally:
                # Extra call in case the threads don't call stop_scf() ...
                swarm.parallel_safe(self.stop_scf)

    def T_platform_from_tb(self, tb_pos, tb_orientation):
        """ Helper function tha calculates the transformation from the tumbller to the platform

        tb_pos: tuple or list: x, y, z position of the tumbller
        tb_orientation: tuple or list: roll, pitch, yaw orientation of the tumbller
        """
        p_local = self.platform_offset

        # rotation matrix of tumbler yaw angle
        #print("yaw_angle (deg): ", tb_orientation[2])
        yaw_angle = np.radians(tb_orientation[2])
        R = np.array([[np.cos(yaw_angle), -np.sin(yaw_angle), 0],
                             [np.sin(yaw_angle), np.cos(yaw_angle), 0],
                             [0, 0, 1]])

        # Compute the global position of the platform
        p_global = R @ p_local + tb_pos

        return p_global

    def move_drone_old(self, scf, x, y, z, yaw):
        if not self.simulate:
            if self.verbose:
                print(f"Setting drone to position: {x}, {y}, {z}, {yaw}")
            scf.cf.commander.send_position_setpoint(x, y, z, yaw)
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")
        data = [timestamp] + [x, y, z, 0, 0, 0, 0, 0, yaw, 0, 0, 0]  # Fill in the data
        self.write_to_pos_log('cf_ref_log.txt', data)

    def move_drone(self, scf, pos, vel, orientation):
        if not self.simulate:
            if self.verbose:
                print(f"Setting drone to position: {pos[0]}, {pos[1]}, {pos[2]}, {orientation[2]}")
            q = R.from_euler('xyz', orientation, degrees=True).as_quat()  # Returns [x, y, z, w]
            scf.cf.commander.send_full_state_setpoint(pos, vel, [0, 0, 0], q, 0, 0, 0)
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")
        data = [timestamp] + [pos[0], pos[1], pos[2], 0, 0, 0, orientation[0], orientation[1], orientation[2], 0, 0, 0]  # Fill in the data
        self.write_to_pos_log('cf_ref_log.txt', data)

    def write_to_pos_log(self, filename, data, new_file=False):
        # Helper function to write data to the passed filename
        if self.log_pos:
            if not os.path.exists(filename) or new_file:
                with open(filename, 'w') as f:
                    f.write(f"{data}\n")
            else:
                with open(filename, 'a') as f:
                    f.write(f"{data}\n")

    def stop_scf(self, scf):
        # Function to stop all crazyflies
        scf.cf.commander.send_stop_setpoint()
        # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
        scf.cf.commander.send_notify_setpoint_stop()
        time.sleep(0.1)

    def update_logs(self, msg):
        # Helper function just to write the phases to logs
        if self.new_state:
            self.new_state = False
            timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")
            data = [timestamp] + [msg]  # Fill in the data
            self.write_to_pos_log('cf_pos_log.txt', data)
            self.write_to_pos_log('tb_pos_log.txt', data)
            self.write_to_pos_log('cf_ref_log.txt', data)

    def run_tb(self, scf):
        # Add log configs and callbacks to the tb instance
        scf.cf.log.add_config(self.tb_translation_log_conf)
        self.tb_translation_log_conf.data_received_cb.add_callback(self.tb_translation_callback)
        self.tb_translation_log_conf.start()

        scf.cf.log.add_config(self.tb_orientation_log_conf)
        self.tb_orientation_log_conf.data_received_cb.add_callback(self.tb_orientation_callback)
        self.tb_orientation_log_conf.start()

        logger.info(f"Started logging for {self.URI['tb']}.")
        print("TB initialization complete!")

        try:
            while not stop_event.is_set():
                match self.mode_select:
                    case 'q':
                        print("Quitting")
                        break

                    case _:
                        pass


                if self.verbose:
                    print(f"TB State: {self.tb_state[0:3]}")

                #platform_pos = self.T_platform_from_tb(self.tb_state[0:3], self.tb_state[6:9])
                #print(f"Platform State: {platform_pos}")

                # Get the timestamp, including milliseconds
                timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")
                data = [timestamp] + self.tb_state.tolist() # Fill in the data
                self.write_to_pos_log('tb_pos_log.txt', data)

                time.sleep(1/self.log_rate) # Wait for a bit, rate update

        except KeyboardInterrupt:
            logger.info(f"Keyboard interrupt detected for {self.URI['tb']}.")
            stop_event.set()  # Signal all threads to stop
        finally:
            self.stop_scf(scf)
            self.tb_translation_log_conf.stop()
            self.tb_orientation_log_conf.stop()
            logger.info(f"Stopped logging for {self.URI['tb']}.")

    def run_cf(self, scf):
        # Add log configs and callbacks to the cf instance
        scf.cf.log.add_config(self.cf_translation_log_conf)
        self.cf_translation_log_conf.data_received_cb.add_callback(self.cf_translation_callback)
        self.cf_translation_log_conf.start()

        scf.cf.log.add_config(self.cf_orientation_log_conf)
        self.cf_orientation_log_conf.data_received_cb.add_callback(self.cf_orientation_callback)
        self.cf_orientation_log_conf.start()

        logger.info(f"Started logging for {self.URI['cf']}.")
        print("CF initialization complete!")

        try:
            while time.time() < self.end_time or not stop_event.is_set():
                match self.mode_select:
                    case 1:
                        self.update_logs('Phase 1: Dwell')
                        self.move_drone(scf,[0, 0, 0.5], [0,0,0], [0,0,0])

                    case 2:
                        self.update_logs('Phase 2: Rendesvous & Follow')
                        # update the cf_reference to be the position of the tb plus tether length
                        self.cf_reference_pos = self.tb_state[0:3] + np.array([0, 0, self.tether_length])
                        platform_pos = self.T_platform_from_tb(self.cf_reference_pos[0:3], self.tb_state[6:9])
                        self.move_drone(scf, platform_pos, self.tb_state[3:6], [0,0,0])

                    case 3:
                        self.update_logs('Phase 3: Attach and Follow')
                        self.cf_reference_pos = self.tb_state[0:3] + np.array([0, 0, self.tether_length-0.20])
                        platform_pos = self.T_platform_from_tb(self.cf_reference_pos[0:3], self.tb_state[6:9])
                        self.move_drone(scf, platform_pos, self.tb_state[3:6], [0,0,0])

                    case 4:
                        self.update_logs('Phase 4: Detach (Landing)')
                        self.move_drone(scf, [0, 0, 0.1], [0,0,0], [0,0,0])

                    case 'q':
                        print("Quitting")
                        break

                    case _:
                        pass

                # Print out the tumbler state
                if self.verbose:
                    print(f"CF State: {self.cf_state[0:3]}")

                # Get the timestamp, including milliseconds
                timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")
                data = [timestamp] + self.cf_state.tolist() # Fill in the data
                self.write_to_pos_log('cf_pos_log.txt', data)

                time.sleep(1/self.log_rate) # Wait for a bit, rate update

        except KeyboardInterrupt:
            logger.info(f"Keyboard interrupt detected for {self.URI['cf']}.")
            stop_event.set()  # Signal all threads to stop
        finally:
            self.stop_scf(scf)
            self.cf_translation_log_conf.stop()
            self.cf_orientation_log_conf.stop()
            logger.info(f"Stopped logging for {self.URI['cf']}")

    def start_crazyflie_thread(self, scf, target):
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

    def start_key_listener(self):
        # Define the key listener callback
        def on_press(key):
            try:
                k = key.char
            except:
                k = key.name

            match k:
                case '1':
                    self.mode_select = 1
                    print(f"\n\n Switching into Phase 1: Dwell")
                case '2':
                    self.mode_select = 2
                    print(f"\n\n Switching into Phase 2: Rendesvous & Follow")
                case '3':
                    self.mode_select = 3
                    print(f"\n\n Switching into Phase 3: Detach")
                case '4':
                    self.mode_select = 4
                    print(f"\n\n Switching into Phase 4: Landing")
                case 'q':
                    print("Quit mode selected")
                    self.mode_select = 'q'
                case _:
                    pass

            self.new_state = True
            return None

        # Start the key listener
        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

    def initialize_async_logs(self):
        # Define the logging variables
        t_lg_vars = {'stateEstimate.x': 'float',
                     'stateEstimate.y': 'float',
                     'stateEstimate.z': 'float',
                     'stateEstimate.vx': 'float',
                     'stateEstimate.vy': 'float',
                     'stateEstimate.vz': 'float'}
        o_lg_vars = {'stabilizer.roll': 'float',
                     'stabilizer.pitch': 'float',
                     'stabilizer.yaw': 'float',
                     'stateEstimateZ.rateRoll': 'float',
                     'stateEstimateZ.ratePitch': 'float',
                     'stateEstimateZ.rateYaw': 'float'}

        # Define log configuration for translation and orientation for all crazyflies
        cf_trans_log_conf = LogConfig(name="t_cf", period_in_ms=50)
        for key in t_lg_vars:
            cf_trans_log_conf.add_variable(key, t_lg_vars[key])
        cf_orien_log_conf = LogConfig(name="o_cf", period_in_ms=50)
        for key in o_lg_vars:
            cf_orien_log_conf.add_variable(key, o_lg_vars[key])
        tb_trans_log_conf = LogConfig(name="t_tb", period_in_ms=50)
        for key in t_lg_vars:
            tb_trans_log_conf.add_variable(key, t_lg_vars[key])
        tb_orien_log_conf = LogConfig(name="o_tb", period_in_ms=50)
        for key in o_lg_vars:
            tb_orien_log_conf.add_variable(key, o_lg_vars[key])

        return cf_trans_log_conf, cf_orien_log_conf, tb_trans_log_conf, tb_orien_log_conf

    def cf_translation_callback(self, timestamp, data, logconf):
        self.cf_state[0:6] = np.array([data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z'],
                                       data['stateEstimate.vx'], data['stateEstimate.vy'], data['stateEstimate.vz']])

    def cf_orientation_callback(self, timestamp, data, logconf):
        self.cf_state[6:12] = np.array([data['stabilizer.roll'], data['stabilizer.pitch'], data['stabilizer.yaw'],
                                        data['stateEstimateZ.rateRoll'], data['stateEstimateZ.ratePitch'], data['stateEstimateZ.rateYaw']])

    def tb_translation_callback(self, timestamp, data, logconf):
        self.tb_state[0:6] = np.array([data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z'],
                                       data['stateEstimate.vx'], data['stateEstimate.vy'], data['stateEstimate.vz']])

    def tb_orientation_callback(self, timestamp, data, logconf):
        self.tb_state[6:12] = np.array([data['stabilizer.roll'], data['stabilizer.pitch'], data['stabilizer.yaw'],
                                        data['stateEstimateZ.rateRoll'], data['stateEstimateZ.ratePitch'], data['stateEstimateZ.rateYaw']])

if __name__ == '__main__':

    # Create dictionary of uris
    URI = dict()
    URI['tb'] = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E702')
    URI['cf'] = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E701')

    # Initialize the orchestrator
    refueling_orchestrator = RefuelingOrchestrator(URI,
                                 max_scenario_time=120, # seconds
                                 tether_length=0.30, # meters
                                 log_pos=True, # Set to True to log position and reference data
                                 log_rate=20, # Hz, rate to log position data
                                 simulate=False, # Set to True to simulate without connecting to Crazyflie
                                 verbose=True, # Set to True to print more information
                                 )
