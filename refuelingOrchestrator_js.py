import logging
import os
import time
import datetime
import numpy as np
from threading import Event, Thread
from pynput import keyboard

from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
from cflib.crtp import init_drivers
from cflib.crazyflie.swarm import Swarm, CachedCfFactory


logging.basicConfig(level=logging.WARNING, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
stop_event = Event()

class RefuelingOrchestrator():
    def __init__(self, URI, max_scenario_time, simulate=False, verbose=False):
        # Check if URIs are defined
        if 'tb' not in URI:
            logger.error("Warning: CrazyFlie tumbller URI not defined")
        #assert URI['cf'] is not None, "CrazyFlie URI not defined"
        #assert URI['tb'] is not None, "Tumbller URI is not defined"

        ## Params
        self.tether_length = 0.2
        self.rotate_mode = False
        self.URI = URI
        self.max_scenario_time = max_scenario_time
        self.simulate = simulate
        self.verbose = verbose

        # Create the position log file
        data = "time, x, y, z, vx, vy, vz, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate"
        self.write_to_pos_log('cf_pos_log.txt', data, new_file=True)

        self.mode_select = 0  # Phase 0 - Init & No Controls, 1 - Dwell, 2 - Rendesvous & Follow, 3 - Detach

        self.cf_state = np.zeros(12)
        self.tb_state = np.zeros(12)
        self.cf_reference_pos = np.zeros(12)

        threads = []

        #cflib.crtp.init_drivers()
        init_drivers(enable_debug_driver=False)  # initialize drivers
        factory = CachedCfFactory(rw_cache='./cache')  # For reducing connection time

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

                if URI['cf'] in scfs:
                    threads.append(self.start_crazyflie_thread(scfs[URI['cf']], self.run_cf))
                    print("Successfully started cf thread.")

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
                swarm.parallel_safe(self.stop_all)

    def move_drone(self, scf, x, y, z, yaw):
        if not self.simulate:
            if self.verbose:
                print(f"Setting drone to position: {x}, {y}, {z}, {yaw}")
            scf.cf.commander.send_position_setpoint(x, y, z, yaw)

    def write_to_event_log(self, event):
        # Helper function to write to event log
        if self.log_events:
            if os.path.exists('event_log.txt'):
                with open('event_log.txt', 'a') as f:
                    f.write(f"{event}\n")
            else:
                with open('event_log.txt', 'w') as f:
                    f.write(f"{event}\n")

    def write_to_pos_log(self, filename, data, new_file=False):
        # Helper function to write data to the passed filename
        if not os.path.exists(filename) or new_file:
            with open(filename, 'w') as f:
                f.write(f"{data}\n")
        else:
            with open(filename, 'a') as f:
                f.write(f"{data}\n")

    def stop_all(self, scf):
        # Function to stop all crazyflies
        scf.cf.commander.send_stop_setpoint()
        # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
        scf.cf.commander.send_notify_setpoint_stop()
        time.sleep(0.1)

    def run_tb(self, scf):
        # Add log configs and callbacks to the tb instance
        scf.cf.log.add_config(self.tb_translation_log_conf)
        scf.cf.log.add_config(self.tb_orientation_log_conf)

        self.tb_translation_log_conf.data_received_cb.add_callback(self.tb_translation_callback)
        self.tb_orientation_log_conf.data_received_cb.add_callback(self.tb_orientation_callback)
        self.tb_translation_log_conf.start()
        self.tb_orientation_log_conf.start()

        logger.info(f"Started logging for {self.URI['tb']}.")
        print("TB initialization complete!")

        # Keep logging running until the stop event is set
        try:
            while not stop_event.is_set():
                time.sleep(0.1)
        except KeyboardInterrupt:
            logger.info(f"Keyboard interrupt detected for {self.URI['tb']}.")
        finally:
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
            while time.time() < self.end_time:
                match self.mode_select:
                    case 1:
                        self.move_drone(scf,0, 0, 0.5, 0)
                        timestamp = time.strftime("%H:%M:%S", time.localtime())
                        #self.write_to_event_log(f"{timestamp},Phase 1: Dwell")

                    case 2:
                        # update the cf_reference to be the position of the tb plus tether length
                        self.cf_reference_pos = self.tb_state[0:3] + np.array([0, 0, self.tether_length])
                        x_ref = self.cf_reference_pos[0]
                        y_ref = self.cf_reference_pos[1]
                        z_ref = self.cf_reference_pos[2]
                        self.move_drone(scf, x_ref, y_ref, z_ref, 0)
                        #timestamp = time.strftime("%H:%M:%S", time.localtime())
                        #self.write_to_event_log(f"{timestamp},Phase 2: Tether")

                    case 3:
                        self.move_drone(scf, -0.25, 0, 0.5, 0)
                        #timestamp = time.strftime("%H:%M:%S", time.localtime())
                        #self.write_to_event_log(f"{timestamp},Phase 3: Detach")

                    case 4:
                        self.move_drone(scf, 0, 0, 0.1, 0)
                        #timestamp = time.strftime("%H:%M:%S", time.localtime())
                        #self.write_to_event_log(f"{timestamp} - Phase 4: Land")
                        time.sleep(3)
                        break

                    case 'forward':
                        print("Moving forward")
                        x, y, z = self.cf_state[0:3]
                        print(f"Current position: {x}, {y}, {z}")
                        self.move_drone(scf, x + 0.25, y, z, 0)
                        print(f"New position: {x + 0.25}, {y}, {z}")

                    case 'backward':
                        x, y, z = self.cf_state[0:3]
                        self.move_drone(scf, x - 0.25, y, z, 0)

                    case 'left':
                        x, y, z = self.cf_state[0:3]
                        self.move_drone( scf, x, y - 0.25, z, 0)

                    case 'right':
                        x, y, z = self.cf_state[0:3]
                        self.move_drone(scf, x, y + 0.25, z, 0)

                    case 'q':
                        print("Quitting")
                        break

                    case _:
                        pass

                # Get the timestamp, including milliseconds
                timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")
                data = [timestamp] + self.cf_state.tolist() # Fill in the data
                self.write_to_pos_log('cf_pos_log.txt', data)

                # Wait for a bit, rate update
                time.sleep(0.05)


            # When out of the while loop, attempt to stop the Crazyflie
            scf.cf.commander.send_stop_setpoint()
            scf.cf.commander.send_notify_setpoint_stop()
            time.sleep(0.1)

        except KeyboardInterrupt:
            logger.info(f"Keyboard interrupt detected for {self.URI['cf']}.")
            scf.cf.commander.send_stop_setpoint()
            scf.cf.commander.send_notify_setpoint_stop()
            time.sleep(0.1)

        finally:
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
                case 'r':
                    self.mode_select = 'record'
                    self.log_cf_pose = True
                    print(f"\n\n Recording CF position")
                case 's':
                    self.mode_select = 'stop'
                    self.log_cf_pose = False
                    print(f"\n\n Stopped recording CF position")
                case 'up':
                    self.mode_select = 'forward'
                    print(f"\n\n Moving forward, step 0.25m")
                case 'down':
                    self.mode_select = 'backward'
                    print(f"\n\n Moving backward, step 0.25m")
                case 'left':
                    self.mode_select = 'left'
                    print(f"\n\n Moving left, step 0.25m")
                case 'right':
                    self.mode_select = 'right'
                    print(f"\n\n Moving right, step 0.25m")
                case 'o':
                    self.rotate_mode = True
                    print(f"\n\n Rotating mode enabled")
                case 't':
                    self.rotate_mode = False
                    print(f"\n\n Rotating mode disabled")
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
        self.cf_state[6:12] = np.array([data['stabilizer.roll'], data['stabilizer.pitch'], data['stabilizer.yaw'],
                                        data['stateEstimateZ.rateRoll'], data['stateEstimateZ.ratePitch'], data['stateEstimateZ.rateYaw']])

if __name__ == '__main__':

    # Create dictionary of uris
    URI = dict()
    #URI['tb'] = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E701')
    URI['cf'] = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E702')

    # Initialize the orchestrator
    refueling_orchestrator = RefuelingOrchestrator(URI,
                                                   max_scenario_time=120,
                                                   simulate=False,
                                                   verbose=False)
