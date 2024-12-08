import logging
import threading
import sys
import os
import time
import numpy as np
import scipy as sp

from threading import Event, Thread
from pynput import keyboard

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie import Commander
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.swarm import Swarm

from cflib.utils import uri_helper
from scipy import signal
from scipy import sparse

URI = dict()
URI['cf'] = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E701')
URI['tb'] = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E702')

logging.basicConfig(level=logging.WARNING, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
stop_event = Event()


class RefuelingOrchestrator():
    def __init__(self, URI, max_scenario_time, verbose=False):
        assert URI['cf'] is not None, "CrazyFlie URI not defined"
        assert URI['tb'] is not None, "Tumbller URI is not defined"

        ## Params
        self.tether_length = 0.2
        self.log_cf_pos = False
        self.log_events = False
        self.new_state = False

        self.URI = URI
        self.max_scenario_time = max_scenario_time

        self.mode_select = 0  # Phase 0 - Init & No Controls, 1 - Dwell, 2 - Rendesvous & Follow, 3 - Detach

        self.cf_state = np.zeros(12)
        self.tb_state = np.zeros(12)

        self.cf_reference_pos = np.zeros(6)

        threads = []

        cflib.crtp.init_drivers()

        # Initialize key listener in separate thread
        listener_thread = Thread(target=self.start_key_listener)
        listener_thread.daemon = True
        listener_thread.start()

        with Swarm(list(URI.values())) as swarm:
            scfs = swarm._cfs  # Crazyflie instances

            self.cf_translation_log_conf, self.cf_orientation_log_conf, self.tb_translation_log_conf, self.tb_orientation_log_conf = self.initialize_log_configs()
            try:
                self.start_time = time.time()
                self.end_time = self.start_time + self.max_scenario_time

                # self.run_cf(scfs[URI['cf']])
                # self.run_tb(scfs[URI['tb']])
                if URI['cf'] in scfs:
                    threads.append(self.start_crazyflie_thread(scfs[URI['cf']], self.run_cf))
                    print("Successfully started cf thread.")
                # if URI['tb'] in scfs:
                #     threads.append(self.start_crazyflie_thread(scfs[URI['tb']], self.run_tb))
                #     print("Successfully started tb thread.")

                # Wait for all threads to finish
                for thread in threads:
                    thread.join()
                    # thread.join(timeout=5)
                    if thread.is_alive():
                        logger.info("Thread did not finish in time (5 seconds).")
            except KeyboardInterrupt:
                logger.info("Keyboard interrupt detected. Stopping threads")
                stop_event.set()  # Signal all threads to stop

        return None

    def write_to_event_log(self, event):
        if self.log_events:
            if os.path.exists('event_log.txt'):
                with open('event_log.txt', 'a') as f:
                    f.write(f"{event}\n")
            else:
                with open('event_log.txt', 'w') as f:
                    f.write(f"{event}\n")


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
        scf.cf.log.add_config(self.cf_orientation_log_conf)

        self.cf_translation_log_conf.data_received_cb.add_callback(self.cf_translation_callback)
        self.cf_orientation_log_conf.data_received_cb.add_callback(self.cf_orientation_callback)
        self.cf_translation_log_conf.start()
        self.cf_orientation_log_conf.start()

        logger.info(f"Started logging for {self.URI['cf']}.")
        print("CF initialization complete!")

        try:
            # Commander.send_position_setpoint(self, 0, 0, 0.2, 0)
            # start = time.time()

            while time.time() < self.end_time:
                match self.mode_select:
                    case 1:
                        scf.cf.commander.send_position_setpoint(0, 0, 0.5, 0)
                        if self.new_state and self.log_events:
                            print(f"CF State 1")
                            self.new_state = False

                    case 2:
                        # update the cf_reference to be the position of the tb plus tether length
                        self.cf_reference_pos = self.tb_state[0:3] + np.array([0, 0, self.tether_length])
                        x_ref = self.cf_reference_pos[0]
                        y_ref = self.cf_reference_pos[1]
                        z_ref = self.cf_reference_pos[2]
                        scf.cf.commander.send_position_setpoint(x_ref, y_ref, z_ref, 0)

                    case 3:
                        scf.cf.commander.send_position_setpoint(-0.25, 0, 0.5, 0)

                    case 4:
                        scf.cf.commander.send_position_setpoint(0, 0, 0.1, 0)
                        time.sleep(3)
                        break

                    case _:
                        pass

                if self.log_cf_pos:

                time.sleep(0.1)

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
                case _:
                    pass

            self.new_state = True
            return None

        # Start the key listener
        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

    def initialize_log_configs(self):
        # Crazyflie log configs
        cf_translation_log_conf = LogConfig(name='t_cf', period_in_ms=50)
        cf_translation_log_conf.add_variable('stateEstimate.x', 'float')
        cf_translation_log_conf.add_variable('stateEstimate.y', 'float')
        cf_translation_log_conf.add_variable('stateEstimate.z', 'float')
        cf_translation_log_conf.add_variable('stateEstimate.vx', 'float')
        cf_translation_log_conf.add_variable('stateEstimate.vy', 'float')
        cf_translation_log_conf.add_variable('stateEstimate.vz', 'float')

        cf_orientation_log_conf = LogConfig(name='o_cf', period_in_ms=50)
        cf_orientation_log_conf.add_variable('stabilizer.roll', 'float')
        cf_orientation_log_conf.add_variable('stabilizer.pitch', 'float')
        cf_orientation_log_conf.add_variable('stabilizer.yaw', 'float')
        cf_orientation_log_conf.add_variable('stateEstimateZ.rateRoll', 'int16_t')
        cf_orientation_log_conf.add_variable('stateEstimateZ.ratePitch', 'int16_t')
        cf_orientation_log_conf.add_variable('stateEstimateZ.rateYaw', 'int16_t')

        # Tumbller log configs
        tb_translation_log_conf = LogConfig(name='t_tb', period_in_ms=50)
        tb_translation_log_conf.add_variable('stateEstimate.x', 'float')
        tb_translation_log_conf.add_variable('stateEstimate.y', 'float')
        tb_translation_log_conf.add_variable('stateEstimate.z', 'float')
        tb_translation_log_conf.add_variable('stateEstimate.vx', 'float')
        tb_translation_log_conf.add_variable('stateEstimate.vy', 'float')
        tb_translation_log_conf.add_variable('stateEstimate.vz', 'float')

        tb_orientation_log_conf = LogConfig(name='o_tb', period_in_ms=50)
        tb_orientation_log_conf.add_variable('stabilizer.roll', 'float')
        tb_orientation_log_conf.add_variable('stabilizer.pitch', 'float')
        tb_orientation_log_conf.add_variable('stabilizer.yaw', 'float')
        tb_orientation_log_conf.add_variable('stateEstimateZ.rateRoll', 'int16_t')
        tb_orientation_log_conf.add_variable('stateEstimateZ.ratePitch', 'int16_t')
        tb_orientation_log_conf.add_variable('stateEstimateZ.rateYaw', 'int16_t')
        return cf_translation_log_conf, cf_orientation_log_conf, tb_translation_log_conf, tb_orientation_log_conf

    def cf_translation_callback(self, timestamp, data, logconf):
        x = data['stateEstimate.x']
        y = data['stateEstimate.y']
        z = data['stateEstimate.z']
        vx = data['stateEstimate.vx']
        vy = data['stateEstimate.vy']
        vz = data['stateEstimate.vz']

        self.cf_state[0:6] = np.array([x, y, z, vx, vy, vz])

    def cf_orientation_callback(self, timestamp, data, logconf):
        roll = data['stabilizer.roll']
        pitch = data['stabilizer.pitch']
        yaw = data['stabilizer.yaw']
        vroll = data['stateEstimateZ.rateRoll']
        vpitch = data['stateEstimateZ.ratePitch']
        vyaw = data['stateEstimateZ.rateYaw']

        self.cf_state[6:12] = np.array([roll, pitch, yaw, vroll, vpitch, vyaw])

    def tb_translation_callback(self, timestamp, data, logconf):
        print('hello tb')
        x = data['stateEstimate.x']
        y = data['stateEstimate.y']
        z = data['stateEstimate.z']
        vx = data['stateEstimate.vx']
        vy = data['stateEstimate.vy']
        vz = data['stateEstimate.vz']

        self.tb_state[0:6] = np.array([x, y, z, vx, vy, vz])

    def tb_orientation_callback(self, timestamp, data, logconf):
        roll = data['stabilizer.roll']
        pitch = data['stabilizer.pitch']
        yaw = data['stabilizer.yaw']
        vroll = data['stateEstimateZ.rateRoll']
        vpitch = data['stateEstimateZ.ratePitch']
        vyaw = data['stateEstimateZ.rateYaw']

        self.cf_state[6:12] = np.array([roll, pitch, yaw, vroll, vpitch, vyaw])


if __name__ == '__main__':
    refueling_orchestrator = RefuelingOrchestrator(URI, max_scenario_time=120, verbose=False)
