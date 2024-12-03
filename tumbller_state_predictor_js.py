import logging
import time
import numpy as np

from cflib.crtp import init_drivers
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

logging.basicConfig(level=logging.ERROR)

URI = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E702')


class TumbllerStatePredictor:
    def __init__(self, prediction_horizon_time, sampling_rate=20):
        self.tb_state = np.zeros(12)  # Initialize state vector
        self.sampling_rate = sampling_rate
        self.prev_future_state = np.zeros(6) # For debugging the future state values
        self.prediction_horizon_time = prediction_horizon_time

        # Define log configuration for translation
        self.t_log_conf = LogConfig(name="Translation", period_in_ms=50)
        self.t_log_conf.add_variable("stateEstimate.x", "float")
        self.t_log_conf.add_variable("stateEstimate.y", "float")
        self.t_log_conf.add_variable("stateEstimate.z", "float")
        self.t_log_conf.add_variable("stateEstimate.vx", "float")
        self.t_log_conf.add_variable("stateEstimate.vy", "float")
        self.t_log_conf.add_variable("stateEstimate.vz", "float")

        # Define log configuration for orientation
        self.o_log_conf = LogConfig(name="Orientation", period_in_ms=50)
        self.o_log_conf.add_variable("stabilizer.roll", "float")
        self.o_log_conf.add_variable("stabilizer.pitch", "float")
        self.o_log_conf.add_variable("stabilizer.yaw", "float")
        self.o_log_conf.add_variable("stateEstimateZ.rateRoll", "float")
        self.o_log_conf.add_variable("stateEstimateZ.ratePitch", "float")
        self.o_log_conf.add_variable("stateEstimateZ.rateYaw", "float")

        # Connect to Crazyflie using SyncCrazyflie
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
            print("Connected to Crazyflie.")

            # Add log configuration to Crazyflie
            scf.cf.log.add_config(self.t_log_conf)
            scf.cf.log.add_config(self.o_log_conf)

            if self.t_log_conf.valid and self.o_log_conf.valid:
                # Start logging translation data
                self.t_log_conf.data_received_cb.add_callback(self.translation_state_est_callback)
                self.t_log_conf.start()

                # Start logging orientation data
                self.o_log_conf.data_received_cb.add_callback(self.orientation_state_est_callback)
                self.o_log_conf.start()

                print("Logging translation and orientation data...")

                # Let the logging run for some time
                #time.sleep(100)

                # Run a while loop which calls the tumbller_state_prediction function
                while True:
                    future_state = self.tumbller_state_prediction(self.prediction_horizon_time)

                    # print each element up to the 3rd decimal place
                    future_state = np.round(future_state, 3)
                    #print(f"Future state: {future_state}")

                    cur_state = np.concatenate([
                        np.round(self.tb_state[0:3], 3),  # Position
                        np.round(self.tb_state[6:9], 3)  # Orientation
                    ])
                    #print(f"Current state: {cur_state}")

                    # Show the difference between the current and future state
                    diff = np.round(cur_state - self.prev_future_state, 3)
                    print(f"Difference: {diff}")

                    self.prev_future_state = future_state
                    time.sleep(1/self.sampling_rate)
                self.t_log_conf.stop()
            else:
                print("Log configuration is not valid.")

    def translation_state_est_callback(self, timestamp, data, logconf):
        # Update the state with received data
        x = data["stateEstimate.x"]
        y = data["stateEstimate.y"]
        z = data["stateEstimate.z"]
        vx = data["stateEstimate.vx"]
        vy = data["stateEstimate.vy"]
        vz = data["stateEstimate.vz"]
        self.tb_state[0:6] = np.array([x, y, z, vx, vy, vz])
        trans_state = np.round(self.tb_state[0:6], 3)
        #print(f"State updated: {trans_state}")  # Print updated state

    def orientation_state_est_callback(self, timestamp, data, logconf):
        # Update orientation state with received data
        roll = data["stabilizer.roll"]
        pitch = data["stabilizer.pitch"]
        yaw = data["stabilizer.yaw"]
        rate_roll = data["stateEstimateZ.rateRoll"]
        rate_pitch = data["stateEstimateZ.ratePitch"]
        rate_yaw = data["stateEstimateZ.rateYaw"]
        self.tb_state[6:12] = np.array([roll, pitch, yaw, rate_roll, rate_pitch, rate_yaw])
        orient_state = np.round(self.tb_state[6:12], 3)
        #print(f"Orientation state updated: {orient_state}")

    def tumbller_state_prediction(self, t_pred):
        # Predict future state based on current translation and orientation
        trans_pos = self.tb_state[0:3]
        trans_vel = self.tb_state[3:6]
        orient_pos = self.tb_state[6:9]
        orient_vel = self.tb_state[9:12]

        future_trans_pos = trans_pos + t_pred * trans_vel
        future_orient_pos = orient_pos + t_pred * orient_vel

        # Combine predicted translation and orientation states
        return np.concatenate([future_trans_pos, future_orient_pos])


if __name__ == "__main__":
    try:
        # Initialize drivers
        init_drivers()

        # Initialize the TumbllerStatePredictor
        tumbller_state_predictor = TumbllerStatePredictor(0.1)

    except KeyboardInterrupt:
        print("Interrupted by user.")

