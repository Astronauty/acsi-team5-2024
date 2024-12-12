# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

import logging
import sys
import time
import math
from threading import Event
import matplotlib.pyplot as plt
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E701')

DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.5

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

position_estimate = [0,0,0]
position_log = []

class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0, output_limits=(-1, 1)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits

        self._integral = 0
        self._prev_error = None
        self._prev_time = None

    def compute(self, current_value):
        current_time = time.time()
        error = self.setpoint - current_value

        # Proportional term
        p = self.kp * error

        # Integral term
        self._integral += error * (current_time - self._prev_time if self._prev_time else 0)
        i = self.ki * self._integral

        # Derivative term
        d = 0
        if self._prev_error is not None:
            d = self.kd * (error - self._prev_error) / (current_time - self._prev_time)

        # Save for next iteration
        self._prev_error = error
        self._prev_time = current_time

        # Control signal
        output = p + i + d

        # Clamp output to limits
        output = max(self.output_limits[0], min(self.output_limits[1], output))
        return output


def run_sequence(scf, hover_time=10, ref_point=(0, 0, 1.5, 0)):
    cf = scf.cf

    x = ref_point[0]
    y = ref_point[1]
    z = ref_point[2]
    yaw = ref_point[3]

    print(f"Hovering at position {ref_point[:3]} for {hover_time} seconds...")
    start_time = time.time()

    while time.time() - start_time < hover_time:
        cf.commander.send_position_setpoint(x, y, z, yaw)
        time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

def hover_with_pid(scf, hover_time=20, reference_point=(0, 0, DEFAULT_HEIGHT, 0)):
    """
    Hover with PID control to maintain position and yaw.
    :param scf: SyncCrazyflie object
    :param hover_time: Duration to hover (in seconds)
    :param reference_point: Desired [x, y, z, yaw] (yaw in degrees)
    """
    # Initialize PID controllers for x, y, z, and yaw
    pid_x = PIDController(kp=1.0, ki=0.0, kd=0.1, setpoint=reference_point[0])
    pid_y = PIDController(kp=1.0, ki=0.0, kd=0.1, setpoint=reference_point[1])
    pid_z = PIDController(kp=1.0, ki=0.0, kd=0.1, setpoint=reference_point[2])
    pid_yaw = PIDController(kp=1.0, ki=0.0, kd=0.1, setpoint=reference_point[3])  # Yaw in degrees

    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        print("Taking off...")
        time.sleep(3)  # Initial takeoff time

        print(f"Hovering at position {reference_point} for {hover_time} seconds...")
        start_time = time.time()

        while time.time() - start_time < hover_time:
            # Calculate control efforts using PID
            control_x = pid_x.compute(position_estimate[0])
            control_y = pid_y.compute(position_estimate[1])
            control_z = pid_z.compute(position_estimate[2])
            control_yaw = pid_yaw.compute(0)  # Assuming yaw can be estimated (set as 0 if not available)

            # Send position and yaw control commands
            mc.start_position_setpoint(control_x, control_y, control_z, control_yaw)

            # Small delay to avoid overloading the Crazyflie
            time.sleep(0.1)

        print("Landing...")
        mc.stop()



def move_box_limit(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        body_x_cmd = 0.2
        body_y_cmd = 0.1
        max_vel = 0.2

        while (1):
            '''if position_estimate[0] > BOX_LIMIT:
                mc.start_back()
            elif position_estimate[0] < -BOX_LIMIT:
                mc.start_forward()
            '''

            if position_estimate[0] > BOX_LIMIT:
                body_x_cmd = -max_vel
            elif position_estimate[0] < -BOX_LIMIT:
                body_x_cmd = max_vel
            if position_estimate[1] > BOX_LIMIT:
                body_y_cmd = -max_vel
            elif position_estimate[1] < -BOX_LIMIT:
                body_y_cmd = max_vel

            mc.start_linear_motion(body_x_cmd, body_y_cmd, 0)

            time.sleep(0.1)


def move_linear_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)
        mc.turn_left(180)
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)


def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        mc.stop()
        
def take_off_mpc(scf):
    scf.cf.commander.send_position_setpoint(0, 0, 0.25, 0)

def take_off_and_hover(scf, hover_time=10):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        print("Taking off...")
        time.sleep(3)  # Initial takeoff time

        print(f"Hovering at position for {hover_time} seconds...")
        start_time = time.time()

        while time.time() - start_time < hover_time:
            # Issue a zero-velocity command to maintain position
            mc.start_linear_motion(0, 0, 0)
            time.sleep(0.1)  # Small delay to prevent overwhelming the Crazyflie

        print("Landing...")
        mc.stop()


def log_pos_callback(timestamp, data, logconf):
    print(data)
    global position_estimate, position_log
    position_estimate = [data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z'], data['stateEstimate.yaw']]
    position_log.append(position_estimate)
    #position_estimate[0] = data['stateEstimate.x']
    #position_estimate[1] = data['stateEstimate.y']
    #position_estimate[2] = data['stateEstimate.z']


def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


def reset_estimator(scf):
    """Reset and wait for the position estimator to initialize."""
    print('Resetting position estimator...')
    scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    scf.cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(scf)

def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

def wait_for_position_estimator(scf):
    """Wait for the Kalman filter to stabilize."""
    print('Waiting for position estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_history = {'x': [1000] * 10, 'y': [1000] * 10, 'z': [1000] * 10}
    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]
            var_history['x'].append(data['kalman.varPX'])
            var_history['y'].append(data['kalman.varPY'])
            var_history['z'].append(data['kalman.varPZ'])

            var_history['x'].pop(0)
            var_history['y'].pop(0)
            var_history['z'].pop(0)

            if all(
                max(var_history[axis]) - min(var_history[axis]) < threshold
                for axis in var_history
            ):
                print('Position estimator ready.')
                break

def init_pos_log(scf):
    logconf = LogConfig(name='Position', period_in_ms=10)
    logconf.add_variable('stateEstimate.x', 'float')
    logconf.add_variable('stateEstimate.y', 'float')
    logconf.add_variable('stateEstimate.z', 'float')
    logconf.add_variable('stateEstimate.yaw', 'float')
    scf.cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_pos_callback)
    logconf.start()

    return logconf


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        #scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
        #                                 cb=param_deck_flow)
        time.sleep(1)
        scf.cf.param.set_value('stabilizer.controller', 6)
        

        logconf = init_pos_log(scf)

        # set_initial_position(scf, 1, 0, 0, 0)
        # reset_estimator(scf)
        #if not deck_attached_event.wait(timeout=5):
        #    print('No flow deck detected!')
        #    sys.exit(1)

        #logconf.start()
        #temp
        start = time.time()
        
        while(time.time() < start + 2.0):
            take_off_mpc(scf)
        
        
        #hover_with_pid(scf, hover_time=10, reference_point=(0, 0, 1.5))
        # run_sequence(scf, hover_time=10)
        # move_linear_simple(scf)
        # move_box_limit(scf)
        logconf.stop()

    # Write the xyz data to a csv file
    np.savetxt('position_log.csv', np.array(position_log), delimiter=',')
    print('Data saved to position_estimate.csv')

