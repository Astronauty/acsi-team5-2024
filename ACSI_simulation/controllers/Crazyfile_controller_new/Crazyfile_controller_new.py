# -*- coding: utf-8 -*-
#
#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  MIT Licence
#
#  Copyright (C) 2023 Bitcraze AB
#

"""
file: Crazyfile_controller.py

"""

from controller import Robot
from controller import Keyboard

import numpy as np
import math
from math import cos, sin

from pid_controller import pid_velocity_fixed_height_controller
from osqp_solver import QuadrotorLQR

N_MPC_HORIZON = 3 # number of steps to consider in MPC horizon
N_STATES = 12
N_CONTROLS = 4
FLYING_ATTITUDE = 0.5


"""
def process_received_data(receiver):
    #Process received data from the emitter.
    if receiver.getQueueLength() > 0:  # Check if there's data in the queue
        data = receiver.getBytes()  # Get the raw data (bytes)
        receiver.nextPacket()  # Move to the next packet in the queue

        # Convert bytes back to numpy array
        state = np.frombuffer(data, dtype=float).reshape(12, 1)
        print("receiver get",state)
        return state
    return None  # No data received

def process_received_data(receiver):
    if receiver.getQueueLength() > 0:  # Check if there's data in the queue
        data_string = receiver.getString()  # Get the string data
        data_bytes = data_string.encode('latin1')  # Convert the string back to bytes
        state = np.frombuffer(data_bytes, dtype=float).reshape(12, 1)  # Convert bytes to numpy array
        print(state)
        return state
    return None  # No data received
"""
def process_received_data(receiver):
    """Process received data from the emitter."""
    if receiver.getQueueLength() > 0:  # Check if there's data in the queue
        data_string = receiver.getString()  # Get the string data
        receiver.nextPacket()  # Move to the next packet in the queue
        data_bytes = data_string.encode('latin1')  # Convert the string back to bytes
        state = np.frombuffer(data_bytes, dtype=float).reshape(12, 1)  # Convert bytes to numpy array
        #print("Updated state:", state)
        return state
    return None  # No data received

def calculate_f(U1, U2, U3, U4):
    mg = 55.368#55.9
    d = 0.006#0.003183#0.009 #meters
    c = 0.1#0.009 #don't know

    # Coefficient matrix
    #A = np.array([
    #    [1, 1, 1, 1],
    #    [0, 1, 0, -1],
    #    [1, 0, -1, 0],
    #    [-1, 1, -1, 1]
    #])
    A = np.array([
        [1, 1, 1, 1],
        [-1, -1, 1, 1],
        [-1, 1, 1, -1],
        [1, -1, 1, -1]
    ])
    
    # Right-hand side vector
    U = np.array([U1+mg, U2/d, U3/d, U4/c])
    
    # Solve the system of linear equations
    f = np.linalg.solve(A, U)
    
    # Return the values of f1, f2, f3, f4
    return f

if __name__ == '__main__':

    robot = Robot()
    timestep = int(robot.getBasicTimeStep()) #in ms
    
    # Initialize motors
    m1_motor = robot.getDevice("m1_motor")
    m1_motor.setPosition(float('inf'))
    m1_motor.setVelocity(-1)
    m2_motor = robot.getDevice("m2_motor")
    m2_motor.setPosition(float('inf'))
    m2_motor.setVelocity(1)
    m3_motor = robot.getDevice("m3_motor")
    m3_motor.setPosition(float('inf'))
    m3_motor.setVelocity(-1)
    m4_motor = robot.getDevice("m4_motor")
    m4_motor.setPosition(float('inf'))
    m4_motor.setVelocity(1)

    # Initialize Sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)
    #receiver.setChannel(1)
    #camera = robot.getDevice("camera")
    #camera.enable(timestep)
    #range_front = robot.getDevice("range_front")
    #range_front.enable(timestep)
    #range_left = robot.getDevice("range_left")
    #range_left.enable(timestep)
    #range_back = robot.getDevice("range_back")
    #range_back.enable(timestep)
    #range_right = robot.getDevice("range_right")
    #range_right.enable(timestep)

    # Get keyboard
    keyboard = Keyboard()
    keyboard.enable(timestep)

    # Initialize variables

    #past_x_global = 0
    #past_y_global = 0
    past_time = 0
    first_time = True

    # Crazyflie velocity PID controller
    PID_crazyflie = pid_velocity_fixed_height_controller()
    PID_update_last_time = robot.getTime()
    sensor_read_last_time = robot.getTime()

    height_desired = FLYING_ATTITUDE

    # Crazyflie OSQP controller
    OSQP_crazyfile = QuadrotorLQR(dt=timestep/1000)

    #wall_following = WallFollowing(angle_value_buffer=0.01, reference_distance_from_wall=0.5,
    #                               max_forward_speed=0.3, init_state=WallFollowing.StateWallFollowing.FORWARD)

    autonomous_mode = False

    print("\n")

    print("====== Controls =======\n\n")

    print(" The Crazyflie can be controlled from your keyboard!\n")
    print(" All controllable movement is in body coordinates\n")
    print("- Use the up, back, right and left button to move in the horizontal plane\n")
    print("- Use Q and E to rotate around yaw\n ")
    print("- Use W and S to go up and down\n ")
    print("- Press A to start autonomous mode\n")
    print("- Press D to disable autonomous mode\n")

    # Main loop:
    while robot.step(timestep) != -1:

        dt = robot.getTime() - past_time
        actual_state = {}

        if first_time:
            past_x_global = gps.getValues()[0]
            past_y_global = gps.getValues()[1]
            past_z_global = gps.getValues()[2]
            past_time = robot.getTime()
            first_time = False

        # Get sensor data
        roll,pitch,yaw = imu.getRollPitchYaw()
        roll_rate,pitch_rate,yaw_rate = gyro.getValues()
        x_global,y_global,z_global = gps.getValues()
        v_x_global,v_y_global,v_z_global = gps.getSpeedVector()
        yaw1 = yaw+135/180*math.pi

        # Get body fixed velocities
        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)
        v_x = v_x_global * cos_yaw + v_y_global * sin_yaw
        v_y = - v_x_global * sin_yaw + v_y_global * cos_yaw

        # Initialize values
        desired_state = [0, 0, 0, 0]
        forward_desired = 0
        sideways_desired = 0
        yaw_desired = 0
        height_diff_desired = 0

        key = keyboard.getKey()
        while key > 0:
            if key == Keyboard.UP:
                forward_desired += 0.5
            elif key == Keyboard.DOWN:
                forward_desired -= 0.5
            elif key == Keyboard.RIGHT:
                sideways_desired -= 0.5
            elif key == Keyboard.LEFT:
                sideways_desired += 0.5
            elif key == ord('Q'):
                yaw_desired = + 1
            elif key == ord('E'):
                yaw_desired = - 1
            elif key == ord('W'):
                height_diff_desired = 0.1
            elif key == ord('S'):
                height_diff_desired = - 0.1
            elif key == ord('A'):
                if autonomous_mode is False:
                    autonomous_mode = True
                    print("Autonomous mode: ON")
            elif key == ord('D'):
                if autonomous_mode is True:
                    autonomous_mode = False
                    print("Autonomous mode: OFF")
            key = keyboard.getKey()

        height_desired += height_diff_desired * dt


        OSQP_crazyfile.update_x0(x_global,y_global,z_global,v_x_global,v_y_global,v_z_global, roll,pitch,yaw,roll_rate,pitch_rate,yaw_rate)

        desired_state = np.zeros((12,1))
        #print(desired_state)
        #receive T state
        T_state = process_received_data(receiver)
        
        desired_state = np.zeros((12,1))

        if np.any(T_state) != None:
            print("received state",T_state)
            T_state = np.copy(T_state) #copy read only array
            desired_state[:6] = T_state[:6]
            desired_state[8] = T_state[8]

        else:
            print("didn't received T states",desired_state)
        
        ##########################################
        #test code for pure MPC takeoff
        
        #desired_state = np.zeros((12,1))
        #desired_state[0] = 0
        #desired_state[1] = -1.0
        #desired_state[2] = FLYING_ATTITUDE#1
        #desired_state[8] = 0 #+= 135/180*math.pi

        
        ##########################################
        desired_state[2] += 0.35
        #print(desired_state)

        res = OSQP_crazyfile.solve_linear_mpc(desired_state)


        motor_power = res.x*1
        U1 = motor_power[0]*1E1
        U2 = motor_power[1]*1E1
        U3 = motor_power[2]*1E1
        U4 = motor_power[3]*1E1


        f=calculate_f(U1,U2,U3,U4)
        #print("f",f)

        # PID velocity controller with fixed height
        motor_power = PID_crazyflie.pid(dt, forward_desired, sideways_desired,
                                        yaw_desired, height_desired,
                                        roll, pitch, yaw_rate,
                                        z_global, v_x, v_y)
        #print(motor_power)
        
        if past_time <= 5:
            m1_motor.setVelocity(-motor_power[0])
            m2_motor.setVelocity(motor_power[1])
            m3_motor.setVelocity(-motor_power[2])
            m4_motor.setVelocity(motor_power[3])
            print("g speeds",[-motor_power[0],motor_power[1],-motor_power[2],motor_power[3]])
        else:
            print("start MPC")
            scale = 0.4
            m1_motor.setVelocity(-motor_power[0]-f[0]*scale)
            m2_motor.setVelocity(motor_power[1]+f[1]*scale)
            m3_motor.setVelocity(-motor_power[2]-f[2]*scale)
            m4_motor.setVelocity(motor_power[3]+f[3]*scale)
            #scale = 4
            #m1_motor.setVelocity(-f[0]*scale)
            #m2_motor.setVelocity(f[1]*scale)
            #m3_motor.setVelocity(-f[2]*scale)
            #m4_motor.setVelocity(f[3]*scale)
            print([f[0]*scale,f[1]*scale,f[2]*scale,f[3]*scale])
            print("PID",[-motor_power[0],motor_power[1],-motor_power[2],motor_power[3]])
            
            #scale = 50
        """
        scale = 4
        m1_motor.setVelocity(-f[0]*scale)
        m2_motor.setVelocity(f[1]*scale)
        m3_motor.setVelocity(-f[2]*scale)
        m4_motor.setVelocity(f[3]*scale)
        print([f[0]*scale,f[1]*scale,f[2]*scale,f[3]*scale])
        """

        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global