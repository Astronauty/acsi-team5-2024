
#modified from Crazyfile_controller.py
import numpy as np
from controller import Robot
from controller import Keyboard

import math

#from pid_controller import pid_velocity_fixed_height_controller
from osqp_solver import QuadrotorLQR

N_MPC_HORIZON = 3 # number of steps to consider in MPC horizon
N_STATES = 12
N_CONTROLS = 4

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

    # Initialize variables
    past_time = 0
    first_time = True
    
    # Crazyflie OSQP controller
    OSQP_crazyfile = QuadrotorLQR(dt=timestep/1000)


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
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)
        roll_rate,pitch_rate,yaw_rate = gyro.getValues()
        roll_rate = math.degrees(roll_rate)
        pitch_rate = math.degrees(pitch_rate)
        yaw_rate = math.degrees(yaw_rate)
        x_global,y_global,z_global = gps.getValues()
        v_x_global,v_y_global,v_z_global = gps.getSpeedVector()
        

        OSQP_crazyfile.update_x0(x_global,y_global,z_global,v_x_global,v_y_global,v_z_global, roll,pitch,yaw,roll_rate,pitch_rate,yaw_rate)
        #print(OSQP_crazyfile.x0)
        #OSQP_crazyfile.x0 = np.array([x_global,y_global,z_global,v_x_global,v_y_global,v_z_global, roll,pitch,yaw,roll_rate,pitch_rate,yaw_rate]).reshape(12,1)
        
        #OSQP_crazyfile.x0[np.abs(OSQP_crazyfile.x0) < 0.000001] = 0
        #OSQP_crazyfile.x0 = np.array([0,0,0.014,0,0,0,0,0,0,0,0,0]).reshape(12,1)
        print(OSQP_crazyfile.x0)
        desired_state = np.zeros((12,1))
        desired_state[0] = 1
        desired_state[1] = 1
        desired_state[2] = 0
        #print(desired_state)
        res = OSQP_crazyfile.solve_linear_mpc(desired_state)

        motor_power = res.x
        
        #m3_motor.setVelocity(1000000)

        # Limit the motor command
        #m1 = np.clip(-motor_power[0], 0, 600)
        #m2 = np.clip(motor_power[1], 0, 600)
        #m3 = np.clip(-motor_power[2], 0, 600)
        #m4 = np.clip(motor_power[3], 0, 600)
        m1 = motor_power[0]
        m2 = -motor_power[1]
        m3 = motor_power[2]
        m4 = -motor_power[3]
        #print(m1,m2,m3,m4)
        #print("timestep",timestep)
        
        m1_motor.setVelocity(m1)
        m2_motor.setVelocity(m2)
        m3_motor.setVelocity(m3)
        m4_motor.setVelocity(m4)
        past_time = robot.getTime()
        