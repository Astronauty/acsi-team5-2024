
#modified from Crazyfile_controller.py
import numpy as np
from controller import Robot
from controller import Keyboard

from math import cos, sin

#from pid_controller import pid_velocity_fixed_height_controller
from osqp_solver import QuadrotorLQR
#from wall_following import WallFollowing

FLYING_ATTITUDE = 1

if __name__ == '__main__':

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

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

    # Get keyboard
    keyboard = Keyboard()
    keyboard.enable(timestep)

    # Initialize variables
    past_x_global = 0
    past_y_global = 0
    past_time = 0
    first_time = True
    
    # Crazyflie OSQP controller
    OSQP_crazyfile = QuadrotorLQR(dt=0.05)

    """
    # Crazyflie velocity PID controller
    PID_crazyflie = pid_velocity_fixed_height_controller()
    PID_update_last_time = robot.getTime()
    sensor_read_last_time = robot.getTime()
    """
    height_desired = FLYING_ATTITUDE

    #autonomous_mode = False

    #print("\n")
    #print("====== Controls =======\n\n")
    #print(" The Crazyflie can be controlled from your keyboard!\n")
    #print(" All controllable movement is in body coordinates\n")
    #print("- Use the up, back, right and left button to move in the horizontal plane\n")
    #print("- Use Q and E to rotate around yaw\n ")
    #print("- Use W and S to go up and down\n ")
    #print("- Press A to start autonomous mode\n")
    #print("- Press D to disable autonomous mode\n")

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
        roll = imu.getRollPitchYaw()[0]
        pitch = imu.getRollPitchYaw()[1]
        yaw = imu.getRollPitchYaw()[2]
        roll_rate = gyro.getValues()[0]
        pitch_rate = gyro.getValues()[1]
        yaw_rate = gyro.getValues()[2]
        x_global = gps.getValues()[0]
        v_x_global = (x_global - past_x_global)/dt
        y_global = gps.getValues()[1]
        v_y_global = (y_global - past_y_global)/dt
        z_global = gps.getValues()[2]
        v_z_global = (z_global - past_z_global)/dt

        # Get body fixed velocities
        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)
        v_x = v_x_global * cos_yaw + v_y_global * sin_yaw
        v_y = - v_x_global * sin_yaw + v_y_global * cos_yaw

        OSQP_crazyfile.update_x0(x_global,y_global,z_global,v_x_global,v_x_global,v_x_global,
                                 roll,pitch,yaw,roll_rate,pitch_rate,yaw_rate)
        desired_state = np.zeros((12,1))
        desired_state[0] = 0.5
        desired_state[2] = 1
        #print(desired_state)
        res = OSQP_crazyfile.solve_linear_mpc(desired_state)

        """
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
            key = keyboard.getKey()

        height_desired += height_diff_desired * dt
        
        # PID velocity controller with fixed height
        motor_power = PID_crazyflie.pid(dt, forward_desired, sideways_desired,
                                        yaw_desired, height_desired,
                                        roll, pitch, yaw_rate,
                                        z_global, v_x, v_y)

        """
        motor_power = res.x*1000000000
        
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
        print(m1,m2,m3,m4)
        
        #m1_motor.setVelocity(m1)
        #m2_motor.setVelocity(m2)
        #m3_motor.setVelocity(m3)
        #m4_motor.setVelocity(m4)
        #past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global
        past_z_global = z_global
