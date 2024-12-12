###############
# Base controller of simulation
###############

from controller import Robot, Keyboard
import numpy as np
from math import cos, sin
from pid_controller import pid_velocity_fixed_height_controller
from osqp_solver import QuadrotorLQR

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

N_MPC_HORIZON = 3
N_STATES = 12
N_CONTROLS = 4
FLYING_ATTITUDE = 0.0
SIMTIME = 100

# Create global lists to store x, y, z positions of both robots
crazyflie_positions = []
tumbller_positions = []

def initialize_robot():
    """Initialize the Webots robot and its devices."""
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Motors
    motors = {f"m{i}_motor": robot.getDevice(f"m{i}_motor") for i in range(1, 5)}
    for motor in motors.values():
        motor.setPosition(float('inf'))
        motor.setVelocity(0)

    # Sensors
    sensors = {
        "imu": robot.getDevice("inertial_unit"),
        "gps": robot.getDevice("gps"),
        "gyro": robot.getDevice("gyro"),
        "receiver": robot.getDevice("receiver")
    }
    for sensor in sensors.values():
        sensor.enable(timestep)

    # Keyboard
    keyboard = Keyboard()
    keyboard.enable(timestep)

    return robot, timestep, motors, sensors, keyboard

def process_keyboard_input(keyboard, dt, autonomous_mode, height_desired):
    """Process keyboard input to update desired states."""
    forward_desired, sideways_desired, yaw_desired, height_diff_desired = 0, 0, 0, 0
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
            yaw_desired = 1
        elif key == ord('E'):
            yaw_desired = -1
        elif key == ord('W'):
            height_diff_desired = 0.1
        elif key == ord('S'):
            height_diff_desired = -0.1
        elif key == ord('A') and not autonomous_mode:
            autonomous_mode = True
            print("Autonomous mode: ON")
        elif key == ord('D') and autonomous_mode:
            autonomous_mode = False
            print("Autonomous mode: OFF")
        key = keyboard.getKey()

    height_desired += height_diff_desired * dt
    return forward_desired, sideways_desired, yaw_desired, height_desired, autonomous_mode

def process_received_data(sensors):
    """Process received data from the emitter."""
    receiver = sensors["receiver"]
    if receiver.getQueueLength() > 0:  # Check if there's data in the queue
        data_string = receiver.getString()  # Get the string data
        receiver.nextPacket()  # Move to the next packet in the queue
        data_bytes = data_string.encode('latin1')  # Convert the string back to bytes
        state = np.frombuffer(data_bytes, dtype=float).reshape(12, 1)  # Convert bytes to numpy array
        #print("Updated state:", state)
        return state
    return None  # No data received
"""
def calculate_f(U1, U2, U3, U4):
    #Calculate motor forces from control inputs.
    mg, d, c = 55.368, 0.006, 0.1
    A = np.array([
        [1, 1, 1, 1],
        [-1, -1, 1, 1],
        [-1, 1, 1, -1],
        [1, -1, 1, -1]
    ])
    U = np.array([U1 + mg, U2 / d, U3 / d, U4 / c])
    f = np.linalg.solve(A, U)
    print("f",f)
    return f
"""
def calculate_f(U1, U2, U3, U4):
    #Calculate motor forces from control inputs.
    mg, d, c = 55.368, 0.006, 0.1
    #mg, d, c = 0,1,1
    u1 = U1+mg
    u2 = U2/d
    u3 = U3/d
    u4 = U4/c
    f1 = (u1-u2-u3+u4)/4
    f2 = (u1-u2+u3-u4)/4
    f3 = (u1+u2+u3+u4)/4
    f4 = (u1+u2-u3-u4)/4
    f1 = np.clip(f1, 0, 600)
    f2 = np.clip(f2, 0, 600)
    f3 = np.clip(f3, 0, 600)
    f4 = np.clip(f4, 0, 600)
    f = np.array((f1,f2,f3,f4))
    
    return f
def get_sensor_data(sensors):
    """Retrieve sensor data."""
    imu, gps, gyro = sensors["imu"], sensors["gps"], sensors["gyro"]
    roll, pitch, yaw = imu.getRollPitchYaw()
    roll_rate, pitch_rate, yaw_rate = gyro.getValues()
    x_global, y_global, z_global = gps.getValues()
    v_x_global, v_y_global, v_z_global = gps.getSpeedVector()
    return roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, x_global, y_global, z_global, v_x_global, v_y_global, v_z_global

def update_motors(motors, motor_power, scale, forces):
    """Update motor velocities based on control inputs."""
    motors["m1_motor"].setVelocity(-motor_power[0] - forces[0] * scale)
    motors["m2_motor"].setVelocity(motor_power[1] + forces[1] * scale)
    motors["m3_motor"].setVelocity(-motor_power[2] - forces[2] * scale)
    motors["m4_motor"].setVelocity(motor_power[3] + forces[3] * scale)

def update_motors_MPC(motors, scale, forces):
    """Update motor velocities based on control inputs."""
    motors["m1_motor"].setVelocity(- forces[0] * scale)
    motors["m2_motor"].setVelocity(  forces[1] * scale)
    motors["m3_motor"].setVelocity(- forces[2] * scale)
    motors["m4_motor"].setVelocity(  forces[3] * scale)
    print("motor",- forces[0] * scale,forces[1] * scale,- forces[2] * scale,forces[3] * scale)

def save_positions_to_file(filename, positions):
    """
    Save the positions into a CSV file.

    :param filename: The name of the file to save.
    :param positions: A list of tuples containing position data (x, y, z, time).
    """
    np.savetxt(filename, positions, delimiter=',', header='x,y,z,time', comments='', fmt='%.5f')



def finalize_plot1():
    """
    Finalize and display the trajectory plot with time-based gradients.
    """
    plt.figure()
    ax = plt.axes(projection='3d')
    # Save positions to files
    save_positions_to_file('crazyflie_positions_line.csv', crazyflie_positions)
    save_positions_to_file('tumbller_positions_line.csv', tumbller_positions)
    # Extract positions and times
    cx, cy, cz, ctime = zip(*crazyflie_positions)
    tx, ty, tz, ttime = zip(*tumbller_positions)
    #tz(0)=0
    tz_list = list(tz) # Change the first element to 0 
    tz_list[0] = 0.1 # Convert list back to tuple 
    tz = tuple(tz_list)
    #print(tz)

    # Normalize time for colormap
    ctime_norm = (np.array(ctime) - min(ctime)) / (max(ctime) - min(ctime))
    ttime_norm = (np.array(ttime) - min(ttime)) / (max(ttime) - min(ttime))

    # Create scatter plots with colormap based on time
    scatter_crazyflie = ax.scatter(cx, cy, cz, c=ctime_norm, cmap='cool')#, label='Crazyflie Trajectory')
    scatter_tumbller = ax.scatter(tx, ty, tz, c=ttime_norm, cmap='hot')#, label='Tumbller Trajectory')

    # Add colorbars for time
    #cbar_crazyflie = 
    plt.colorbar(scatter_crazyflie, ax=ax, shrink=0.5, aspect=20, pad=0.05, label='Time (s) - Crazyflie')
    #cbar_tumbller = 
    plt.colorbar(scatter_tumbller, ax=ax, shrink=0.5, aspect=20, pad=0.2, label='Time (s) - Tumbller')

    # Labels, legend, and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    ax.set_title('Trajectories')

    plt.show()



def finalize_plot():
    """
    Finalize and display the trajectory plot with time-based gradients.
    """
    plt.figure()
    ax = plt.axes(projection='3d')
    
    # Save positions to files
    save_positions_to_file('crazyflie_positions_line.csv', crazyflie_positions)
    save_positions_to_file('tumbller_positions_line.csv', tumbller_positions)
    
    # Extract positions and times
    cx, cy, cz, ctime = zip(*crazyflie_positions)
    tx, ty, tz, ttime = zip(*tumbller_positions)
    
    # Set the first Z value to a small number (0.1) for Tumbller (TZ = 0 should not be in the plot)
    tz_list = list(tz)
    tz_list[0] = 0.1  # Modify the first element to avoid plotting zero
    tz = tuple(tz_list)

    # Plot Crazyflie trajectory (red lines)
    ax.plot(cx, cy, cz, color='red', label='Crazyflie Trajectory')

    # Plot Tumbller trajectory (blue lines)
    ax.plot(tx, ty, tz, color='blue', label='Tumbller Trajectory')

    # Labels, legend, and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    ax.set_title('Trajectories')

    plt.show()


def read_log_file(filename):
    """
    Reads a log file and yields states for each timestep.
    Skips invalid rows and removes the timestamp (first term).
    Detects "phase 4" and yields a special marker when encountered.
    """
    with open(filename, 'r') as file:
        for line in file:
            try:
                # Remove square brackets and split the line
                cleaned_line = line.strip().replace('[', '').replace(']', '')
                parts = cleaned_line.split(',')
                
                # Detect phase information (assuming it's in the first column or specified format)
                if "phase 4" in cleaned_line.lower():
                    yield "PHASE_4"
                    continue
                
                # Convert the rest of the row to floats, skipping the first element (timestamp)
                data = list(map(float, parts[1:]))
                if len(data) == 12:  # Ensure the correct number of elements
                    yield np.array(data).reshape(12, 1)
                else:
                    print(f"Skipping invalid row: {line.strip()}")
            except ValueError:
                print(f"Skipping non-numeric row: {line.strip()}")



            
def main():
    robot, timestep, motors, sensors, keyboard = initialize_robot()

    PID_crazyflie = pid_velocity_fixed_height_controller()
    OSQP_crazyflie = QuadrotorLQR(dt=timestep / 1000)
    autonomous_mode = False
    height_desired = FLYING_ATTITUDE
    past_time, first_time = 0, True
    landing = False  # Flag for landing

    simulation_duration = SIMTIME  # Run simulation for 10 seconds
    start_time = robot.getTime()  # Record start time
    
    # Load log file for T_state
    log_file_path = "tb_pos_log.txt"  # Replace with your actual log file path
    log_data = read_log_file(log_file_path)

    #while robot.step(timestep) != -1:
    while robot.step(timestep) != -1:
        current_time = robot.getTime()
        elapsed_time = current_time - start_time

        if elapsed_time > simulation_duration:
            print(f"Simulation stopped after {simulation_duration} seconds.")
            break

        dt = robot.getTime() - past_time
        if first_time:
            past_time, first_time = current_time, False

        # Process keyboard input
        forward_desired, sideways_desired, yaw_desired, height_desired, autonomous_mode = process_keyboard_input(
            keyboard, dt, autonomous_mode, height_desired
        )

        # Get sensor data
        roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, x_global, y_global, z_global, v_x_global, v_y_global, v_z_global = get_sensor_data(
            sensors
        )

        # Update OSQP controller state
        # reversed the angle because of the define of system
        OSQP_crazyflie.update_x0(
            x_global, y_global, z_global, v_x_global, v_y_global, v_z_global, 
            -roll, -pitch, -yaw, -roll_rate, -pitch_rate, -yaw_rate
        )
        """
        try:
            T_state = next(log_data)
            if T_state == "PHASE_4":
                print("Phase 4 detected. Initiating landing sequence.")
                landing = True
                T_state = np.zeros((12, 1))  # Default to zero for safety
        except StopIteration:
            print("End of log file reached. Using default zero state.")
            T_state = np.zeros((12, 1))
            break

        # Desired state
        if landing:
            desired_state = np.zeros((12, 1))
            desired_state[2] = 0  # Target zero altitude for landing
        else:
            desired_state = np.zeros((12, 1))
            desired_state[:6] = T_state[:6]
            desired_state[2] += 0.4 + height_desired
            desired_state[6:12] = 0
        """
        desired_state = np.zeros((12, 1))
        T_state = process_received_data(sensors)

        if np.any(T_state) != None:
            print("received state",T_state)
            T_state = np.copy(T_state) #copy read only array
            desired_state[:6] = T_state[:6]
            #desired_state[8] = T_state[8]
        else:
            desired_state = np.zeros((12, 1))
            T_state = desired_state
            print("didn't received T states",desired_state)
        
        
        ##################################################
        #use MPC to goto certain point
        #desired_state = np.zeros((12, 1))
        #desired_state[0] = 0.0
        #desired_state[1] = -1.0
        #desired_state[2] = 1.0
        ##################################################
        desired_state[2] += 0.4+height_desired
        desired_state[6] = 0
        desired_state[7] = 0
        desired_state[8] = 0
        desired_state[9] = 0
        desired_state[10] = 0
        desired_state[11] = 0
        
        

        # Solve for control inputs
        res = OSQP_crazyflie.solve_linear_mpc(desired_state)
        U1, U2, U3, U4 = res.x[0:4] * 1e1

        # Calculate forces and update motors
        forces = calculate_f(U1, U2, U3, U4)
        motor_power = PID_crazyflie.pid(dt, forward_desired, sideways_desired, yaw_desired, height_desired, roll, pitch, yaw_rate, z_global, v_x_global, v_y_global)
        #for PID + MPC
        #update_motors(motors, motor_power, scale=0.4, forces=forces)
        #for Pure MPC
        update_motors_MPC(motors, scale=4.005, forces=forces)

        # Record positions
        crazyflie_positions.append([x_global, y_global, z_global,current_time])
        tumbller_positions.append([T_state[0, 0], T_state[1, 0], T_state[2, 0],current_time])
        #print("Cp",crazyflie_positions)
        #print("Tp",tumbller_positions)
        
        past_time = robot.getTime()
    
    finalize_plot()

if __name__ == '__main__':
    main()
