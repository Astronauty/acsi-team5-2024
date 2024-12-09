import csv
import numpy as np
from controller import Robot, Gyro, InertialUnit, GPS
import control
import time

# System parameters and LQR matrices
g = 9.81  # Gravity (m/s^2)
l = 0.095  # Length to the center of mass (m)
m = 0.5  # Mass (kg)
r = 0.04  # Wheel radius (m)

A = np.array([[0, 1, 0, 0],
              [g / l, 0, 0, 0],
              [0, 0, 0, 1],
              [-g / m, 0, 0, 0]])

B = np.array([[0],
              [-1 / (m * (l ** 2))],
              [0],
              [1 / m]])

#Q = 1000*np.diag([10, 1, 0.2, 0.00001])  # Penalizing theta, theta_dot, x, x_dot
#Q = 1000*np.diag([10, 1, 0.5, 0.1])
#Q = 100*np.diag([10,0.05,0.0001,0.00001])
#Q_1 = np.array([10000,0.1,0.0001,0.0001])
Q_1 = 100*np.array([1,1,1,1]) # x,x_dot,theta,theta_dot
Q = np.diag(Q_1)
print(Q)
R = np.array([[0.5]])  # Penalizing control effort

K, _, _ = control.lqr(A, B, Q, R)  # Calculate LQR gain
K = K[0]  # Extract gain row


def read_trajectory(file_path):
    """Read trajectory waypoints from a CSV file."""
    waypoints = []
    with open(file_path, mode='r') as file:
        reader = csv.reader(file)
        for row in reader:
            waypoints.append([float(row[0]), float(row[1])])  # Assuming [x, y] format
    return waypoints


def calibrate_gps(initial_gps, current_gps):
    """Calibrate GPS readings to start from (0, 0, 0)."""
    return [current_gps[i] - initial_gps[i] for i in range(3)]


def lqr_control(state):
    """Compute the control input using LQR."""
    return -np.dot(K, state)


def is_stable(error, threshold=0.015):
    """Check if the robot is stable within a given threshold."""
    return abs(error) < threshold

"""
def stabilize_robot(robot, timestep, imu, gps, initial_gps, target, duration=1000):
    #Stabilize the robot at the current position.
    start_time = robot.getTime()
    while robot.getTime() - start_time < duration:
        roll, pitch, yaw = imu.getRollPitchYaw()
        gps_position = calibrate_gps(initial_gps, gps.getValues())
        x_error = target[0] - gps_position[0]
        y_error = target[1] - gps_position[1]
       

        #state = np.array([theta_error, 0, x_error, y_error])
        state = np.array([pitch, pitch_rate, x_error, acc_x])
        print("state",state)
        control_input = lqr_control(state)

        motor_left.setVelocity(control_input)
        motor_right.setVelocity(control_input)
        robot.step(timestep)
"""
def stabilize_robot(robot, timestep, state, duration=1000):
    #Stabilize the robot at the current position.
    start_time = robot.getTime()
    while robot.getTime() - start_time < duration:
        control_input = lqr_control(state)
        motor_left.setVelocity(control_input)
        motor_right.setVelocity(control_input)
        robot.step(timestep)
        

if __name__ == "__main__":
    # Create the Robot instance
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Get devices
    motor_left = robot.getDevice('motor_left')
    motor_right = robot.getDevice('motor_right')
    imu = robot.getDevice('inertial unit')
    gyro = robot.getDevice('gyro')
    gps = robot.getDevice('gps')
    acc = robot.getDevice('accelerometer')

    # Enable sensors
    imu.enable(timestep)
    gps.enable(timestep)
    gyro.enable(timestep)
    acc.enable(timestep)

    # Set motors to velocity mode
    motor_left.setPosition(float('inf'))
    motor_right.setPosition(float('inf'))
    motor_left.setVelocity(0.0)
    motor_right.setVelocity(0.0)

    # Read trajectory waypoints
    trajectory_file = "Trace_T.csv"  # Replace with your file path
    waypoints = read_trajectory(trajectory_file)
    current_waypoint_index = 0

    # Initial GPS position for calibration
    first_gps_position = None

    # Main control loop
    while robot.step(timestep) != -1:
        # Read sensors
        roll, pitch, yaw = imu.getRollPitchYaw()
        roll_rate, pitch_rate, yaw_rate = gyro.getValues()
        gps_position = gps.getValues()
        acc_x,acc_y,acc_z = acc.getValues()

        # Calibrate GPS to start at [0, 0, 0]
        if first_gps_position is None:
            first_gps_position = gps_position

        adjusted_gps_position = calibrate_gps(first_gps_position, gps_position)
        #target_position = waypoints[current_waypoint_index]
        target_position = waypoints[0]
        #print(target_position)
        # Calculate errors
        x_error = target_position[0] - adjusted_gps_position[0]
        y_error = target_position[1] - adjusted_gps_position[1]
        

        # State vector
        state = np.array([-x_error, -acc_x, -pitch, -pitch_rate])
        print("state",state)

        # LQR control
        control_input = lqr_control(state)
        #m = np.clip(control_input, 0, 600)
        m = control_input
        print(m)
        # Apply control
        motor_left.setVelocity(m)
        motor_right.setVelocity(m)
        """
        # Check if waypoint is reached
        if is_stable(x_error) and is_stable(y_error):
            print(f"Reached waypoint {current_waypoint_index}: {target_position}")
            stabilize_robot(robot, timestep, imu, gps, first_gps_position, target_position)
            if current_waypoint_index < len(waypoints):
                current_waypoint_index += 1

            if current_waypoint_index >= len(waypoints):
                current_waypoint_index = len(waypoints)-1
                print("Trajectory complete")
                stabilize_robot(robot, timestep, imu, gps, first_gps_position, target_position)


        # Debugging output
        print(f"Current GPS: {adjusted_gps_position}, Target: {target_position}")
        """