import csv
from controller import Robot, Gyro, InertialUnit, GPS

def read_trajectory(file_path):
    """Read trajectory waypoints from a CSV file."""
    waypoints = []
    with open(file_path, mode='r') as file:
        reader = csv.reader(file)
        for row in reader:
            waypoints.append([float(row[0]), float(row[1])])  # Assuming [x, y] format
    return waypoints

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
    
    # Enable sensors
    imu.enable(timestep)
    gyro.enable(timestep)
    gps.enable(timestep)
    
    # Set motors to velocity mode
    motor_left.setPosition(float('inf'))
    motor_right.setPosition(float('inf'))
    motor_left.setVelocity(0.0)
    motor_right.setVelocity(0.0)

    # PID parameters for pitch stabilization
    Kp_pitch = 65.0
    Ki_pitch = 0.5
    Kd_pitch = 0.5
    
    # PID parameters for position control
    Kp_pos = 3.0
    Ki_pos = 0.01
    Kd_pos = 0.10
    
    # Initialize variables
    pitch_integral = 0.0
    pitch_prev_error = 0.0
    pos_integral = 0.0
    pos_prev_error = 0.0

    # Read trajectory waypoints from a CSV file
    trajectory_file = "Trace_T.csv"  # Replace with your file path
    waypoints = read_trajectory(trajectory_file)
    current_waypoint_index = 0

    # Main control loop
    while robot.step(timestep) != -1:
        # Read IMU data
        roll, pitch, yaw = imu.getRollPitchYaw()
        pitch_rate = gyro.getValues()[1]

        # PID for pitch stabilization
        pitch_error = pitch
        pitch_integral += pitch_error * timestep / 1000.0
        pitch_derivative = (pitch_error - pitch_prev_error) / (timestep / 1000.0)
        pitch_pid_output = Kp_pitch * pitch_error + Ki_pitch * pitch_integral + Kd_pitch * pitch_derivative
        pitch_prev_error = pitch_error

        # Read GPS data
        gps_position = gps.getValues()  # [x, y, z]
        target_position = waypoints[current_waypoint_index]

        # Calculate positional errors
        x_error = target_position[0] - gps_position[0]
        y_error = target_position[1] - gps_position[1]
        distance_error = (x_error**2 + y_error**2)**0.5

        # PID for position control
        pos_error = x_error  # Forward/backward correction based on x-axis
        pos_integral += pos_error * timestep / 1000.0
        pos_derivative = (pos_error - pos_prev_error) / (timestep / 1000.0)
        pos_pid_output = Kp_pos * pos_error + Ki_pos * pos_integral + Kd_pos * pos_derivative
        pos_prev_error = pos_error

        # Check if waypoint is reached
        if distance_error < 0.01:  # Threshold for reaching waypoint
            current_waypoint_index += 1
            if current_waypoint_index >= len(waypoints):
                print("Trajectory complete")
                # Stabilize at the last position
                motor_left.setVelocity(0.0)
                motor_right.setVelocity(0.0)
                while robot.step(timestep) != -1:
                    # Keep stabilizing the robot at the final position
                    roll, pitch, yaw = imu.getRollPitchYaw()
                    pitch_error = pitch
                    pitch_integral += pitch_error * timestep / 1000.0
                    pitch_derivative = (pitch_error - pitch_prev_error) / (timestep / 1000.0)
                    pitch_pid_output = Kp_pitch * pitch_error + Ki_pitch * pitch_integral + Kd_pitch * pitch_derivative
                    pitch_prev_error = pitch_error
                    # Set the motor speed to stabilize at the last position
                    motor_left.setVelocity(pitch_pid_output)
                    motor_right.setVelocity(pitch_pid_output)
                    #print("Final Position Stabilized",gps_position)
                    #print(pitch_pid_output)
                    # Optionally, add a check for user interruption here if needed
                break  # Exit the main loop when the trajectory is completed and stabilized

        # Combine outputs for motors
        left_motor_speed = pitch_pid_output + 0.8*pos_pid_output
        right_motor_speed = pitch_pid_output + 0.8*pos_pid_output
        #print(pitch_pid_output)
        #print(pos_pid_output)
        motor_left.setVelocity(left_motor_speed)
        motor_right.setVelocity(right_motor_speed)

        #print(f"Waypoint: {current_waypoint_index}, GPS: {gps_position}, Target: {target_position}")
