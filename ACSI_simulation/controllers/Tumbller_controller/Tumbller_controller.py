from controller import Robot, Gyro, InertialUnit, GPS

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
    Kp_pos = 1.0
    Ki_pos = 0.01
    Kd_pos = 0.10
    
    # Initialize variables
    pitch_integral = 0.0
    pitch_prev_error = 0.0
    pos_integral = 0.0
    pos_prev_error = 0.0

    # Target position (desired GPS coordinates)
    target_position = [0.0, 0.0]  # Replace with desired target [x, z]

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
        x_error = target_position[0] - gps_position[0]
        z_error = target_position[1] - gps_position[2]

        # PID for position control (using x_error for forward-backward movement)
        pos_error = x_error
        pos_integral += pos_error * timestep / 1000.0
        pos_derivative = (pos_error - pos_prev_error) / (timestep / 1000.0)
        pos_pid_output = Kp_pos * pos_error + Ki_pos * pos_integral + Kd_pos * pos_derivative
        pos_prev_error = pos_error

        # Combine outputs for motors
        left_motor_speed = pitch_pid_output + pos_pid_output
        right_motor_speed = pitch_pid_output + pos_pid_output

        motor_left.setVelocity(left_motor_speed)
        motor_right.setVelocity(right_motor_speed)

        print(f"Pitch PID: {pitch_pid_output}, Position PID: {pos_pid_output}")
