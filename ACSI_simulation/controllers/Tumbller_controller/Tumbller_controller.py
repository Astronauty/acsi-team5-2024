from controller import Robot, Gyro, InertialUnit, GPS, Keyboard

def initialize_devices(robot, timestep):
    """Initialize and enable sensors and motors."""
    motor_left = robot.getDevice('motor_left')
    motor_right = robot.getDevice('motor_right')
    imu = robot.getDevice('inertial unit')
    gyro = robot.getDevice('gyro')
    gps = robot.getDevice('gps')
    keyboard = robot.getKeyboard()

    # Enable sensors
    imu.enable(timestep)
    gyro.enable(timestep)
    gps.enable(timestep)
    keyboard.enable(timestep)

    # Set motors to velocity mode
    motor_left.setPosition(float('inf'))
    motor_right.setPosition(float('inf'))
    motor_left.setVelocity(0.0)
    motor_right.setVelocity(0.0)

    return motor_left, motor_right, imu, gyro, gps, keyboard

def calculate_pid(error, integral, prev_error, Kp, Ki, Kd, timestep):
    """Generic PID control calculation."""
    integral += error * timestep / 1000.0
    derivative = (error - prev_error) / (timestep / 1000.0)
    output = Kp * error + Ki * integral + Kd * derivative
    return output, integral, error

def main():
    # Create the Robot instance
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    
    # Initialize devices
    motor_left, motor_right, imu, gyro, gps, keyboard = initialize_devices(robot, timestep)
    
    # PID parameters for pitch stabilization
    Kp_pitch = 180.0
    Ki_pitch = 0.0
    Kd_pitch = 0.3
    pitch_integral = 0.0
    pitch_prev_error = 0.0

    # PID parameters for speed stabilization
    Kp_speed = 15#10.0
    Ki_speed = Kp_speed / 200.0
    Kd_speed = 0.0
    speed_integral = 0.0
    speed_prev_error = 0.0

    # PID parameters for Z rotation control
    Kp_z_rotation = 0.1
    Ki_z_rotation = 0.0
    Kd_z_rotation = 0.0
    z_rotation_integral = 0.0
    z_rotation_prev_error = 0.0

    # Target values
    target_speed = 0.0
    target_z_rotation = 0.0  # Maintain zero angular velocity around Z-axis

    # Speed and rotation increments for keyboard control
    speed_increment = 0.05
    rotation_increment = 0.1

    # Main control loop
    while robot.step(timestep) != -1:
        # Read IMU data
        _, pitch, _ = imu.getRollPitchYaw()
        pitch_rate = gyro.getValues()[1]

        # Read GPS data for speed
        velocity = gps.getSpeed()

        # Read gyro data for Z rotation
        z_rotation_rate = gyro.getValues()[2]

        # Read keyboard input
        key = keyboard.getKey()
        if key == Keyboard.UP:
            target_speed += speed_increment
        elif key == Keyboard.DOWN:
            target_speed -= speed_increment
        elif key == Keyboard.LEFT:
            target_z_rotation += rotation_increment
        elif key == Keyboard.RIGHT:
            target_z_rotation -= rotation_increment
        elif key == -1:  # No key pressed
            pass  # Maintain the current targets

        # PID for pitch stabilization
        pitch_pid_output, pitch_integral, pitch_prev_error = calculate_pid(
            pitch, pitch_integral, pitch_prev_error, Kp_pitch, Ki_pitch, Kd_pitch, timestep)

        # PID for speed stabilization
        speed_pid_output, speed_integral, speed_prev_error = calculate_pid(
            target_speed - velocity, speed_integral, speed_prev_error, Kp_speed, Ki_speed, Kd_speed, timestep)

        # PID for Z rotation stabilization
        z_rotation_error = target_z_rotation - z_rotation_rate
        z_rotation_pid_output, z_rotation_integral, z_rotation_prev_error = calculate_pid(
            z_rotation_error, z_rotation_integral, z_rotation_prev_error, Kp_z_rotation, Ki_z_rotation, Kd_z_rotation, timestep)

        # Combine outputs for motors
        left_motor_speed = pitch_pid_output - speed_pid_output + z_rotation_pid_output+0.07
        right_motor_speed = pitch_pid_output - speed_pid_output - z_rotation_pid_output +0.04

        motor_left.setVelocity(left_motor_speed)
        motor_right.setVelocity(right_motor_speed)

if __name__ == "__main__":
    main()
