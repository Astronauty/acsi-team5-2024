import ast
import pandas as pd
import matplotlib.pyplot as plt

def plot_trajectory(filename):
    data = []
    with open(filename, "r") as file:
        for line in file:
            # Skip lines that don't represent data points
            if not line.strip().startswith("["):
                continue
            try:
                data_point = ast.literal_eval(line.strip())
                data.append(data_point)
            except (ValueError, SyntaxError):
                continue

    # Create a DataFrame
    columns = ["time", "x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw", "roll_rate", "pitch_rate", "yaw_rate"]
    df = pd.DataFrame(data, columns=columns)

    # Convert columns to numeric if possible
    for col in ["x", "y", "z"]:
        df[col] = pd.to_numeric(df[col], errors='coerce')

    # Drop rows with NaN values in position data
    df = df.dropna(subset=["x", "y", "z"])

    # Plot the 3D trajectory
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(df["x"], df["y"], df["z"], label="Trajectory")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_zlabel("Z Position (m)")
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([0, 1])
    ax.set_title("3D Trajectory of the System")
    plt.legend()
    plt.show()

if __name__ == "__main__":

    file_name = "cf_pos_log.txt"
    plot_trajectory(file_name)
