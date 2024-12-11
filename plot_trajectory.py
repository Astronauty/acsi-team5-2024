import ast
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def load_trajectory(filename):
    """
    Load trajectory data from a file and return a DataFrame with x, y, z positions.
    """
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

    # Convert x, y, z to numeric and drop rows with NaN
    for col in ["x", "y", "z"]:
        df[col] = pd.to_numeric(df[col], errors='coerce')
    df = df.dropna(subset=["x", "y", "z"])

    return df


def plot_trajectories(crazyflie_file, tumbller_file):
    """
    Plot the trajectories of the Crazyflie and Tumbller in a 3D space.
    """
    # Load the trajectory data
    cf_df = load_trajectory(crazyflie_file)
    tb_df = load_trajectory(tumbller_file)

    # Plot the trajectories
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot Crazyflie trajectory (red)
    ax.plot(cf_df["x"], cf_df["y"], cf_df["z"], color="red", label="Crazyflie")

    # Plot Tumbller trajectory (blue)
    ax.plot(tb_df["x"], tb_df["y"], tb_df["z"], color="blue", label="Tumbller")

    # Set labels and limits
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_zlabel("Z Position (m)")
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([0, 1])
    ax.set_title("3D Trajectories of Crazyflie and Tumbller")

    # Add legend
    plt.legend()
    plt.show()


if __name__ == "__main__":
    # File paths
    crazyflie_file = "cf_pos_log.txt"
    tumbller_file = "tb_pos_log.txt"

    # Plot the trajectories
    plot_trajectories(crazyflie_file, tumbller_file)
