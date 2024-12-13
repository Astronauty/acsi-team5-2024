import ast
import pandas as pd
import re
import matplotlib.pyplot as plt

def load_trajectory(filename):
    """
    Loads a trajectory file and associates each data sample with the corresponding phase number.
    When a new phase is detected, all subsequent samples are assigned to that phase.
    """
    data = []
    current_phase = 0  # Default phase for data without an assigned phase

    with open(filename, "r") as file:
        for line in file:
            line = line.strip()

            # Check if the line indicates a phase and extract the first number after "Phase"
            if "Phase" in line:
                match = re.search(r"Phase\s+(\d+)", line)  # Look for "Phase <number>"
                if match:
                    current_phase = int(match.group(1))  # Extract the first number after "Phase"
                    print(f"Detected new phase: {current_phase}")
                continue  # Skip this line

            # Parse data lines
            if line.startswith("[") and "]" in line:
                try:
                    data_point = ast.literal_eval(line)
                    if len(data_point) == 13:  # Ensure it has the correct number of columns
                        data_point.append(current_phase)  # Append the current phase number to the row
                        data.append(data_point)
                    else:
                        print(f"Skipping row with unexpected column count: {data_point}")
                except (ValueError, SyntaxError):
                    print(f"Skipping malformed row: {line}")
            else:
                print(f"Skipping non-data row: {line}")

    # Define column names for the DataFrame, including the new 'phase' column
    columns = [
        "time", "x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw",
        "roll_rate", "pitch_rate", "yaw_rate", "phase"
    ]

    # Create DataFrame
    if data:
        return pd.DataFrame(data, columns=columns)
    else:
        print("No valid data found in the file.")
        return pd.DataFrame()  # Return an empty DataFrame if no valid data is found

def plot_trajectories(crazyflie_file, tumbller_file, phases):
    """
    Plot the trajectories of the Crazyflie and Tumbller for specified phases in a 3D space.
    """
    try:
        # Load trajectory data
        cf_df = load_trajectory(crazyflie_file)
        tb_df = load_trajectory(tumbller_file)
    except Exception as e:
        print(f"Error loading trajectory files: {e}")
        return

    # Filter the data so include only data from Phase 2 and Phase 3
    cf_df = cf_df[cf_df["phase"].isin(phases)]
    tb_df = tb_df[tb_df["phase"].isin(phases)]

    # remove any rows with NaN or all zeros
    cf_df = cf_df.dropna(subset=["x", "y", "z"])
    tb_df = tb_df.dropna(subset=["x", "y", "z"])
    cf_df = cf_df.loc[~(cf_df[["x", "y", "z"]] == 0).all(axis=1)]
    tb_df = tb_df.loc[~(tb_df[["x", "y", "z"]] == 0).all(axis=1)]


    # Convert DataFrame columns to NumPy arrays for plotting
    cf_x, cf_y, cf_z = cf_df["x"].to_numpy(), cf_df["y"].to_numpy(), cf_df["z"].to_numpy()
    tb_x, tb_y, tb_z = tb_df["x"].to_numpy(), tb_df["y"].to_numpy(), tb_df["z"].to_numpy()

    # Initialize a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot Crazyflie trajectory
    ax.plot(cf_x, cf_y, cf_z, color="red", label="Crazyflie")

    # Plot Tumbller trajectory
    ax.plot(tb_x, tb_y, tb_z, color="blue", label="Tumbller")

    # Set plot labels, limits, and title
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_zlabel("Z Position (m)")
    ax.set_title("3D Trajectories of Crazyflie and Tumbller (Phase 2 and 3)")

    # Display legend and plot
    ax.legend()
    plt.show()


if __name__ == "__main__":
    # File paths for Crazyflie and Tumbller data
    crazyflie_file = "circular_trajectory/cf_pos_log.txt"
    tumbller_file = "circular_trajectory/tb_pos_log.txt"

    # Phases to plot (Phase 2 and Phase 3)
    phases_to_plot = [2, 3]

    # Plot the trajectories
    plot_trajectories(crazyflie_file, tumbller_file, phases_to_plot)
