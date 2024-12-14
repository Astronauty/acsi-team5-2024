import ast
import re
import pandas as pd

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


if __name__ == "__main__":
    # File paths
    crazyflie_file = "linear_trajectory/cf_pos_log.txt"
    ref_file = "linear_trajectory/cf_ref_log.txt"

    phases = [2, 3]

    # Load the trajectory data
    cf_df = load_trajectory(crazyflie_file)
    ref_df = load_trajectory(ref_file)

    # Save cf_df and ref_df to CSV files
    cf_df.to_csv("c_cf_trajectory.csv", index=False)
    ref_df.to_csv("c_ref_trajectory.csv", index=False)

    # Pass these to ChatGPT to generate the plots with the prompt:
    # "Can you plot the difference between the reference and actual trajectory?
    # Can you do a single plot that shows the x, y, and z component difference over
    # time during Phase 2 and 3"
