import numpy as np
import matplotlib.pyplot as plt
import os

def load_latest_data(directory, base_filename):
    # Find the latest file based on the modification time
    files = [os.path.join(directory, f) for f in os.listdir(directory) if f.startswith(base_filename)]
    if not files:
        print("No files found")
        return None
    latest_file = max(files, key=os.path.getmtime)
    return np.load(latest_file, allow_pickle=True)


def plot_and_save(data, path_save, title_prefix, set_point):
    if isinstance(data[0, 0], np.float64):  # Check if the first element is a float
        timestamps = data[:, 0]  # No conversion needed
    else:
        timestamps = [time.to_sec() for time in data[:, 0]]  # Convert rospy.Time to seconds
    
    if data.ndim == 1:  # Handle one-dimensional array differently
        data = data.reshape(-1, 1)  # Reshape to a column vector for consistency
    
    if title_prefix == "Surge" or title_prefix == "Heave":
        ylabel_str = f'{title_prefix} (m)'
    else:
        ylabel_str = f'{title_prefix} (rad)'
    
    # Measured vs True plot
    plt.figure(figsize=(10, 5))
    plt.plot(timestamps, data[:, 1], label=f'{title_prefix}', color='blue')
    if data.shape[1] >= 3:  # Ensure there's at least three columns for GT data
        plt.plot(timestamps, data[:, 2], label=f'{title_prefix} GT', color='green')
    if set_point is not None and title_prefix in ["Surge", "Heave"]:
        plt.axhline(y=set_point, color='red', linestyle='-', label='Set Point')
    plt.title(f'{title_prefix} Over Time')
    plt.xlabel('Time (seconds)')
    plt.ylabel(ylabel_str)
    plt.legend()
    plt.grid(True)
    plt.savefig(f"{path_save}{title_prefix.lower()}_over_time.png")
    plt.close()

    # Control input
    if data.shape[1] >= 4:  # Ensure there's at least four columns for control signal
        plt.figure(figsize=(10, 5))
        plt.plot(timestamps, data[:, 3], label=f'{title_prefix} Control', color='magenta')
        plt.title(f'{title_prefix} Control Signal Over Time')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Control Signal')
        plt.legend()
        plt.grid()
        plt.savefig(f"{path_save}{title_prefix.lower()}_control_signal.png")
        plt.close()

# Usage
directory = '/overlay_ws/src/amr_prj/script/plots/data'
base_filename = 'control_data'
data = load_latest_data(directory, base_filename)
path_save = '/overlay_ws/src/amr_prj/script/plots/'

if data is not None:
    # Plot and save Surge
    plot_and_save(data[:, :], path_save, 'Surge', 1.5)

    # Plot and save Yaw
    plot_and_save(data[:, :], path_save, 'Yaw', 0)

    # Plot and save Heave
    plot_and_save(data[:, :], path_save, 'Heave', 0)
