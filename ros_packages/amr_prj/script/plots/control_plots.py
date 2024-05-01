import numpy as np
import matplotlib.pyplot as plt
import os
import rospy


def load_latest_data(directory, base_filename):
    # Find the latest file based on the modification time
    files = [os.path.join(directory, f) for f in os.listdir(directory) if f.startswith(base_filename)]
    if not files:
        print("No files found")
        return None
    latest_file = max(files, key=os.path.getmtime)
    return np.load(latest_file, allow_pickle=True)

def calculate_metrics(timestamps, data, setpoint, from_val, to_val):
    error = data - setpoint
    steady_state_error = np.mean(error[-10:])

    # Check if within absolute error threshold for settling time
    within_range = np.abs(error) <= 0.1  # Using a specific value for demonstration
    first_in_range = np.where(within_range)[0]
    if first_in_range.size > 0 and np.all(within_range[first_in_range[0]:]):
        settling_time = timestamps[first_in_range[0]]
    else:
        settling_time = None

    # Adjust rise time calculations to reflect realistic data behavior for a decreasing trajectory
    start_threshold = from_val - 0.1 * (from_val - to_val)
    end_threshold = from_val - 0.9 * (from_val - to_val)
    start_indices = np.where(data <= start_threshold)[0]
    end_indices = np.where(data <= end_threshold)[0]

    # Ensure that both start and end indices are found
    if start_indices.size > 0 and end_indices.size > 0 and end_indices[0] > start_indices[0]:
        rise_time = timestamps[end_indices[0]] - timestamps[start_indices[0]]
    else:
        rise_time = None  # Safeguard if data does not cross the threshold correctly

    return steady_state_error, settling_time, rise_time



def plot_and_save(data, path_save, title_prefix, set_point):
    if isinstance(data[0, 0], np.float64):  # Check if the first element is a float
        timestamps = data[:, 0]  # No conversion needed
    else:
        timestamps = [time.to_sec() for time in data[:, 0]]  # Convert rospy.Time to seconds
    
    if data.ndim == 1:  # Handle one-dimensional array differently
        data = data.reshape(-1, 1)  # Reshape to a column vector for consistency
    
    if title_prefix == "Surge":
        ylabel_str = f'{title_prefix} (m)'
        plot_data = data[:, 1:4]
    elif title_prefix == "Yaw":
        ylabel_str = f'{title_prefix} (rad)'
        plot_data = data[:, 4:7]
    elif title_prefix == "Heave":
        ylabel_str = f'{title_prefix} (m)'
        plot_data = data[:, 7:10]
     
    # Calculate metrics
    steady_state_error, settling_time, rise_time = calculate_metrics(timestamps, plot_data[:, 1], set_point, 0.5, 0)
    
    # Measured vs True plot
    plt.figure(figsize=(10, 5))
    plt.plot(timestamps, plot_data[:, 0], label=f'{title_prefix}', color='blue')
    if plot_data.shape[1] >= 3:  # Ensure there's at least three columns for GT data
        plt.plot(timestamps, plot_data[:, 1], label=f'{title_prefix} GT', color='green')
    plt.axhline(y=set_point, color='red', linestyle='-', label='Set Point')
    if settling_time is not None and rise_time is not None:
        settling_time_offset = settling_time - 20.47
        plt.axvline(x=settling_time, color='purple', linestyle='--', label='Settling Time')
        plt.title(f'{title_prefix} Over Time\nSSE: {steady_state_error:.3f}, Settling Time: {settling_time_offset:.3f}s, Rise Time: {rise_time:.3f}s')
    plt.xlabel('Time (seconds)')
    plt.ylabel(ylabel_str)
    plt.legend()
    plt.grid(True)
    plt.savefig(f"{path_save}{title_prefix.lower()}_over_time.png")
    plt.close()

    # Control input
    plt.figure(figsize=(10, 5))
    plt.plot(timestamps, plot_data[:, 2], label=f'{title_prefix} Control', color='magenta')
    plt.title(f'{title_prefix} Control Signal Over Time')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Control Signal')
    plt.legend()
    plt.grid(True)
    plt.savefig(f"{path_save}{title_prefix.lower()}_control_signal.png")
    plt.close()

# Usage
directory = '/overlay_ws/src/amr_prj/script/plots/data'
base_filename = 'control_data_surge_tracking.npy'
data = load_latest_data(directory, base_filename)
path_save = '/overlay_ws/src/amr_prj/script/plots/'
data = np.load(os.path.join(directory, base_filename), allow_pickle=True)
if isinstance(data[0, 0], np.float64):  # Check if the first element is a float
    timestamps = data[:, 0]  # No conversion needed
else:
    timestamps = [time.to_sec() for time in data[:, 0]]  # Convert rospy.Time to seconds


if data is not None:
    # Plot and save Surge
    plot_and_save(data[:, :], path_save, 'Surge', 1)

    # Plot and save Yaw
    plot_and_save(data[:, :], path_save, 'Yaw', 0)

    #Plot and save Heave
    plot_and_save(data[:, :], path_save, 'Heave', 0)
