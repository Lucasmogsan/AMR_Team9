import pandas as pd
import matplotlib.pyplot as plt
import os

def load_latest_data(directory, base_filename):
    # Find the latest CSV file based on the modification time
    files = [os.path.join(directory, f) for f in os.listdir(directory) if f.startswith(base_filename) and f.endswith('.csv')]
    if not files:
        print("No files found")
        return None
    latest_file = max(files, key=os.path.getmtime)
    # Read the CSV file into a pandas DataFrame
    return pd.read_csv(latest_file)


def plot_and_save(data, column_indices, path_save, title_prefix, set_point):
    time_steps = range(len(data))
    
    if title_prefix == "Surge" or title_prefix == "Heave":
        ylabel_str = f'{title_prefix} (m)'
    else:
        ylabel_str = f'{title_prefix} (rad)'
    
    # Measured vs True plot
    plt.figure(figsize=(10, 5))
    plt.plot(time_steps, data.iloc[:, column_indices[0]], label=f'{title_prefix}', color='blue')
    plt.plot(time_steps, data.iloc[:, column_indices[1]], label=f'{title_prefix} GT', color='green')
    if set_point is not None and title_prefix in ["Surge", "Heave"]:  # Only add set point for Surge or Heave
        plt.axhline(y=set_point, color='red', linestyle='-', label='Set Point')
    plt.title(f'{title_prefix} Over Time')
    plt.xlabel('Time Step')
    plt.ylabel(ylabel_str)
    plt.legend()
    plt.grid(True)
    plt.savefig(f"{path_save}{title_prefix.lower()}_over_time.png")
    plt.close()

    # Control input plot
    plt.figure(figsize=(10, 5))
    plt.plot(time_steps, data.iloc[:, column_indices[2]], label=f'{title_prefix} Control', color='magenta')
    plt.title(f'{title_prefix} Control Signal Over Time')
    plt.xlabel('Time Step')
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
    plot_and_save(data, [0, 1, 2], path_save, 'Surge', 1.5)

    # Plot and save Yaw
    plot_and_save(data, [3, 4, 5], path_save, 'Yaw', 0)

    # Plot and save Heave
    plot_and_save(data, [6, 7, 8], path_save, 'Heave', 0)
    
directory = '/overlay_ws/src/amr_prj/script/plots/data'
base_filename = 'control_data'
data = load_latest_data(directory, base_filename)
path_save = '/overlay_ws/src/amr_prj/script/plots/'

if data is not None:
    # Plot and save Surge
    plot_and_save(data[:, :3], path_save, 'Surge', 1.5)

    # Plot and save Yaw
    plot_and_save(data[:, 3:6], path_save, 'Yaw', 0)

    # Plot and save Heave
    plot_and_save(data[:, 6:], path_save, 'Heave', 0)