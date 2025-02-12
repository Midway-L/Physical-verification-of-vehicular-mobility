import json
import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate

def read_json_file(filename):
    timestamps = []
    distances = []
    with open(filename, 'r') as f:
        for line in f:
            data = json.loads(line)
            timestamps.append(data['timestamp'])
            distances.append(data['distance'])
    return timestamps, distances

def calculate_error(ground_truth_t, ground_truth_d, compare_t, compare_d):
    f = interpolate.interp1d(ground_truth_t, ground_truth_d, kind='linear', fill_value='extrapolate')
    interpolated_ground_truth = f(compare_t)
    error = np.abs(np.array(compare_d) - interpolated_ground_truth)
    return compare_t, error

def calculate_overall_error(error):
    mae = np.mean(error)  # Mean Absolute Error
    rmse = np.sqrt(np.mean(np.square(error)))  # Root Mean Square Error
    return mae, rmse

filenames = ['cars_distance.json', 'processed_lidar_data_147.json', 'processed_lidar_data_151.json', 'rssi_distance.json']
colors = ['red', 'blue', 'green', 'purple']
labels = ['ground truth', 'vehicle 147 lidar', 'vehicle 151 lidar', 'rssi']

plt.figure(figsize=(15, 12))

# Plot distance vs time
plt.subplot(2, 1, 1)

ground_truth_t, ground_truth_d = read_json_file(filenames[0])

for filename, color, label in zip(filenames, colors, labels):
    timestamps, distances = read_json_file(filename)
    plt.plot(timestamps, distances, color=color, label=label)

plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.title('Distance vs Time')
plt.legend()
plt.grid(True)

# Plot error vs time
plt.subplot(2, 1, 2)

error_text = "Overall Errors:\n"

for filename, color, label in zip(filenames[1:], colors[1:], labels[1:]):
    timestamps, distances = read_json_file(filename)
    error_t, error = calculate_error(ground_truth_t, ground_truth_d, timestamps, distances)
    plt.plot(error_t, error, color=color, label=f'{label} error')
    
    mae, rmse = calculate_overall_error(error)
    error_text += f"{label}:\n  MAE: {mae:.2f}m\n  RMSE: {rmse:.2f}m\n\n"

plt.xlabel('Time (s)')
plt.ylabel('Error (m)')
plt.title('Error vs Time')
plt.legend()
plt.grid(True)

# Add error text to the plot
plt.text(1.05, 0.5, error_text, transform=plt.gca().transAxes, verticalalignment='center', fontsize=9)

plt.tight_layout()
plt.subplots_adjust(right=0.8)  # Adjust the right margin to make room for the text
plt.show()

output_filename = 'distance_and_error_vs_time_with_overall_errors.png'
plt.savefig(output_filename, dpi=300, bbox_inches='tight')
print(f"save to {output_filename}")