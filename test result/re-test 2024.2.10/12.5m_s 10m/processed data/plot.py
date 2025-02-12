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
            timestamps.append(data['new_timestamp'])
            distances.append(data['distance'])
    return timestamps, distances

def read_lidar_data(filename):
    timestamps = []
    distances = []
    with open(filename, 'r') as f:
        for line in f:
            data = json.loads(line)
            timestamps.append(data['new_timestamp'])
            distances.append(data['lidar_distance'])
    return timestamps, distances

def read_sumo_data(filename):
    timestamps = []
    distances = []
    relative_speeds = []
    leader_speeds = []
    follower_speeds = []
    with open(filename, 'r') as f:
        for line in f:
            data = json.loads(line)
            timestamps.append(data['new_timestamp'])
            distances.append(data['distance'])
            relative_speeds.append(data['relative_speed'])
            leader_speeds.append(data['leader_speed'])
            follower_speeds.append(data['following_speed'])
    return timestamps, distances, relative_speeds, leader_speeds, follower_speeds

def read_veins_data(filename):
    timestamps = []
    distances = []
    with open(filename, 'r') as f:
        for line in f:
            data = json.loads(line)
            timestamps.append(data['new_timestamp'])
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

def plot_distance_and_error(ax1, ax2, ground_truth_t, ground_truth_d, data_list, colors, labels):
    ax1.plot(ground_truth_t, ground_truth_d, color='red', label='Ground Truth')
    error_text = "Overall Errors:\n"

    for data, color, label in zip(data_list, colors, labels):
        timestamps, distances = data
        ax1.plot(timestamps, distances, color=color, label=label)
        
        error_t, error = calculate_error(ground_truth_t, ground_truth_d, timestamps, distances)
        ax2.plot(error_t, error, color=color, label=f'{label} error')
        
        mae, rmse = calculate_overall_error(error)
        error_text += f"{label}:\n  MAE: {mae:.2f}m\n  RMSE: {rmse:.2f}m\n\n"

    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Distance (m)')
    ax1.set_title('Distance vs Time', pad=20)
    ax1.legend()
    ax1.grid(True)

    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error (m)')
    ax2.set_title('Error vs Time', pad=20)
    ax2.legend()
    ax2.grid(True)

    return error_text

carla_t, carla_d = read_json_file('carla_distance.json')
lidar_lead_t, lidar_lead_d = read_lidar_data('processed_data_278.json')
lidar_follow_t, lidar_follow_d = read_lidar_data('processed_data_282.json')
rssi_t, rssi_d = read_json_file('improved_rssi_distance.json')
sumo_t, sumo_d, sumo_rs, sumo_ls, sumo_fs = read_sumo_data('sumo_ground.json')
veins_t, veins_d = read_veins_data('veins_distance.json')

# Figure 1: Ground Truth vs LiDAR vs RSSI
fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 16))
data_list = [(lidar_follow_t, lidar_follow_d), (rssi_t, rssi_d)]
colors = ['blue', 'green']
labels = ['Following Vehicl LiDAR', 'Improved RSSI']
error_text1 = plot_distance_and_error(ax1, ax2, sumo_t, sumo_d, data_list, colors, labels)
plt.text(1.05, 0.5, error_text1, transform=ax2.transAxes, verticalalignment='center', fontsize=9)
plt.tight_layout(h_pad=3.0)
plt.savefig('Sumo distance, Lidar and Rssi Comparison.png', dpi=300, bbox_inches='tight')

# Figure 2: Carla vs sumo vs Veins
fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 16))
data_list = [(sumo_t, sumo_d), (veins_t, veins_d)]
colors = ['orange', 'cyan']
labels = ['Sumo', 'Veins']
error_text2 = plot_distance_and_error(ax1, ax2, carla_t, carla_d, data_list, colors, labels)
plt.text(1.05, 0.5, error_text2, transform=ax2.transAxes, verticalalignment='center', fontsize=9)
plt.tight_layout(h_pad=3.0)
plt.savefig('Carla_sumo_veins_comparison.png', dpi=300, bbox_inches='tight')

# Figure 3: Veins vs RSSI
fig3, ax = plt.subplots(figsize=(15, 8))
ax.plot(veins_t, veins_d, color='cyan', label='Veins Distance')
ax.plot(rssi_t, rssi_d, color='purple', label='RSSI Distance')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Distance (m)')
ax.set_title('Veins Distance vs RSSI Distance', pad=20)
ax.legend()
ax.grid(True)

error_t, error = calculate_error(veins_t, veins_d, rssi_t, rssi_d)
mae, rmse = calculate_overall_error(error)
error_text3 = f"RSSI vs Veins Error Metrics:\nMAE: {mae:.2f}m\nRMSE: {rmse:.2f}m"
plt.text(0.05, 0.95, error_text3, transform=ax.transAxes, verticalalignment='top', fontsize=9, bbox=dict(facecolor='white', alpha=0.8))
plt.tight_layout()
plt.savefig('veins_vs_rssi_distance.png', dpi=300, bbox_inches='tight')

# Figure 4: SUMO Distance and Speed
fig4, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 16))
ax1.plot(sumo_t, sumo_d, color='blue')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Distance (m)')
ax1.set_title('SUMO Distance vs Time', pad=20)
ax1.grid(True)

ax2.plot(sumo_t, sumo_rs, color='red', label='Relative Speed')
ax2.plot(sumo_t, sumo_ls, color='green', label='Leader Speed')
ax2.plot(sumo_t, sumo_fs, color='orange', label='Follower Speed')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Speed (m/s)')
ax2.set_title('SUMO Speeds vs Time', pad=20)
ax2.legend()
ax2.grid(True)

plt.tight_layout(h_pad=3.0)
plt.savefig('sumo_distance_and_speeds.png', dpi=300, bbox_inches='tight')

plt.show()
