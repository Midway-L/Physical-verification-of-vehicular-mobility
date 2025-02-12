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

def read_vehicle_data(filename):
    """Read vehicle data including compass and lidar angle"""
    timestamps = []
    compass_angles = []
    lidar_angles = []
    
    with open(filename, 'r') as f:
        for line in f:
            data = json.loads(line)
            timestamps.append(data['new_timestamp'])
            compass_angles.append(data['imu_compass'])
            lidar_angles.append(np.radians(data['lidar_angle']))
    return timestamps, compass_angles, lidar_angles


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

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def calculate_true_relative_angle(ego_compass, other_compass):
    relative_angle = normalize_angle(other_compass - ego_compass)
    return relative_angle


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

def plot_angle_comparison(lead_file, follow_file):
    """Plot angle comparison from two vehicle data files"""
    # Read data from both files
    lead_t, lead_compass, _ = read_vehicle_data(lead_file)
    follow_t, follow_compass, follow_lidar = read_vehicle_data(follow_file)
    
    # Use follower's timestamps as reference
    # Interpolate leader's compass to follower's timestamps
    f = interpolate.interp1d(lead_t, lead_compass, kind='linear', fill_value='extrapolate')
    lead_compass_interp = f(follow_t)
    
    # Calculate true relative angles
    true_relative_angles = [calculate_true_relative_angle(fc, lc) 
                          for fc, lc in zip(follow_compass, lead_compass_interp)]
    
    # Calculate angle errors
    angle_errors = [normalize_angle(lidar - true) 
                   for lidar, true in zip(follow_lidar, true_relative_angles)]
    
    # Create figure with two subplots
    fig, ax1 = plt.subplots(figsize=(15, 16))
    
    # Plot angles
    ax1.plot(follow_t, true_relative_angles, color='red', label='True Relative Angle')
    ax1.plot(follow_t, follow_lidar, color='blue', label='LiDAR Estimated Angle')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Angle (rad)')
    ax1.set_title('Relative Angle Comparison')
    ax1.legend()
    ax1.grid(True)
    
    # Calculate error metrics
    rmse = np.sqrt(np.mean(np.square(angle_errors)))
    mae = np.mean(np.abs(angle_errors))
    max_error = np.max(np.abs(angle_errors))
    
    # Add error metrics directly to the first plot
    stats_text = (f'RMSE: {rmse:.3f} rad ({np.degrees(rmse):.1f}°)\n'
                 f'MAE: {mae:.3f} rad ({np.degrees(mae):.1f}°)\n'
                 f'Max Error: {max_error:.3f} rad ({np.degrees(max_error):.1f}°)')
    ax1.text(0.02, 0.95, stats_text, transform=ax1.transAxes, 
             bbox=dict(facecolor='white', alpha=0.8), verticalalignment='top')
    
    plt.tight_layout(h_pad=3.0)
    return fig, ax1

ground_truth_t, ground_truth_d = read_json_file('cars_distance.json')
lidar_197_t, lidar_197_d = read_lidar_data('processed_data_548.json')
lidar_201_t, lidar_201_d = read_lidar_data('processed_data_552.json')
rssi_t, rssi_d = read_json_file('improved_rssi_distance.json')
sumo_t, sumo_d, sumo_rs, sumo_ls, sumo_fs = read_sumo_data('sumo_ground.json')
veins_t, veins_d = read_veins_data('veins_distance.json')

# Figure 1: Ground Truth vs LiDAR vs RSSI
fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 16))
data_list = [(lidar_197_t, lidar_197_d), (lidar_201_t, lidar_201_d), (rssi_t, rssi_d)]
colors = ['blue', 'green','purple']
labels = ['Lead Vehicle Lidar', 'Follow Vehicle LiDAR', 'Improved RSSI']
error_text1 = plot_distance_and_error(ax1, ax2, ground_truth_t, ground_truth_d, data_list, colors, labels)
plt.text(1.05, 0.5, error_text1, transform=ax2.transAxes, verticalalignment='center', fontsize=9)
plt.tight_layout(h_pad=3.0)
plt.savefig('Ground_truth, Lead and follow lidar and rssi comparison.png', dpi=300, bbox_inches='tight')

# Figure 2: Ground Truth vs SUMO vs Veins
fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 16))
data_list = [(sumo_t, sumo_d), (veins_t, veins_d)]
colors = ['orange', 'cyan']
labels = ['SUMO', 'Veins']
error_text2 = plot_distance_and_error(ax1, ax2, ground_truth_t, ground_truth_d, data_list, colors, labels)
plt.text(1.05, 0.5, error_text2, transform=ax2.transAxes, verticalalignment='center', fontsize=9)
plt.tight_layout(h_pad=3.0)
plt.savefig('Carla_Sumo_veins_comparison.png', dpi=300, bbox_inches='tight')

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

fig5, axes = plot_angle_comparison('processed_data_548.json', 'processed_data_552.json')
fig5.savefig('heading_angle_comparison.png', dpi=300, bbox_inches='tight')

plt.show()
