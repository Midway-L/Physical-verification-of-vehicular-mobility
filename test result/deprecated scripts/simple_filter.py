import json
import numpy as np
import matplotlib.pyplot as plt

def read_json_file(file_path):
    with open(file_path, 'r') as file:
        return [json.loads(line) for line in file]

class KalmanFilter:
    def __init__(self, dim_x, dim_z):
        self.dim_x = dim_x
        self.dim_z = dim_z
        
        self.x = np.zeros((dim_x, 1))  
        self.P = np.eye(dim_x) * 1000  
        self.F = np.eye(dim_x)        
        self.H = np.zeros((dim_z, dim_x))  
        self.R = np.eye(dim_z)         
        self.Q = np.eye(dim_x)        

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.dim_x)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)

def create_kalman_filter():
    kf = KalmanFilter(dim_x=4, dim_z=4)
    dt = 0.1  
    
    kf.F = np.array([
        [1, 0, dt, 0],
        [0, 1, 0, dt],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    kf.H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])
    
    kf.Q *= 0.1
    kf.R = np.diag([1.0, 1.0, 5.0, 5.0])
    
    return kf

def run_kalman_filter(lidar_data, rssi_data, true_positions, self_positions, kf):
    
    timestamps = []
    estimated_positions = []
    true_relative_positions = []
    
    lidar_index = 0
    rssi_index = 0
    
    for true_pos, self_pos in zip(true_positions, self_positions):
        timestamp = true_pos['timestamp']
        
        while lidar_index < len(lidar_data) and lidar_data[lidar_index]['timestamp'] <= timestamp:
            lidar_measurement = lidar_data[lidar_index]
            lidar_index += 1
        
        while rssi_index < len(rssi_data) and rssi_data[rssi_index]['timestamp'] <= timestamp:
            rssi_measurement = rssi_data[rssi_index]
            rssi_index += 1
        
        if 'avg_x' in lidar_measurement and 'avg_y' in lidar_measurement and 'distance' in rssi_measurement and 'angle' in lidar_measurement:
            lidar_x, lidar_y = lidar_measurement['avg_x'], lidar_measurement['avg_y']
            rssi_distance = rssi_measurement['distance']
            rssi_angle = lidar_measurement['angle']  
            rssi_x = rssi_distance * np.cos(np.radians(rssi_angle))
            rssi_y = rssi_distance * np.sin(np.radians(rssi_angle))
            
            z = np.array([[lidar_x], [lidar_y], [rssi_x], [rssi_y]])
        else:
            continue
        
        kf.predict()
        kf.update(z)
        
        timestamps.append(timestamp)
        estimated_positions.append(kf.x[:2].flatten())
        
        true_relative_x = abs(true_pos['x'] - self_pos['x'])
        true_relative_y = abs(true_pos['y'] - self_pos['y'])
        true_relative_positions.append([true_relative_x, true_relative_y])
    
    return np.array(timestamps), np.array(estimated_positions), np.array(true_relative_positions)

def optimize_kalman_filter(lidar_data, rssi_data, true_positions, self_positions):
    best_rmse = float('inf')
    best_q = None
    best_r = None
    results = []

    q_scales = [0.01, 0.1, 1, 10, 100]
    r_scales = [0.1, 1, 10, 100]

    for q_scale in q_scales:
        for r_scale in r_scales:
            kf = create_kalman_filter()
            kf.Q *= q_scale
            kf.R *= r_scale

            timestamps, estimated_positions, true_relative_positions = run_kalman_filter(lidar_data, rssi_data, true_positions, self_positions, kf)
            
            rmse = np.sqrt(np.mean(np.sum((estimated_positions - true_relative_positions)**2, axis=1)))
            
            results.append((q_scale, r_scale, rmse))
            
            if rmse < best_rmse:
                best_rmse = rmse
                best_q = q_scale
                best_r = r_scale

    print(f"Best Q scale: {best_q}, Best R scale: {best_r}, Best RMSE: {best_rmse}")
    return best_q, best_r, results

def plot_optimization_results(results):
    q_scales = sorted(set(result[0] for result in results))
    r_scales = sorted(set(result[1] for result in results))
    rmse_matrix = np.zeros((len(q_scales), len(r_scales)))

    for q_scale, r_scale, rmse in results:
        i = q_scales.index(q_scale)
        j = r_scales.index(r_scale)
        rmse_matrix[i, j] = rmse

    plt.figure(figsize=(10, 8))
    plt.imshow(rmse_matrix, cmap='viridis', aspect='auto', origin='lower')
    plt.colorbar(label='RMSE')
    plt.xlabel('R Scale')
    plt.ylabel('Q Scale')
    plt.title('RMSE for different Q and R scales')
    plt.xticks(range(len(r_scales)), r_scales)
    plt.yticks(range(len(q_scales)), q_scales)
    for i in range(len(q_scales)):
        for j in range(len(r_scales)):
            plt.text(j, i, f'{rmse_matrix[i, j]:.2f}', ha='center', va='center', color='w')
    plt.tight_layout()
    plt.show()

def plot_results(timestamps, estimated_positions, true_positions):
    fig, axs = plt.subplots(2, 2, figsize=(15, 15))
    
    # X Position over Time
    axs[0, 0].plot(timestamps, estimated_positions[:, 0], label='Estimated X')
    axs[0, 0].plot(timestamps, true_positions[:, 0], label='True X')
    axs[0, 0].set_xlabel('Time')
    axs[0, 0].set_ylabel('X Position')
    axs[0, 0].legend()
    axs[0, 0].set_title('X Position over Time')
    
    # Y Position over Time
    axs[0, 1].plot(timestamps, estimated_positions[:, 1], label='Estimated Y')
    axs[0, 1].plot(timestamps, true_positions[:, 1], label='True Y')
    axs[0, 1].set_xlabel('Time')
    axs[0, 1].set_ylabel('Y Position')
    axs[0, 1].legend()
    axs[0, 1].set_title('Y Position over Time')
    
    # X Error over Time
    x_error = estimated_positions[:, 0] - true_positions[:, 0]
    axs[1, 0].plot(timestamps, x_error)
    axs[1, 0].set_xlabel('Time')
    axs[1, 0].set_ylabel('X Error')
    axs[1, 0].set_title('X Prediction Error over Time')
    
    # Y Error over Time
    y_error = estimated_positions[:, 1] - true_positions[:, 1]
    axs[1, 1].plot(timestamps, y_error)
    axs[1, 1].set_xlabel('Time')
    axs[1, 1].set_ylabel('Y Error')
    axs[1, 1].set_title('Y Prediction Error over Time')
    
    plt.tight_layout()
    plt.show()


lidar_data = read_json_file('processed_data_151.json')
rssi_data = read_json_file('rssi_distance.json')
true_positions = read_json_file('vehicle_147_processed.json')
self_positions = read_json_file('vehicle_151_processed.json')

best_q, best_r, optimization_results = optimize_kalman_filter(lidar_data, rssi_data, true_positions, self_positions)

plot_optimization_results(optimization_results)

best_kf = create_kalman_filter()
best_kf.Q *= best_q
best_kf.R *= best_r

timestamps, estimated_positions, true_relative_positions = run_kalman_filter(lidar_data, rssi_data, true_positions, self_positions, best_kf)


plot_results(timestamps, estimated_positions, true_relative_positions)