import json
import numpy as np
import matplotlib.pyplot as plt

def read_json_file(file_path):
    with open(file_path, 'r') as file:
        return [json.loads(line) for line in file]

class ExtendedKalmanFilter:
    def __init__(self, dim_x, dim_z):
        self.dim_x = dim_x
        self.dim_z = dim_z
        
        self.x = np.zeros((dim_x, 1))  # 状态
        self.P = np.eye(dim_x) * 1000  # 状态协方差
        self.Q = np.eye(dim_x)         # 过程噪声协方差
        self.R = np.eye(dim_z)         # 测量噪声协方差

    def predict(self, dt):
        # 非线性状态转移函数
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        self.x = np.dot(F, self.x)
        self.P = np.dot(np.dot(F, self.P), F.T) + self.Q

    def update(self, z):
        H = self.jacobian_h(self.x)
        y = z - self.h(self.x)
        S = np.dot(np.dot(H, self.P), H.T) + self.R

        # 添加数值稳定性检查
        if np.linalg.det(S) < 1e-12:  # 如果行列式接近零
            print("Warning: S matrix is close to singular. Skipping update step.")
            return
        
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
        
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.dim_x)
        self.P = np.dot((I - np.dot(K, H)), self.P)

    def h(self, x):
        # x[0, 0] 是 x 方向的相对位置
        # x[1, 0] 是 y 方向的相对位置
        lidar_x, lidar_y = x[0, 0], x[1, 0]
        rssi_distance = np.sqrt(x[0, 0]**2 + x[1, 0]**2)
        return np.array([[lidar_x], [lidar_y], [rssi_distance]])

    def jacobian_h(self, x):
        x_val, y_val = x[0, 0], x[1, 0]
        r = np.sqrt(x_val**2 + y_val**2)
        # 添加数值稳定性检查
        if r < 1e-12:  # 如果r接近零
            print("Warning: r is close to zero. Using small non-zero value.")
            r = 1e-12
        return np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [x_val/r, y_val/r, 0, 0]
        ])

def create_extended_kalman_filter():
    ekf = ExtendedKalmanFilter(dim_x=4, dim_z=3)
    
    # 设置过程噪声协方差
    ekf.Q = np.diag([0.1, 0.1, 0.2, 0.2])
    
    # 设置测量噪声协方差
    # 激光雷达误差约为2m，RSSI误差约为10m
    ekf.R = np.diag([2**2, 2**2, 10**2])
    
    return ekf

def run_extended_kalman_filter(lidar_data, rssi_data, true_positions, self_positions, ekf):
    timestamps = []
    estimated_positions = []
    true_relative_positions = []
    
    lidar_index = 0
    rssi_index = 0
    last_timestamp = None
    
    for true_pos, self_pos in zip(true_positions, self_positions):
        timestamp = true_pos['timestamp']
        
        while lidar_index < len(lidar_data) and lidar_data[lidar_index]['timestamp'] <= timestamp:
            lidar_measurement = lidar_data[lidar_index]
            lidar_index += 1
        
        while rssi_index < len(rssi_data) and rssi_data[rssi_index]['timestamp'] <= timestamp:
            rssi_measurement = rssi_data[rssi_index]
            rssi_index += 1
        
        if 'avg_x' in lidar_measurement and 'avg_y' in lidar_measurement and 'distance' in rssi_measurement:
            lidar_x, lidar_y = lidar_measurement['avg_x'], lidar_measurement['avg_y']
            rssi_distance = rssi_measurement['distance']
            
            z = np.array([[lidar_x], [lidar_y], [rssi_distance]])
            
            if last_timestamp is not None:
                dt = timestamp - last_timestamp
                ekf.predict(dt)
            
            try:
                ekf.update(z)
            except np.linalg.LinAlgError as e:
                print(f"LinAlgError at timestamp {timestamp}: {e}")
                continue  # 跳过这次更新
            
            timestamps.append(timestamp)
            estimated_positions.append(ekf.x[:2].flatten())
            
            true_relative_x = abs(true_pos['x'] - self_pos['x'])
            true_relative_y = abs(true_pos['y'] - self_pos['y'])
            true_relative_positions.append([true_relative_x, true_relative_y])
            
            last_timestamp = timestamp
    
    return np.array(timestamps), np.array(estimated_positions), np.array(true_relative_positions)

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

# 主程序
lidar_data = read_json_file('processed_data_151.json')
rssi_data = read_json_file('rssi_distance.json')
true_positions = read_json_file('vehicle_147_processed.json')
self_positions = read_json_file('vehicle_151_processed.json')

# 创建并运行扩展卡尔曼滤波器
ekf = create_extended_kalman_filter()
timestamps, estimated_positions, true_relative_positions = run_extended_kalman_filter(lidar_data, rssi_data, true_positions, self_positions, ekf)

# 绘制结果
plot_results(timestamps, estimated_positions, true_relative_positions)

# 计算RMSE
rmse = np.sqrt(np.mean(np.sum((estimated_positions - true_relative_positions)**2, axis=1)))
print(f"RMSE: {rmse}")