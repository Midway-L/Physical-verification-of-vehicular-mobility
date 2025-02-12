import numpy as np
from scipy.linalg import inv
import json
import matplotlib.pyplot as plt
from math import cos, sin, radians

class KalmanFilter:
    def __init__(self, dt, std_acc, x_std_meas, y_std_meas):
        
        # 定义状态向量：[x, y, vx, vy, ax, ay]
        # 定义状态转移矩阵A
        self.A = np.array([[1, 0, dt, 0, 0.5*dt**2, 0],
                           [0, 1, 0, dt, 0, 0.5*dt**2],
                           [0, 0, 1, 0, dt, 0],
                           [0, 0, 0, 1, 0, dt],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1]])
        
        # 定义测量矩阵H
        self.H = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1]])
        
        # 定义过程噪声协方差矩阵Q
        self.Q = np.eye(6) * std_acc**2
        
        # 定义测量噪声协方差矩阵R
        self.R = np.array([[x_std_meas**2, 0, 0, 0],
                           [0, y_std_meas**2, 0, 0],
                           [0, 0, 0.1**2, 0],
                           [0, 0, 0, 0.1**2]])
        
        # 初始化状态估计向量
        self.x = np.zeros((6, 1))
        
        # 初始化误差协方差矩阵P
        self.P = np.eye(6)

    def predict(self):
        # 预测
        self.x = np.dot(self.A, self.x)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x

    def update(self, z):
        # 更新
        y = z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(6)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)
        return self.x

def load_true_positions(file_path):
    true_positions = {}
    with open(file_path, 'r') as f:
        for line in f:
            data = json.loads(line)
            true_positions[data['timestamp']] = (data['x'], data['y'])
    return true_positions

def load_rssi_distances(file_path):
    rssi_distances = {}
    with open(file_path, 'r') as f:
        for line in f:
            data = json.loads(line)
            rssi_distances[data['timestamp']] = data['distance']
    return rssi_distances

def combine_measurements(lidar_pos, rssi_pos, lidar_weight, rssi_weight):
    combined_x = (lidar_pos[0] * lidar_weight + rssi_pos[0] * rssi_weight) / (lidar_weight + rssi_weight)
    combined_y = (lidar_pos[1] * lidar_weight + rssi_pos[1] * rssi_weight) / (lidar_weight + rssi_weight)
    return combined_x, combined_y


""" def combine_lidar_rssi(lidar_x, lidar_y, rssi_distance, angle):
    # 使用RSSI距离和角度计算相对x和y位置
    rssi_x = rssi_distance * cos(radians(angle))
    rssi_y = rssi_distance * sin(radians(angle))
    
    # 加权平均，可以根据实际情况调整权重
    weight_lidar = 0.9
    weight_rssi = 0.1
    combined_x = weight_lidar * lidar_x + weight_rssi * rssi_x
    combined_y = weight_lidar * lidar_y + weight_rssi * rssi_y
    
    return combined_x, combined_y """

def process_data_with_kalman(input_file, true_position_file, rssi_file, output_file):
    # 读取处理过的数据
    data = []
    with open(input_file, 'r') as f:
        for line in f:
            data.append(json.loads(line))

    # 读取真实位置数据
    true_positions = load_true_positions(true_position_file)

    # 读取RSSI距离数据
    rssi_distances = load_rssi_distances(rssi_file)

    # 初始化卡尔曼滤波器
    dt = 0.1  # 时间步长
    std_acc = 0.01  # 过程噪声标准差
    x_std_meas = 1  # x位置测量噪声标准差
    y_std_meas = 1 # y位置测量噪声标准差
    kf_main = KalmanFilter(dt, std_acc, x_std_meas, y_std_meas)
    kf_lidar = KalmanFilter(dt, std_acc, x_std_meas, y_std_meas)
    kf_rssi = KalmanFilter(dt, std_acc, x_std_meas, y_std_meas)

    # 用于存储结果的列表
    timestamps = []
    true_positions_list = []
    kalman_positions = []
    target_positions = []
    lidar_positions = []
    rssi_positions = []

    
    weight_lidar = 0.9
    weight_rssi = 0.1

    # 处理每个数据点
    for point in data:
        timestamp = point['timestamp']
        x = point['location_x']
        y = point['location_y']
        ax = point['acc_x']
        ay = point['acc_y']
        lidar_x = point['avg_x']
        lidar_y = point['avg_y']
        angle = point.get('angle', 0)  

        if all(v is not None for v in [x, y, ax, ay, lidar_x, lidar_y]):
            # 获取RSSI距离，如果没有则使用None
            rssi_distance = rssi_distances.get(timestamp)

            # 计算RSSI位置
            if rssi_distance is not None:
                rssi_x = rssi_distance * cos(radians(angle))
                rssi_y = rssi_distance * sin(radians(angle))
            else:
                rssi_x, rssi_y = None, None

            # 对Lidar数据应用卡尔曼滤波
            kf_lidar.predict()
            lidar_measurement = np.array([[x + lidar_x], [y + lidar_y], [ax], [ay]])
            lidar_updated = kf_lidar.update(lidar_measurement)
            lidar_filtered_x, lidar_filtered_y = lidar_updated[0][0], lidar_updated[1][0]

            # 对RSSI数据应用卡尔曼滤波
            if rssi_x is not None and rssi_y is not None:
                kf_rssi.predict()
                rssi_measurement = np.array([[x + rssi_x], [y + rssi_y], [ax], [ay]])
                rssi_updated = kf_rssi.update(rssi_measurement)
                rssi_filtered_x, rssi_filtered_y = rssi_updated[0][0], rssi_updated[1][0]
            else:
                rssi_filtered_x, rssi_filtered_y = None, None

            # 融合Lidar和RSSI的结果
            if rssi_filtered_x is not None and rssi_filtered_y is not None:
                target_x, target_y = combine_measurements(
                    (lidar_filtered_x, lidar_filtered_y),
                    (rssi_filtered_x, rssi_filtered_y),
                    weight_lidar, weight_rssi
                )
            else:
                target_x, target_y = lidar_filtered_x, lidar_filtered_y

            # 主卡尔曼滤波器处理
            kf_main.predict()
            main_measurement = np.array([[target_x], [target_y], [ax], [ay]])
            main_updated = kf_main.update(main_measurement)

            # 存储结果
            timestamps.append(timestamp)
            true_positions_list.append(true_positions.get(timestamp, (None, None)))
            kalman_positions.append((main_updated[0][0], main_updated[1][0]))
            target_positions.append((target_x, target_y))
            lidar_positions.append((lidar_filtered_x, lidar_filtered_y))
            rssi_positions.append((rssi_filtered_x, rssi_filtered_y) if rssi_filtered_x is not None else (None, None))

    # 将结果写入输出文件
    with open(output_file, 'w') as f:
        for t, true, kalman, target, lidar, rssi in zip(timestamps, true_positions_list, kalman_positions, target_positions, lidar_positions, rssi_positions):
            json.dump({
                'timestamp': t,
                'true_x': true[0],
                'true_y': true[1],
                'predicted_x': kalman[0],
                'predicted_y': kalman[1],
                'target_x': target[0],
                'target_y': target[1],
                'lidar_x': lidar[0],
                'lidar_y': lidar[1],
                'rssi_x': rssi[0],
                'rssi_y': rssi[1]
            }, f)
            f.write('\n')

    # 绘制结果
    true_x, true_y = zip(*[pos for pos in true_positions_list if pos[0] is not None])
    predicted_x, predicted_y = zip(*kalman_positions)
    lidar_x, lidar_y = zip(*[(pos[0], pos[1]) for pos in lidar_positions if pos[0] is not None])
    rssi_x, rssi_y = zip(*[(pos[0], pos[1]) for pos in rssi_positions if pos[0] is not None])

    plt.figure(figsize=(20, 16))

    # X轴位置比较
    plt.subplot(2, 2, 1)
    plt.plot(timestamps, true_x, label='True X')
    plt.plot(timestamps, predicted_x, label='Predicted X')
    plt.plot(timestamps, lidar_x, label='Lidar X')
    plt.plot(timestamps, rssi_x, label='RSSI X')
    plt.legend()
    plt.title('X Position Comparison')
    plt.xlabel('Timestamp')
    plt.ylabel('X Position')
    plt.grid(True)

    # Y轴位置比较
    plt.subplot(2, 2, 2)
    plt.plot(timestamps, true_y, label='True Y')
    plt.plot(timestamps, predicted_y, label='Predicted Y')
    plt.plot(timestamps, lidar_y, label='Lidar Y')
    plt.plot(timestamps, rssi_y, label='RSSI Y')
    plt.legend()
    plt.title('Y Position Comparison')
    plt.xlabel('Timestamp')
    plt.ylabel('Y Position')
    plt.grid(True)

    # X-Y平面位置比较
    plt.subplot(2, 2, 3)
    plt.plot(true_x, true_y, label='True Position')
    plt.plot(predicted_x, predicted_y, label='Predicted Position')
    plt.plot(lidar_x, lidar_y, label='Lidar Position')
    plt.plot(rssi_x, rssi_y, label='RSSI Position')
    plt.legend()
    plt.title(f'X-Y Position Comparison\nLidar Weight: {weight_lidar}, RSSI Weight: {weight_rssi}')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.grid(True)

    # 误差分析
    plt.subplot(2, 2, 4)
    errors = np.sqrt(np.array([(px-tx)**2 + (py-ty)**2 for (px, py), (tx, ty) in zip(kalman_positions, true_positions_list) if tx is not None]))
    plt.plot(timestamps[:len(errors)], errors)
    plt.title('Prediction Error Over Time')
    plt.xlabel('Timestamp')
    plt.ylabel('Error (units)')
    plt.grid(True)

    # 计算和显示平均误差和最大误差
    avg_error = np.mean(errors)
    max_error = np.max(errors)
    plt.text(0.05, 0.95, f'Average Error: {avg_error:.2f}\nMax Error: {max_error:.2f}', 
             transform=plt.gca().transAxes, verticalalignment='top')

    plt.tight_layout()
    plt.savefig('double_kalman_position_comparison.png')
    plt.close()

    print(f"Processed data with Kalman filter and saved to {output_file}")
    print(f"Position comparison plot saved as position_comparison.png")
    print(f"Average Error: {avg_error:.2f}")
    print(f"Max Error: {max_error:.2f}")


# 使用示例
process_data_with_kalman('processed_data_151.json', 'vehicle_147_processed.json', 'rssi_distance.json', 'double_kalman_filtered_predict_for_147.json')