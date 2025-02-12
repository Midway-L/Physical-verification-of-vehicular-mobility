import numpy as np
import json
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
from scipy.linalg import cholesky
from scipy.stats import circstd

class SimpleEKF:
    def __init__(self, dt, init_x, init_P):
        self.dt = dt
        self.x = np.array([init_x[0], init_x[1], 0])  # [rx, ry, theta]
        self.P = np.diag([init_P[0,0], init_P[1,1], 0.1])  # 添加角度的初始不确定性
        
       # 调整过程噪声和测量噪声
        self.Q = np.diag([0.1, 0.1, 0.01])  # 添加角度的过程噪声
        self.R = np.diag([0.1, 0.1, 0.1, 0.1])  # [rx, ry, distance, angle]

    def predict(self):
        # 使用恒定位置模型
        F = np.eye(3)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q
        self.ensure_positive_definite()

    def update(self, z):
        def h(x):
            rx, ry, theta = x
            return np.array([
                rx,
                ry,
                np.sqrt(rx**2 + ry**2),
                np.arctan2(ry, rx)
            ])
        
        epsilon = 1e-10
        denominator = np.sqrt(self.x[0]**2 + self.x[1]**2) + epsilon
        
        H = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [self.x[0]/denominator, self.x[1]/denominator, 0],
            [-self.x[1]/(denominator**2), self.x[0]/(denominator**2), 0]
        ])
        
        z_pred = h(self.x)
        y = z - z_pred
        y[3] = normalize_angle(y[3]) # 确保角度差在 -pi 到 pi 之间
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.x = self.x + K @ y
        self.x[2] = normalize_angle(self.x[2])  # 确保估计的角度在 -pi 到 pi 之间
        self.P = (np.eye(3) - K @ H) @ self.P
        self.ensure_positive_definite()
        
        return K # 返回卡尔曼增益以用于权重分析

    def ensure_positive_definite(self):
        """
        确保 P 矩阵是正定的
        """
        self.P = (self.P + self.P.T) / 2  # 确保对称性
        try:
            cholesky(self.P)
        except np.linalg.LinAlgError:
            # 如果不是正定的，进行修正
            eigvals, eigvecs = np.linalg.eigh(self.P)
            eigvals = np.maximum(eigvals, 1e-8)
            self.P = eigvecs @ np.diag(eigvals) @ eigvecs.T

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def global_to_local(global_x, global_y, ego_x, ego_y, ego_compass):
    # 计算相对位置
    dx = global_x - ego_x
    dy = global_y - ego_y
    
    # Carla的compass是顺时针的，我们需要转换为逆时针的角度
    # 同时，我们需要调整坐标系，使得车辆前方为x轴正方向
    theta = np.pi/2 - ego_compass
    
    # 进行坐标旋转
    local_x = dx * np.cos(theta) + dy * np.sin(theta)
    local_y = -dx * np.sin(theta) + dy * np.cos(theta)
    
    return local_x, local_y

def process_lidar_data(lidar_points, ego_x, ego_y, ego_heading, timestamp):
    
    # 过滤掉无效的点
    valid_points = [point for point in lidar_points if np.all(np.isfinite(point))]
    
    if len(valid_points) < 2:  # 需要至少两个点来计算协方差
        return None  # 或者返回一个表示无效数据的特殊值

    clustering = DBSCAN(eps=0.5, min_samples=5).fit(lidar_points)
    labels = clustering.labels_
    
    unique_labels, counts = np.unique(labels, return_counts=True)
    if len(unique_labels) == 1 and unique_labels[0] == -1:  # 所有点都被视为噪声
        return None
    largest_cluster = unique_labels[np.argmax(counts[unique_labels != -1])]
    
    cluster_points = np.array(lidar_points)[labels == largest_cluster]
    rx, ry = np.mean(cluster_points, axis=0)
    if len(cluster_points) < 2:
        return None
    
    relative_distance = np.sqrt(rx**2 + ry**2)
    relative_angle = np.arctan2(ry, rx)
    relative_angle = normalize_angle(relative_angle)

    # 进行误差分析
    error_analysis = analyze_lidar_error(cluster_points, rx, ry, relative_distance, relative_angle)
    
    return {
        'timestamp': timestamp,
        'rx': rx,
        'ry': ry,
        'distance': relative_distance,
        'angle': relative_angle,
        'error_analysis': error_analysis
    }

def analyze_lidar_error(cluster_points, rx, ry, relative_distance, relative_angle):
    try:
        covariance = np.cov(cluster_points.T)
        eigenvalues, _ = np.linalg.eig(covariance)
        point_spread = np.sqrt(np.sum(eigenvalues))
    except np.linalg.LinAlgError:
        # 如果出现线性代数错误，使用一个默认值
        point_spread = 1.0

    # 2. 距离相关误差分析
    distances = np.sqrt(np.sum(cluster_points**2, axis=1))
    distance_std = np.std(distances)
    distance_error_ratio = distance_std / relative_distance if relative_distance != 0 else 1.0

    # 3. 角度误差分析
    angles = np.arctan2(cluster_points[:, 1], cluster_points[:, 0])
    angle_std = circstd(angles)  # 使用圆形标准差

    # 4. 聚类质量评估
    cluster_density = len(cluster_points) / point_spread if point_spread != 0 else 1.0

    # 5. 计算几何稀疏度
    geometric_sparsity = np.max(distances) - np.min(distances)

    return {
        'point_spread': point_spread,
        'distance_std': distance_std,
        'distance_error_ratio': distance_error_ratio,
        'angle_std': angle_std,
        'cluster_density': cluster_density,
        'geometric_sparsity': geometric_sparsity
    }


def load_json_file(file_path):
    data_dict = {}
    with open(file_path, 'r') as f:
        for line in f:
            try:
                data = json.loads(line.strip())
                # 每个JSON对象都有一个唯一的timestamp字段
                timestamp = data['new_timestamp']
                data_dict[timestamp] = data
            except json.JSONDecodeError as e:
                print(f"Error decoding JSON in {file_path}: {e}")
            except KeyError as e:
                print(f"Missing key in JSON object: {e}")
    return data_dict


def main():
    # 加载数据
    data_151 = load_json_file('processed_data_151.json')
    data_147 = load_json_file('processed_data_147.json')
    lidar_data = load_json_file('filtered_lidar_data_151.json')
    rssi_data = load_json_file('improved_rssi_distance.json')

    # 初始化EKF
    init_x = np.array([0, 0, 0])  # 初始状态包括角度
    init_P = np.diag([100, 100, np.pi/2])  # 初始协方差包括角度不确定性
    ekf = SimpleEKF(dt=0.1, init_x=init_x, init_P=init_P)

    results = []
    weights = []

    for timestamp in sorted(data_151.keys()):
        if timestamp not in data_147 or timestamp not in lidar_data or timestamp not in rssi_data:
            continue

        ego_data = data_151[timestamp]
        other_data = data_147[timestamp]

        # 预测步骤
        ekf.predict()

        lidar_points = [point['point'][:2] for point in lidar_data[timestamp]['vehicle_lidar_data']]
        lidar_measurement = process_lidar_data(lidar_points, ego_data['x'], ego_data['y'], ego_data['imu_compass'], timestamp)

        if lidar_measurement is None:
            print(f"Warning: Invalid LiDAR measurement at timestamp {timestamp}")
            continue

        # 使用误差分析结果调整EKF的测量噪声
        point_spread = lidar_measurement['error_analysis']['point_spread']
        distance_error_ratio = lidar_measurement['error_analysis']['distance_error_ratio']
        angle_std = lidar_measurement['error_analysis']['angle_std']
        
        # 动态调整EKF的测量噪声
        ekf.R = np.diag([
            max(0.1, point_spread),
            max(0.1, point_spread),
            max(0.1, lidar_measurement['distance'] * distance_error_ratio),
            max(0.1, angle_std)
        ])

        # 获取RSSI数据
        rssi_distance = rssi_data[timestamp]['distance']

        # 构造测量向量
        z = np.array([lidar_measurement['rx'], lidar_measurement['ry'], rssi_distance, lidar_measurement['angle']])

        if np.all(np.isfinite(z)):
            K = ekf.update(z)
            weights.append(np.sum(np.abs(K), axis=0))
        else:
            print(f"警告: 时间戳 {timestamp} 的测量值无效，跳过此次更新")
            weights.append(np.zeros(4))


        # 计算真实的相对位置（局部坐标系）
        true_rx, true_ry = global_to_local(other_data['x'], other_data['y'], ego_data['x'], ego_data['y'], ego_data['imu_compass'])
        true_angle = np.arctan2(true_ry, true_rx)
        true_angle = normalize_angle(true_angle)
        
        estimated_angle = np.arctan2(ekf.x[1], ekf.x[0])
        estimated_angle = normalize_angle(estimated_angle)

        results.append({
            'timestamp': timestamp,
            'true_rx': true_rx,
            'true_ry': true_ry,
            'estimated_rx': ekf.x[0],
            'estimated_ry': ekf.x[1],
            'true_distance': np.sqrt(true_rx**2 + true_ry**2),
            'estimated_distance': np.sqrt(ekf.x[0]**2 + ekf.x[1]**2),
            'true_angle': true_angle,
            'estimated_angle': estimated_angle
        })

# 提取数据用于绘图
    timestamps = [r['timestamp'] for r in results]
    true_distances = [r['true_distance'] for r in results]
    estimated_distances = [r['estimated_distance'] for r in results]
    true_angles = [r['true_angle'] for r in results]
    estimated_angles = [r['estimated_angle'] for r in results]
    rx_error = [r['estimated_rx'] - r['true_rx'] for r in results]
    ry_error = [r['estimated_ry'] - r['true_ry'] for r in results]
    distance_error = [r['estimated_distance'] - r['true_distance'] for r in results]
    angle_error = [normalize_angle(r['estimated_angle'] - r['true_angle']) for r in results]

    plt.figure(figsize=(15, 25))
    
    # 距离比较图
    plt.subplot(5, 1, 1)
    plt.plot(timestamps, true_distances, label='True Distance')
    plt.plot(timestamps, estimated_distances, label='Estimated Distance')
    plt.title('True vs Estimated Distance')
    plt.xlabel('Timestamp')
    plt.ylabel('Distance (m)')
    plt.legend()

    # 角度比较图
    plt.subplot(5, 1, 2)
    plt.plot(timestamps, true_angles, label='True Angle')
    plt.plot(timestamps, estimated_angles, label='Estimated Angle')
    plt.title('True vs Estimated Angle')
    plt.xlabel('Timestamp')
    plt.ylabel('Angle (rad)')
    plt.legend()

    # 距离误差图
    plt.subplot(5, 1, 3)
    plt.plot(timestamps, distance_error)
    plt.title('Distance Error')
    plt.xlabel('Timestamp')
    plt.ylabel('Error (m)')

    # 角度误差图
    plt.subplot(5, 1, 4)
    plt.plot(timestamps, angle_error)
    plt.title('Angle Error')
    plt.xlabel('Timestamp')
    plt.ylabel('Error (rad)')

    # 测量分量的权重
    weights = np.array(weights)
    plt.subplot(5, 1, 5)
    plt.plot(timestamps, weights[:, 0], label='Lidar X')
    plt.plot(timestamps, weights[:, 1], label='Lidar Y')
    plt.plot(timestamps, weights[:, 2], label='RSSI Distance')
    plt.plot(timestamps, weights[:, 3], label='Angle')
    plt.title('Measurement Weights')
    plt.xlabel('Timestamp')
    plt.ylabel('Weight')
    plt.legend()

    plt.tight_layout()
    plt.show()

    # 打印统计信息
    print(f"RMS X Error: {np.sqrt(np.mean(np.array(rx_error)**2)):.2f} m")
    print(f"RMS Y Error: {np.sqrt(np.mean(np.array(ry_error)**2)):.2f} m")
    print(f"RMS Distance Error: {np.sqrt(np.mean(np.array(distance_error)**2)):.2f} m")
    print(f"RMS Angle Error: {np.sqrt(np.mean(np.array(angle_error)**2)):.2f} rad")

    # 额外添加：计算相关系数
    distance_corr = np.corrcoef(true_distances, estimated_distances)[0, 1]
    angle_corr = np.corrcoef(true_angles, estimated_angles)[0, 1]
    print(f"Distance Correlation: {distance_corr:.4f}")
    print(f"Angle Correlation: {angle_corr:.4f}")

if __name__ == "__main__":
    main()