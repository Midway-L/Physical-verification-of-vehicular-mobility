import numpy as np
import json
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
from scipy.linalg import cholesky
from scipy.stats import circstd
from matplotlib.gridspec import GridSpec

class SimpleEKF:
    def __init__(self, dt, init_x, init_P):
        self.dt = dt
        self.x = np.array([init_x[0], init_x[1], 0])  # [rx, ry, theta]
        self.P = np.diag([init_P[0,0], init_P[1,1], 0.1])  # 初始不确定性
        
       # 调整过程噪声和测量噪声
        self.Q = np.diag([0.1, 0.01, 0.01])  # 添加角度的过程噪声
        self.R = np.diag([0.1, 0.01, 0.1, 0.01])  # [rx, ry, distance, angle]


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
                calculate_angle(rx, ry)
            ])
        
        epsilon = 1e-10
        denominator = np.sqrt(self.x[0]**2 + self.x[1]**2) + epsilon
        
        H = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [self.x[0]/denominator, self.x[1]/denominator, 0],
            [-self.x[1]/(denominator**2), self.x[0]/(denominator**2), 1]
        ])
        
        z_pred = h(self.x)
        y = z - z_pred
        y[3] = angle_difference(z[3], z_pred[3])  # 使用角度差异函数
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.x = self.x + K @ y
        self.x[0] = abs(self.x[0])
        self.x[2] = np.clip(self.x[2], -np.pi/2, np.pi/2)  # 确保估计的角度在 -π/2 到 π/2 之间
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

def calculate_angle(x, y):
    return np.arctan2(y, x)

def angle_difference(angle1, angle2):
    diff = angle1 - angle2
    return np.arctan2(np.sin(diff), np.cos(diff))

def calculate_true_relative_angle(ego_compass, other_compass):
    # 计算真实的相对角度
    relative_angle = normalize_angle(other_compass - ego_compass)
    return relative_angle

def global_to_local(global_x, global_y, ego_x, ego_y, ego_compass):
    
    # 计算相对位置
    dx = global_x - ego_x
    dy = global_y - ego_y
    
    # 在CARLA中，compass的0对应于-Y方向，π/2对应于+X方向
    # 我们需要将compass转换为标准数学坐标系中的角度
    theta = -(ego_compass - np.pi/2)
    
    # 执行坐标旋转
    local_x = dx * np.cos(theta) + dy * np.sin(theta)
    local_y = -dx * np.sin(theta) + dy * np.cos(theta)

    local_x = abs(local_x)
    
    return local_x, local_y

def process_lidar_data(lidar_points, ego_x, ego_y, ego_heading, timestamp):
    
    # 过滤掉无效的点
    valid_points = [point for point in lidar_points if np.all(np.isfinite(point))]
    
    if len(valid_points) < 2:  # 需要至少两个点来计算协方差
        return None  # 或者返回一个表示无效数据的特殊值

    clustering = DBSCAN(eps=0.5, min_samples=5).fit(valid_points)
    labels = clustering.labels_
    
    unique_labels, counts = np.unique(labels, return_counts=True)
    if len(unique_labels) == 1 and unique_labels[0] == -1:  # 所有点都被视为噪声
        return None
    largest_cluster = unique_labels[np.argmax(counts[unique_labels != -1])]
    
    cluster_points = np.array(lidar_points)[labels == largest_cluster]
    
    if len(cluster_points) < 2:
        return None
        
    rx, ry = np.mean(cluster_points, axis=0)
    relative_distance = np.sqrt(rx**2 + ry**2)
    relative_angle = calculate_angle(rx, ry)

    # 进行误差分析
    error_analysis = analyze_lidar_error(cluster_points, rx, ry, relative_distance, relative_angle)
    
    return {
        'timestamp': timestamp,
        'rx': abs(rx),
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


def plot_results(results, weights):
    timestamps = [r['timestamp'] for r in results]
    true_distances = [r['true_distance'] for r in results]
    estimated_distances = [r['estimated_distance'] for r in results]
    true_angles = [r['true_angle'] for r in results]
    estimated_angles = [r['estimated_angle'] for r in results]
    #true_rx = [r['true_rx'] for r in results]
    #true_ry = [r['true_ry'] for r in results]
    estimated_rx = [r['estimated_rx'] for r in results]
    estimated_ry = [r['estimated_ry'] for r in results]
    rx_error = [r['estimated_rx'] - r['true_rx'] for r in results]
    ry_error = [r['estimated_ry'] - r['true_ry'] for r in results]
    distance_error = [r['estimated_distance'] - r['true_distance'] for r in results]
    angle_error = [angle_difference(r['estimated_angle'], r['true_angle']) for r in results]

    # 创建5个独立的图表
    fig_distance, ax_distance = plt.subplots(figsize=(12, 6))
    fig_angle, ax_angle = plt.subplots(figsize=(12, 6))
    fig_xy, ax_xy = plt.subplots(figsize=(12, 10))
    fig_errors, (ax_distance_error, ax_angle_error) = plt.subplots(2, 1, figsize=(12, 10))
    #fig_xy_error, ax_xy_error = plt.subplots(figsize=(12, 6))
    fig_weights, ax_weights = plt.subplots(figsize=(12, 6))

    # 距离比较图
    ax_distance.plot(timestamps, true_distances, label='True Distance')
    ax_distance.plot(timestamps, estimated_distances, label='Estimated Distance')
    ax_distance.set_title('True vs Estimated Distance')
    ax_distance.set_xlabel('Timestamp')
    ax_distance.set_ylabel('Distance (m)')
    ax_distance.legend()
    ax_distance.grid(True)
    
    rms_distance = np.sqrt(np.mean(np.array(distance_error)**2))
    ax_distance.text(0.02, 0.95, f'RMS Distance Error: {rms_distance:.2f} m', transform=ax_distance.transAxes)

    # 角度比较图
    ax_angle.plot(timestamps, true_angles, label='True Angle')
    ax_angle.plot(timestamps, estimated_angles, label='Estimated Angle')
    ax_angle.set_title('Angle Comparison')
    ax_angle.set_xlabel('Timestamp')
    ax_angle.set_ylabel('Angle (rad)')
    ax_angle.legend()
    ax_angle.grid(True)
    
    rms_angle = np.sqrt(np.mean(np.array(angle_error)**2))
    ax_angle.text(0.02, 0.95, f'RMS Angle Error: {rms_angle:.2f} rad', transform=ax_angle.transAxes)

    # RX和RY
    ax_xy.plot(timestamps, estimated_rx, 'b-', label='Estimated RX')
    ax_xy.plot(timestamps, estimated_ry, 'r-', label='Estimated RY')
    ax_xy.set_title('Estimated RX and RY over Time')
    ax_xy.set_xlabel('Timestamp')
    ax_xy.set_ylabel('Distance (m)')
    ax_xy.legend()
    ax_xy.grid(True)
    
    # 距离和角度误差图
    ax_distance_error.plot(timestamps, distance_error)
    ax_distance_error.set_title('Distance Error')
    ax_distance_error.set_xlabel('Timestamp')
    ax_distance_error.set_ylabel('Error (m)')
    ax_distance_error.grid(True)
    
    mean_distance_error = np.mean(distance_error)
    std_distance_error = np.std(distance_error)
    ax_distance_error.text(0.02, 0.95, f'Mean: {mean_distance_error:.2f} m\nStd: {std_distance_error:.2f} m', transform=ax_distance_error.transAxes)

    ax_angle_error.plot(timestamps, angle_error)
    ax_angle_error.set_title('Angle Error')
    ax_angle_error.set_xlabel('Timestamp')
    ax_angle_error.set_ylabel('Error (rad)')
    ax_angle_error.grid(True)
    
    mean_angle_error = np.mean(angle_error)
    std_angle_error = np.std(angle_error)
    ax_angle_error.text(0.02, 0.95, f'Mean: {mean_angle_error:.2f} rad\nStd: {std_angle_error:.2f} rad', transform=ax_angle_error.transAxes)

    """ # X和Y误差图
    ax_xy_error.plot(timestamps, rx_error, label='X Error')
    ax_xy_error.plot(timestamps, ry_error, label='Y Error')
    ax_xy_error.set_title('X and Y Errors')
    ax_xy_error.set_xlabel('Timestamp')
    ax_xy_error.set_ylabel('Error (m)')
    ax_xy_error.legend()
    ax_xy_error.grid(True)
    
    rms_x = np.sqrt(np.mean(np.array(rx_error)**2))
    rms_y = np.sqrt(np.mean(np.array(ry_error)**2))
    ax_xy_error.text(0.02, 0.95, f'RMS X Error: {rms_x:.2f} m\nRMS Y Error: {rms_y:.2f} m', transform=ax_xy_error.transAxes)
    """
    # 测量分量的权重
    weights = np.array(weights)
    ax_weights.plot(timestamps, weights[:, 0], label='Lidar X')
    ax_weights.plot(timestamps, weights[:, 1], label='Lidar Y')
    ax_weights.plot(timestamps, weights[:, 2], label='RSSI Distance')
    ax_weights.plot(timestamps, weights[:, 3], label='Angle')
    ax_weights.set_title('Measurement Weights')
    ax_weights.set_xlabel('Timestamp')
    ax_weights.set_ylabel('Weight')
    ax_weights.legend()
    ax_weights.grid(True)

    # 调整布局并显示图表
    for fig in [fig_distance, fig_angle, fig_xy, fig_errors, fig_weights]:
        fig.tight_layout()
    plt.show()

    # 打印统计信息
    print(f"RMS Distance Error: {rms_distance:.2f} m")
    print(f"RMS Angle Error: {rms_angle:.2f} rad")

    # 计算相关系数
    distance_corr = np.corrcoef(true_distances, estimated_distances)[0, 1]
    angle_corr = np.corrcoef(true_angles, estimated_angles)[0, 1]
    print(f"Distance Correlation: {distance_corr:.4f}")
    print(f"Angle Correlation: {angle_corr:.4f}")

def main():
    # 加载数据
    data_151 = load_json_file('processed_data_290.json')
    data_147 = load_json_file('processed_data_286.json')
    lidar_data = load_json_file('filtered_lidar_data_290.json')
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
            max(0.1, distance_error_ratio),
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
        
        true_angle = calculate_true_relative_angle(ego_data['imu_compass'], other_data['imu_compass'])
        
        estimated_angle = calculate_angle(ekf.x[0], ekf.x[1])

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

    plot_results(results, weights)


if __name__ == "__main__":
    main()
