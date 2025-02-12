import numpy as np
import json
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from scipy.linalg import cholesky
from scipy.stats import circstd
import pickle
import random

# Constants
DETECTION_THRESHOLD = 1  # 1 meters threshold for detection
ANGLE_THRESHOLD = 0.1  # 0.1 radians threshold for angle detection

class SimpleEKF:
    def __init__(self, dt, init_x, init_P):
        self.dt = dt
        self.x = np.array([init_x[0], init_x[1], 0])  # [rx, ry, theta]
        self.P = np.diag([init_P[0,0], init_P[1,1], 0.1])  # Initial uncertainty
        self.Q = np.diag([0.1, 0.01, 0.01])  # Process noise
        self.R = np.diag([0.1, 0.01, 0.1, 0.01])  # Measurement noise [rx, ry, distance, angle]

    def predict(self):
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
            [-self.x[1]/(denominator**2), self.x[0]/(denominator**2), 1]
        ])
        
        z_pred = h(self.x)
        y = z - z_pred
        y[3] = np.arctan2(np.sin(y[3]), np.cos(y[3]))  # Normalize angle difference
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.x = self.x + K @ y
        self.x[0] = abs(self.x[0])
        self.x[2] = np.clip(self.x[2], -np.pi/2, np.pi/2)
        self.P = (np.eye(3) - K @ H) @ self.P
        self.ensure_positive_definite()
        
        return K

    def ensure_positive_definite(self):
        self.P = (self.P + self.P.T) / 2
        try:
            cholesky(self.P)
        except np.linalg.LinAlgError:
            eigvals, eigvecs = np.linalg.eigh(self.P)
            eigvals = np.maximum(eigvals, 1e-8)
            self.P = eigvecs @ np.diag(eigvals) @ eigvecs.T

def load_json_file(file_path):
    data_dict = {}
    with open(file_path, 'r') as f:
        for line in f:
            try:
                data = json.loads(line.strip())
                timestamp = data['new_timestamp']
                data_dict[timestamp] = data
            except json.JSONDecodeError as e:
                print(f"Error decoding JSON in {file_path}: {e}")
            except KeyError as e:
                print(f"Missing key in JSON object: {e}")
    return data_dict

def load_ekf_model(filename='ekf_model.pkl'):
    with open(filename, 'rb') as f:
        model_data = pickle.load(f)
    
    ekf = SimpleEKF(dt=model_data['dt'], 
                    init_x=model_data['x'], 
                    init_P=model_data['P'])
    ekf.Q = model_data['Q']
    ekf.R = model_data['R']
    return ekf

def calculate_angle(x, y):
    return np.arctan2(y, x)

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def calculate_true_relative_angle(ego_compass, other_compass):
    # 计算真实的相对角度
    relative_angle = normalize_angle(other_compass - ego_compass)
    return relative_angle

def global_to_local(global_x, global_y, ego_x, ego_y, ego_compass, target_compass):
    # 计算相对位置
    dx = global_x - ego_x
    dy = global_y - ego_y
    
    # 计算距离
    distance = np.sqrt(dx**2 + dy**2)
    
    relative_angle = normalize_angle(target_compass - ego_compass)
    
    # 计算局部坐标
    local_x = distance * np.cos(relative_angle)
    local_y = distance * np.sin(relative_angle)
    
    return local_x, local_y

def process_lidar_data(lidar_points, ego_x, ego_y, ego_heading, timestamp):
    
    # 过滤掉无效的点
    valid_points = [point for point in lidar_points if np.all(np.isfinite(point))]
    
    if len(valid_points) < 2:  # 需要至少两个点来计算协方差
        return None  # 或者返回一个表示无效数据的特殊值

    clustering = DBSCAN(eps=1.5, min_samples=3).fit(np.array(valid_points))
    labels = clustering.labels_
    
    unique_labels, counts = np.unique(labels, return_counts=True)
    if len(unique_labels) == 1 and unique_labels[0] == -1:  # 所有点都被视为噪声
        return None
    
    largest_cluster = unique_labels[np.argmax(counts[unique_labels != -1])]
    
    cluster_points = np.array(lidar_points)[labels == largest_cluster]
    
    if len(cluster_points) < 2:
        return None
    
    rx, ry, _ = np.mean(cluster_points, axis=0)
    relative_distance = np.sqrt(rx**2 + ry**2)
    relative_angle = calculate_angle(rx, ry)
 
    return {
        'timestamp': timestamp,
        'rx': abs(rx),
        'ry': ry,
        'distance': relative_distance,
        'angle': relative_angle
    }


def constant_offset_attack(true_x, true_y, offset_x, offset_y):
    #固定偏移攻击
    return true_x + offset_x, true_y + offset_y

def random_offset_attack(true_x, true_y, max_offset, timestamp, last_change_time, current_offset):
    #随机偏移攻击,每5秒更新一次,持续5秒
    if timestamp - last_change_time >= 10:  # 每10秒更新一次
        current_offset = (np.random.uniform(-max_offset, max_offset),
                          np.random.uniform(-max_offset, max_offset))
        last_change_time = timestamp
    return true_x + current_offset[0], true_y + current_offset[1], last_change_time, current_offset

""" def oscillating_attack(true_x, true_y, amplitude, frequency, time):
    #振荡攻击
    offset_x = amplitude * np.sin(2 * np.pi * frequency * time)
    offset_y = amplitude * np.cos(2 * np.pi * frequency * time)
    return true_x + offset_x, true_y + offset_y """

""" def gradual_drift_attack(true_x, true_y, drift_rate, time):
    #逐渐漂移攻击
    offset_x = drift_rate * time
    offset_y = drift_rate * time
    return true_x + offset_x, true_y + offset_y """

def random_jump_attack(true_x, true_y, jump_probability, max_jump, timestamp, last_jump_time, current_jump):
    #随机跳跃攻击,跳跃后保持5秒
    if timestamp - last_jump_time >= 5:  # 距离上次跳跃已经过去5秒
        if np.random.random() < jump_probability:
            current_jump = (np.random.uniform(-max_jump, max_jump),
                            np.random.uniform(-max_jump, max_jump))
            last_jump_time = timestamp
    return true_x + current_jump[0], true_y + current_jump[1], last_jump_time, current_jump

def lane_change_attack(lidar_data, lane_width):
    #车道变换攻击
    fake_ry = lidar_data['ry'] + lane_width
    fake_distance = np.sqrt(lidar_data['rx']**2 + fake_ry**2)
    fake_angle = np.arctan2(fake_ry, lidar_data['rx'])
    return {
        'rx': lidar_data['rx'],
        'ry': fake_ry,
        'distance': fake_distance,
        'angle': fake_angle
    }

def detect_fake_position(ekf, claimed_x, claimed_y, lidar_measurement, rssi_distance, threshold, angle_threshold):
    ekf.predict()
    
    z = np.array([lidar_measurement['rx'], lidar_measurement['ry'], rssi_distance, lidar_measurement['angle']])
    
    ekf.update(z)
    
    predicted_distance = np.sqrt(ekf.x[0]**2 + ekf.x[1]**2)
    claimed_distance = np.sqrt(claimed_x**2 + claimed_y**2)
    
    distance_diff = abs(predicted_distance - claimed_distance)
    
    predicted_angle = np.arctan2(ekf.x[1], ekf.x[0])
    claimed_angle = np.arctan2(claimed_y, claimed_x)
    angle_diff = abs(normalize_angle(predicted_angle - claimed_angle))
    
    return (distance_diff > threshold) or (angle_diff > angle_threshold), distance_diff, angle_diff

def simulate_attack(attack_type, true_rx, true_ry, current_time, lidar_measurement, last_change_time, current_offset, last_jump_time, current_jump, attack_start_time):
    if attack_type == 'constant':
        return (true_rx + 3, true_ry + 3), last_change_time, current_offset, last_jump_time, current_jump
    elif attack_type == 'random':
        if current_time - last_change_time >= 5:
            current_offset = (random.uniform(-3, 3), random.uniform(-3, 3))
            last_change_time = current_time
        return (true_rx + current_offset[0], true_ry + current_offset[1]), last_change_time, current_offset, last_jump_time, current_jump
    elif attack_type == 'jump':
        if current_time - last_jump_time >= 5:
            current_jump = (random.uniform(-3, 3), random.uniform(-3, 3))
            last_jump_time = current_time
        return (true_rx + current_jump[0], true_ry + current_jump[1]), last_change_time, current_offset, last_jump_time, current_jump
    elif attack_type == 'lane':
        lane_width = 3.5  # meters
        return (true_rx, true_ry + lane_width), last_change_time, current_offset, last_jump_time, current_jump
    else:
        return (true_rx, true_ry), last_change_time, current_offset, last_jump_time, current_jump
    
def plot_results(results, attack_type):
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(16, 20))

    timestamps = [r['timestamp'] for r in results]
    true_distances = [r['true_distance'] for r in results]
    claimed_distances = [r['claimed_distance'] for r in results]
    estimated_distances = [r['estimated_distance'] for r in results]
    attack_detected = [r['attack_detected'] for r in results]
    true_angles = [r['true_angle'] for r in results]
    claimed_angles = [r['claimed_angle'] for r in results]
    estimated_angles = [r['estimated_angle'] for r in results]
    attack_periods = [r['attack_period'] for r in results]

    ax1.plot(timestamps, true_distances, label='True Distance', color='red')
    ax1.plot(timestamps, claimed_distances, label='Claimed Distance', color='green')
    ax1.plot(timestamps, estimated_distances, label='Estimated Distance', color='orange')
    ax1.set_title(f'Distance Comparison - {attack_type.capitalize()} Attack')
    ax1.set_xlabel('Timestamp')
    ax1.set_ylabel('Distance (m)')
    ax1.legend()
    ax1.grid(True)

    ax2.plot(timestamps, true_angles, label='True Angle', color='red')
    ax2.plot(timestamps, claimed_angles, label='Claimed Angle', color='green')
    ax2.plot(timestamps, estimated_angles, label='Estimated Angle', color='orange')
    ax2.set_title('Angle Comparison')
    ax2.set_xlabel('Timestamp')
    ax2.set_ylabel('Angle (rad)')
    ax2.legend()
    ax2.grid(True)

    detection_periods = [a if p else 0 for a, p in zip(attack_detected, attack_periods)]

    ax3.plot(timestamps, attack_periods, label='Attack Period', color='red', alpha=0.5)
    ax3.plot(timestamps, detection_periods, label='Attack Detected', color='blue', alpha=0.5)
    ax3.set_ylabel('Attack Detection')
    ax3.set_ylim(-0.1, 1.1)
    ax3.legend()

    # 计算检测率
    total_attack_periods = sum(attack_periods)
    correct_detections = sum(1 for a, p in zip(attack_detected, attack_periods) if a == 1 and p == 1)
    
    if total_attack_periods > 0:
        detection_rate = correct_detections / total_attack_periods
    else:
        detection_rate = 0

    ax3.text(0.02, 0.95, f'Detection Rate: {detection_rate:.2%}', 
             transform=ax3.transAxes, fontsize=12, verticalalignment='top')

    # 添加攻击开始和结束时间的标记
    attack_start_time = min(t for t, p in zip(timestamps, attack_periods) if p == 1)
    attack_end_time = max(t for t, p in zip(timestamps, attack_periods) if p == 1)
    for ax in [ax1, ax2, ax3]:
        ax.axvline(x=attack_start_time, color='r', linestyle='--', label='Attack Start')
        ax.axvline(x=attack_end_time, color='r', linestyle='--', label='Attack End')
        ax.legend()

    ax3.set_xlabel('Timestamp')

    plt.tight_layout()
    plt.savefig(f'{attack_type}_attack_results.png')
    plt.close()


def main():
     # Load the EKF model
    ekf = load_ekf_model('my_ekf_model.pkl')
    
    # Load data
    data_151 = load_json_file('processed_data_2255.json')
    data_147 = load_json_file('processed_data_2251.json')
    lidar_data = load_json_file('filtered_lidar_data_2255.json')
    rssi_data = load_json_file('improved_rssi_distance.json')
    
    attack_types = ['constant', 'random', 'jump', 'lane']
    
    for attack_type in attack_types:
        results = []
        last_change_time = 0
        current_offset = (0, 0)
        last_jump_time = 0
        current_jump = (0, 0)
        attack_start_time = None
        attack_end_time = None
        
        for timestamp in sorted(data_151.keys()):
            if timestamp not in data_147 or timestamp not in lidar_data or timestamp not in rssi_data:
                continue

            ego_data = data_151[timestamp]
            other_data = data_147[timestamp]

            current_time = float(timestamp)

            # 设置攻击开始和结束时间
            if attack_start_time is None:
                attack_start_time = current_time + 30  # 30秒后开始攻击
            if attack_end_time is None:
                attack_end_time = attack_start_time + 60  # 攻击持续60秒

            # 预测步骤
            ekf.predict()

            lidar_points = [point['point'] for point in lidar_data[timestamp]['vehicle_lidar_data']]
            lidar_measurement = process_lidar_data(lidar_points, ego_data['x'], ego_data['y'], ego_data['imu_compass'], timestamp)

            if lidar_measurement is None:
                print(f"Warning: Invalid LiDAR measurement at timestamp {timestamp}")
                continue
            
            rssi_distance = rssi_data[timestamp]['distance']
            true_rx, true_ry = global_to_local(other_data['x'], other_data['y'], ego_data['x'], ego_data['y'], ego_data['imu_compass'], other_data['imu_compass'])

            if current_time >= attack_start_time and current_time < attack_end_time:
                (fake_rx, fake_ry), last_change_time, current_offset, last_jump_time, current_jump = simulate_attack(
                    attack_type, true_rx, true_ry, current_time, lidar_measurement, last_change_time, current_offset, last_jump_time, current_jump, attack_start_time)
            else:
                fake_rx, fake_ry = true_rx, true_ry
            
            
            is_fake, distance_diff, angle_diff = detect_fake_position(ekf, fake_rx, fake_ry, lidar_measurement, rssi_distance, DETECTION_THRESHOLD, ANGLE_THRESHOLD)
            
            true_distance = np.sqrt(true_rx**2 + true_ry**2)
            claimed_distance = np.sqrt(fake_rx**2 + fake_ry**2)
            estimated_distance = np.sqrt(ekf.x[0]**2 + ekf.x[1]**2)

            true_angle = calculate_true_relative_angle(ego_data['imu_compass'], other_data['imu_compass'])
            claimed_angle = np.arctan2(fake_ry, fake_rx)
            estimated_angle = np.arctan2(ekf.x[1], ekf.x[0])
   
            # Determine if we're in an attack period
            attack_period = 1 if (current_time >= attack_start_time and current_time < attack_end_time) else 0

            results.append({
                'timestamp': timestamp,
                'true_distance': true_distance,
                'claimed_distance': claimed_distance,
                'estimated_distance': estimated_distance,
                'true_angle': true_angle,
                'claimed_angle': claimed_angle,
                'estimated_angle': estimated_angle,
                'attack_detected': 1 if is_fake else 0,
                'true_rx': true_rx,
                'true_ry': true_ry,
                'estimated_rx': ekf.x[0],
                'estimated_ry': ekf.x[1],
                'attack_period': attack_period
            })
        plot_results(results, attack_type)
        print(f"Attack {attack_type} executing...  Done")
        #print(f"Timestamp {timestamp}: {'Fake' if is_fake else 'Genuine'} position. Difference: {distance_diff:.2f} m")

if __name__ == "__main__":
    main()
