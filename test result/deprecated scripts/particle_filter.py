import numpy as np
import json
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from scipy.stats import circstd

class ParticleFilter:
    def __init__(self, num_particles, init_x, init_P):
        self.num_particles = num_particles
        self.particles = np.random.multivariate_normal(init_x, init_P, num_particles)
        self.weights = np.ones(num_particles) / num_particles
        
        self.Q = np.diag([0.1, 0.1, 0.01])  # [rx, ry, theta]
        
    def predict(self):

        self.particles += np.random.multivariate_normal([0, 0, 0], self.Q, self.num_particles)
        self.particles[:, 2] = normalize_angle(self.particles[:, 2])

    def update(self, z, R):
        for i in range(self.num_particles):
            particle = self.particles[i]
            z_pred = self.measurement_model(particle)
            self.weights[i] *= self.gaussian_probability(z, z_pred, R)
        
        self.weights += 1.e-300  
        self.weights /= sum(self.weights)  
        

        if self.neff() < self.num_particles / 2:
            self.resample()

    def measurement_model(self, particle):
        rx, ry, _ = particle
        return np.array([
            rx,
            ry,
            np.sqrt(rx**2 + ry**2),
            np.arctan2(ry, rx)
        ])

    def gaussian_probability(self, x, mean, cov):
        diff = x - mean
        return np.exp(-0.5 * diff.T @ np.linalg.inv(cov) @ diff) / np.sqrt((2 * np.pi)**len(x) * np.linalg.det(cov))

    def neff(self):
        return 1. / np.sum(np.square(self.weights))

    def resample(self):
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.  
        indexes = np.searchsorted(cumulative_sum, np.random.random(self.num_particles))
        
        self.particles = self.particles[indexes]
        self.weights = np.ones(self.num_particles) / self.num_particles

    def estimate(self):
        mean = np.average(self.particles, weights=self.weights, axis=0)
        mean[2] = normalize_angle(mean[2])
        return mean

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def global_to_local(global_x, global_y, ego_x, ego_y, ego_compass):
    dx = global_x - ego_x
    dy = global_y - ego_y
    
    theta = np.pi/2 - ego_compass
    
    local_x = dx * np.cos(theta) + dy * np.sin(theta)
    local_y = -dx * np.sin(theta) + dy * np.cos(theta)
    
    return local_x, local_y

def process_lidar_data(lidar_points, ego_x, ego_y, ego_heading, timestamp):
    
    valid_points = [point for point in lidar_points if np.all(np.isfinite(point))]
    
    if len(valid_points) < 2:  
        return None  

    clustering = DBSCAN(eps=0.5, min_samples=5).fit(lidar_points)
    labels = clustering.labels_
    
    unique_labels, counts = np.unique(labels, return_counts=True)
    if len(unique_labels) == 1 and unique_labels[0] == -1:  
        return None
    largest_cluster = unique_labels[np.argmax(counts[unique_labels != -1])]
    
    cluster_points = np.array(lidar_points)[labels == largest_cluster]
    rx, ry = np.mean(cluster_points, axis=0)
    if len(cluster_points) < 2:
        return None
    
    relative_distance = np.sqrt(rx**2 + ry**2)
    relative_angle = np.arctan2(ry, rx)
    relative_angle = normalize_angle(relative_angle)

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

        point_spread = 1.0

    distances = np.sqrt(np.sum(cluster_points**2, axis=1))
    distance_std = np.std(distances)
    distance_error_ratio = distance_std / relative_distance if relative_distance != 0 else 1.0

    angles = np.arctan2(cluster_points[:, 1], cluster_points[:, 0])
    angle_std = circstd(angles)  

    cluster_density = len(cluster_points) / point_spread if point_spread != 0 else 1.0

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
                timestamp = data['new_timestamp']
                data_dict[timestamp] = data
            except json.JSONDecodeError as e:
                print(f"Error decoding JSON in {file_path}: {e}")
            except KeyError as e:
                print(f"Missing key in JSON object: {e}")
    return data_dict


def plot_results(results):
    timestamps = [r['timestamp'] for r in results]
    true_distances = [r['true_distance'] for r in results]
    estimated_distances = [r['estimated_distance'] for r in results]
    true_angles = [r['true_angle'] for r in results]
    estimated_angles = [r['estimated_angle'] for r in results]
    rx_error = [r['estimated_rx'] - r['true_rx'] for r in results]
    ry_error = [r['estimated_ry'] - r['true_ry'] for r in results]
    distance_error = [r['estimated_distance'] - r['true_distance'] for r in results]
    angle_error = [normalize_angle(r['estimated_angle'] - r['true_angle']) for r in results]

    plt.figure(figsize=(20, 25))
    
    # 1. 距离比较图
    plt.subplot(5, 1, 1)
    plt.plot(timestamps, true_distances, label='True Distance')
    plt.plot(timestamps, estimated_distances, label='Estimated Distance')
    plt.xlabel('Timestamp')
    plt.ylabel('Distance (m)')
    plt.title('True vs Estimated Distance')
    plt.legend()

    # 2. 角度比较图
    plt.subplot(5, 1, 2)
    plt.plot(timestamps, true_angles, label='True Angle')
    plt.plot(timestamps, estimated_angles, label='Estimated Angle')
    plt.xlabel('Timestamp')
    plt.ylabel('Angle (rad)')
    plt.title('True vs Estimated Angle')
    plt.legend()

    # 3. 距离误差图
    plt.subplot(5, 1, 3)
    plt.plot(timestamps, distance_error)
    plt.xlabel('Timestamp')
    plt.ylabel('Error (m)')
    plt.title('Distance Error')

    # 4. 角度误差图
    plt.subplot(5, 1, 4)
    plt.plot(timestamps, angle_error)
    plt.xlabel('Timestamp')
    plt.ylabel('Error (rad)')
    plt.title('Angle Error')

    # 5. 2D误差散点图
    plt.subplot(5, 1, 5)
    plt.scatter(rx_error, ry_error)
    plt.xlabel('RX Error (m)')
    plt.ylabel('RY Error (m)')
    plt.title('2D Position Error')
    plt.axis('equal')

    plt.tight_layout()
    plt.savefig('particle_filter_results.png')
    plt.close()

    # 计算和打印统计信息
    print("统计信息:")
    print(f"距离RMSE: {np.sqrt(np.mean(np.array(distance_error)**2)):.2f} m")
    print(f"角度RMSE: {np.sqrt(np.mean(np.array(angle_error)**2)):.2f} rad")
    print(f"RX RMSE: {np.sqrt(np.mean(np.array(rx_error)**2)):.2f} m")
    print(f"RY RMSE: {np.sqrt(np.mean(np.array(ry_error)**2)):.2f} m")



def main():

    data_151 = load_json_file('processed_data_151.json')
    data_147 = load_json_file('processed_data_147.json')
    lidar_data = load_json_file('filtered_lidar_data_151.json')
    rssi_data = load_json_file('improved_rssi_distance.json')

    # 初始化粒子滤波器
    num_particles = 1000
    init_x = np.array([0, 0, 0]) 
    init_P = np.diag([100, 100, np.pi/2])  
    pf = ParticleFilter(num_particles, init_x, init_P)

    results = []

    for timestamp in sorted(data_151.keys()):
        if timestamp not in data_147 or timestamp not in lidar_data or timestamp not in rssi_data:
            continue

        ego_data = data_151[timestamp]
        other_data = data_147[timestamp]

        pf.predict()

        lidar_points = [point['point'][:2] for point in lidar_data[timestamp]['vehicle_lidar_data']]
        lidar_measurement = process_lidar_data(lidar_points, ego_data['x'], ego_data['y'], ego_data['imu_compass'], timestamp)

        if lidar_measurement is None:
            print(f"Warning: Invalid LiDAR measurement at timestamp {timestamp}")
            continue

        R = np.diag([
            max(0.1, lidar_measurement['error_analysis']['point_spread']),
            max(0.1, lidar_measurement['error_analysis']['point_spread']),
            max(0.1, lidar_measurement['distance'] * lidar_measurement['error_analysis']['distance_error_ratio']),
            max(0.1, lidar_measurement['error_analysis']['angle_std'])
        ])

        rssi_distance = rssi_data[timestamp]['distance']

        z = np.array([lidar_measurement['rx'], lidar_measurement['ry'], rssi_distance, lidar_measurement['angle']])

        if np.all(np.isfinite(z)):
            pf.update(z, R)
        else:
            print(f"警告: 时间戳 {timestamp} 的测量值无效，跳过此次更新")

        # 获取估计值
        estimated_state = pf.estimate()

        # 计算真实的相对位置（局部坐标系）
        true_rx, true_ry = global_to_local(other_data['x'], other_data['y'], ego_data['x'], ego_data['y'], ego_data['imu_compass'])
        true_angle = np.arctan2(true_ry, true_rx)
        true_angle = normalize_angle(true_angle)
        
        estimated_angle = np.arctan2(estimated_state[1], estimated_state[0])
        estimated_angle = normalize_angle(estimated_angle)

        results.append({
            'timestamp': timestamp,
            'true_rx': true_rx,
            'true_ry': true_ry,
            'estimated_rx': estimated_state[0],
            'estimated_ry': estimated_state[1],
            'true_distance': np.sqrt(true_rx**2 + true_ry**2),
            'estimated_distance': np.sqrt(estimated_state[0]**2 + estimated_state[1]**2),
            'true_angle': true_angle,
            'estimated_angle': estimated_angle
        })
    
    plot_results(results)

if __name__ == "__main__":
    main()