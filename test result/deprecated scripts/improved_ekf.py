import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import chi2
import json

class ImprovedEKF:
    def __init__(self, dt, init_x, init_P):
        self.dt = dt
        self.x = init_x
        self.P = init_P
        self.Q = np.diag([0.1, 0.1, 0.01, 0.1, 0.1])
        self.R = np.diag([10, 10, 1, 0.1])
        self.innovation_sequence = []
        self.innovation_covariance = []

    def predict(self):
        F = np.array([
            [1, 0, 0, self.dt, 0],
            [0, 1, 0, 0, self.dt],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        H = self.get_H(self.x)
        z_pred = self.h(self.x)
        y = z - z_pred
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

        self.innovation_sequence.append(y)
        self.innovation_covariance.append(S)
        if len(self.innovation_sequence) > 20:
            self.innovation_sequence.pop(0)
            self.innovation_covariance.pop(0)

        return K

    def get_H(self, x):
        rx, ry, theta, vx, vy = x
        denominator = np.sqrt(rx**2 + ry**2) + 1e-10
        return np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [rx/denominator, ry/denominator, 0, 0, 0],
            [-ry/(denominator**2), rx/(denominator**2), 1, 0, 0]
        ])

    def h(self, x):
        rx, ry, theta, vx, vy = x
        return np.array([
            rx,
            ry,
            np.sqrt(rx**2 + ry**2),
            np.arctan2(ry, rx)
        ])

    def innovation_consistency_test(self):
        if len(self.innovation_sequence) < 20:
            return True
        
        chi_squared_test = 0
        for y, S in zip(self.innovation_sequence, self.innovation_covariance):
            chi_squared_test += y.T @ np.linalg.inv(S) @ y
        
        dof = 4 * len(self.innovation_sequence)
        threshold = chi2.ppf(0.95, dof)
        
        return chi_squared_test < threshold

    def likelihood(self, z):
        z_pred = self.h(self.x)
        y = z - z_pred
        S = self.get_H(self.x) @ self.P @ self.get_H(self.x).T + self.R
        return np.exp(-0.5 * y.T @ np.linalg.inv(S) @ y) / np.sqrt((2*np.pi)**4 * np.linalg.det(S))

class AdaptiveEKF(ImprovedEKF):
    def __init__(self, dt, init_x, init_P):
        super().__init__(dt, init_x, init_P)
        self.alpha = 0.3
        self.N = 20

    def adaptive_R(self):
        if len(self.innovation_sequence) < self.N:
            return
        
        C = np.zeros((4, 4))
        for i in range(self.N):
            y = self.innovation_sequence[-i-1]
            C += np.outer(y, y)
        C /= self.N
        
        H = self.get_H(self.x)
        R_hat = C - H @ self.P @ H.T
        R_hat = (R_hat + R_hat.T) / 2
        
        self.R = self.alpha * self.R + (1 - self.alpha) * R_hat
        self.R = np.clip(self.R, 1e-6, None)

    def update(self, z):
        K = super().update(z)
        self.adaptive_R()
        return K

class HuberEKF(ImprovedEKF):
    def __init__(self, dt, init_x, init_P, k=1.345):
        super().__init__(dt, init_x, init_P)
        self.k = k

    def update(self, z):
        H = self.get_H(self.x)
        z_pred = self.h(self.x)
        y = z - z_pred
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        d = np.sqrt(y.T @ np.linalg.inv(S) @ y)
        w = self.k / d if d > self.k else 1

        self.x = self.x + w * K @ y
        self.P = (np.eye(5) - w * K @ H) @ self.P

        self.innovation_sequence.append(y)
        self.innovation_covariance.append(S)
        if len(self.innovation_sequence) > 20:
            self.innovation_sequence.pop(0)
            self.innovation_covariance.pop(0)

        return K

class IMM:
    def __init__(self, models, init_probs):
        self.models = models
        self.probs = init_probs

    def predict(self):
        for model in self.models:
            model.predict()

    def update(self, z):
        likelihoods = np.array([model.likelihood(z) for model in self.models])
        c = np.sum(self.probs * likelihoods)
        self.probs = self.probs * likelihoods / c

        for i, model in enumerate(self.models):
            model.update(z)

    def estimate(self):
        x_combined = np.zeros_like(self.models[0].x)
        P_combined = np.zeros_like(self.models[0].P)

        for i, model in enumerate(self.models):
            x_combined += self.probs[i] * model.x
            P_combined += self.probs[i] * (model.P + np.outer(model.x - x_combined, model.x - x_combined))

        return x_combined, P_combined
    
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

def global_to_local(global_x, global_y, ego_x, ego_y, ego_compass):
    dx = global_x - ego_x
    dy = global_y - ego_y
    theta = -(ego_compass - np.pi/2)
    local_x = dx * np.cos(theta) + dy * np.sin(theta)
    local_y = -dx * np.sin(theta) + dy * np.cos(theta)
    local_x = abs(local_x)
    return local_x, local_y

def calculate_angle(x, y):
    return np.arctan2(y, x)

def process_measurement(data):
    ego_x = data['ego_x']
    ego_y = data['ego_y']
    ego_compass = data['ego_compass']
    other_x = data['other_x']
    other_y = data['other_y']
    other_compass = data['other_compass']

    rx, ry = global_to_local(other_x, other_y, ego_x, ego_y, ego_compass)
    distance = np.sqrt(rx**2 + ry**2)
    angle = calculate_angle(rx, ry)

    return np.array([rx, ry, distance, angle])

def get_true_state(data):
    ego_x = data['ego_x']
    ego_y = data['ego_y']
    ego_compass = data['ego_compass']
    other_x = data['other_x']
    other_y = data['other_y']
    other_compass = data['other_compass']

    rx, ry = global_to_local(other_x, other_y, ego_x, ego_y, ego_compass)
    true_angle = calculate_angle(rx, ry)

    return np.array([rx, ry, true_angle])


def angle_difference(angle1, angle2):
    return np.abs((angle1 - angle2 + np.pi) % (2 * np.pi) - np.pi)

def analyze_performance(results):
    timestamps = [r['timestamp'] for r in results]
    position_errors = [np.sqrt((r['true_rx'] - r['estimated_rx'])**2 + 
                               (r['true_ry'] - r['estimated_ry'])**2) for r in results]
    angle_errors = [angle_difference(r['true_angle'], r['estimated_angle']) for r in results]
    
    rmse_position = np.sqrt(np.mean(np.array(position_errors)**2))
    rmse_angle = np.sqrt(np.mean(np.array(angle_errors)**2))
    
    plt.figure(figsize=(15, 10))
    plt.subplot(2, 1, 1)
    plt.plot(timestamps, position_errors)
    plt.title(f'Position Error Over Time (RMSE: {rmse_position:.2f})')
    plt.ylabel('Error (m)')
    
    plt.subplot(2, 1, 2)
    plt.plot(timestamps, angle_errors)
    plt.title(f'Angle Error Over Time (RMSE: {rmse_angle:.2f})')
    plt.ylabel('Error (rad)')
    plt.xlabel('Timestamp')
    
    plt.tight_layout()
    plt.show()

    return rmse_position, rmse_angle

def main():

    data_151 = load_json_file('processed_data_151.json')
    data_147 = load_json_file('processed_data_147.json')
    lidar_data = load_json_file('filtered_lidar_data_151.json')
    rssi_data = load_json_file('improved_rssi_distance.json')


    dt = 0.1  # 时间步长
    init_x = np.array([0, 0, 0, 0, 0])  # 初始状态
    init_P = np.eye(5) * 100  # 初始协方差

    # 创建不同的滤波器
    ekf = ImprovedEKF(dt, init_x, init_P)
    adaptive_ekf = AdaptiveEKF(dt, init_x, init_P)
    huber_ekf = HuberEKF(dt, init_x, init_P)

    # 创建IMM
    models = [ekf, adaptive_ekf, huber_ekf]
    init_probs = np.array([1/3, 1/3, 1/3])
    imm = IMM(models, init_probs)

    results = []

    for timestamp, measurement in data_147.items():
        imm.predict()
        imm.update(measurement)
        x_est, P_est = imm.estimate()

        true_state = get_true_state(timestamp)  
        
        results.append({
            'timestamp': timestamp,
            'estimated_rx': x_est[0],
            'estimated_ry': x_est[1],
            'estimated_angle': x_est[2],
            'true_rx': true_state[0],
            'true_ry': true_state[1],
            'true_angle': true_state[2]
        })

    rmse_position, rmse_angle = analyze_performance(results)
    print(f"Position RMSE: {rmse_position:.2f} m")
    print(f"Angle RMSE: {rmse_angle:.2f} rad")

if __name__ == "__main__":
    main()