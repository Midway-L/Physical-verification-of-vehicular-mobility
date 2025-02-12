import numpy as np
import matplotlib.pyplot as plt
import json
from scipy.linalg import sqrtm

def read_json_file(file_path):
    with open(file_path, 'r') as file:
        return [json.loads(line) for line in file]

sumo_ground = read_json_file('sumo_ground.json') # sumo distance
processed_data_follow = read_json_file('processed_data_follow_id.json') # follow vehicle
rssi_distance = read_json_file('rssi_distance.json')
processed_data_lead = read_json_file('processed_data_lead_id.json') # lead vehicle


# EKF class
class ExtendedKalmanFilter:
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise):
        self.state = initial_state
        self.covariance = initial_covariance
        self.Q = process_noise
        self.R = measurement_noise

    def predict(self, dt):
        x, y, v = self.state
        dist = np.sqrt(x**2 + y**2)
        epsilon = 1e-10  
        if dist > epsilon:
            dx = v * dt * x / dist
            dy = v * dt * y / dist
        else:
            dx = dy = 0
        
        F = np.array([
            [1, 0, dx/(v + epsilon)],
            [0, 1, dy/(v + epsilon)],
            [0, 0, 1]
        ])
        
        self.state = np.array([x + dx, y + dy, v])
        self.covariance = F @ self.covariance @ F.T + self.Q

    def update(self, measurement):
        H = self.jacobian(self.state)
        S = H @ self.covariance @ H.T + self.R
        K = self.covariance @ H.T @ np.linalg.inv(S)
        y = measurement - self.measurement_function(self.state)
        self.state = self.state + K @ y
        I = np.eye(3)
        self.covariance = (I - K @ H) @ self.covariance

    def measurement_function(self, state):
        x, y, _ = state
        return np.array([np.sqrt(x**2 + y**2), x, y])

    def jacobian(self, state):
        x, y, _ = state
        d = np.sqrt(x**2 + y**2)
        epsilon = 1e-10  
        return np.array([
            [x/(d + epsilon), y/(d + epsilon), 0],
            [1, 0, 0],
            [0, 1, 0]
        ])

    def get_state(self):
        return self.state

# Unscent KF
class UnscentedKalmanFilter:
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise, alpha, beta, kappa):
        self.state = initial_state
        self.covariance = initial_covariance
        self.Q = process_noise
        self.R = measurement_noise
        self.n = len(initial_state)
        self.m = len(measurement_noise)

        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.lambda_ = self.alpha**2 * (self.n + self.kappa) - self.n

        self.weights_m = np.zeros(2*self.n + 1)
        self.weights_c = np.zeros(2*self.n + 1)
        self.weights_m[0] = self.lambda_ / (self.n + self.lambda_)
        self.weights_c[0] = self.lambda_ / (self.n + self.lambda_) + (1 - self.alpha**2 + self.beta)
        for i in range(1, 2*self.n + 1):
            self.weights_m[i] = 1 / (2 * (self.n + self.lambda_))
            self.weights_c[i] = self.weights_m[i]

    def predict(self, dt):
        sigma_points = self.generate_sigma_points()
        transformed_sigma_points = np.zeros_like(sigma_points)
        for i in range(sigma_points.shape[1]):
            transformed_sigma_points[:, i] = self.state_transition_function(sigma_points[:, i], dt)

        self.state = np.sum(self.weights_m[np.newaxis, :] * transformed_sigma_points, axis=1)
        self.covariance = np.zeros_like(self.covariance)
        for i in range(transformed_sigma_points.shape[1]):
            diff = transformed_sigma_points[:, i] - self.state
            self.covariance += self.weights_c[i] * np.outer(diff, diff)
        self.covariance += self.Q

    def update(self, measurement):
        sigma_points = self.generate_sigma_points()
        transformed_sigma_points = np.zeros((self.m, sigma_points.shape[1]))
        for i in range(sigma_points.shape[1]):
            transformed_sigma_points[:, i] = self.measurement_function(sigma_points[:, i])

        
        predicted_measurement = np.sum(self.weights_m[np.newaxis, :] * transformed_sigma_points, axis=1)

        S = np.zeros((self.m, self.m))
        Pxz = np.zeros((self.n, self.m))
        for i in range(transformed_sigma_points.shape[1]):
            diff_z = transformed_sigma_points[:, i] - predicted_measurement
            diff_x = sigma_points[:, i] - self.state
            S += self.weights_c[i] * np.outer(diff_z, diff_z)
            Pxz += self.weights_c[i] * np.outer(diff_x, diff_z)
        S += self.R

        K = Pxz @ np.linalg.inv(S)
        self.state = self.state + K @ (measurement - predicted_measurement)
        self.covariance = self.covariance - K @ S @ K.T

    def generate_sigma_points(self):
        sqrt_term = sqrtm((self.n + self.lambda_) * self.covariance)
        sigma_points = np.zeros((self.n, 2*self.n + 1))
        sigma_points[:, 0] = self.state
        for i in range(self.n):
            sigma_points[:, i+1] = self.state + sqrt_term[:, i]
            sigma_points[:, i+1+self.n] = self.state - sqrt_term[:, i]
        return sigma_points

    def state_transition_function(self, state, dt):
        x, y, v = state
        dist = np.sqrt(x**2 + y**2)
        epsilon = 1e-10 
        if dist > epsilon:
            dx = v * dt * x / dist
            dy = v * dt * y / dist
        else:
            dx = dy = 0
        return np.array([x + dx, y + dy, v])

    def measurement_function(self, state):
        x, y, _ = state
        return np.array([np.sqrt(x**2 + y**2), x, y])

    def get_state(self):
        return self.state


def evaluate_and_plot(predictions, ground_truth, timestamps, title):
    errors = predictions - ground_truth
    rmse_x = np.sqrt(np.mean(errors[:, 0]**2))
    rmse_y = np.sqrt(np.mean(errors[:, 1]**2))
    rmse_distance = np.sqrt(np.mean(np.sum(errors[:, :2]**2, axis=1)))

    print(f"{title} RMSE:")
    print(f"X: {rmse_x:.2f} m")
    print(f"Y: {rmse_y:.2f} m")
    print(f"Distance: {rmse_distance:.2f} m")

    plt.figure(figsize=(15, 5))

    plt.subplot(131)
    plt.plot(timestamps, predictions[:, 0], label='Predicted')
    plt.plot(timestamps, ground_truth[:, 0], label='True')
    plt.xlabel('Time (s)')
    plt.ylabel('X (m)')
    plt.legend()
    plt.title('X Position')

    plt.subplot(132)
    plt.plot(timestamps, predictions[:, 1], label='Predicted')
    plt.plot(timestamps, ground_truth[:, 1], label='True')
    plt.xlabel('Time (s)')
    plt.ylabel('Y (m)')
    plt.legend()
    plt.title('Y Position')

    plt.subplot(133)
    pred_distance = np.sqrt(np.sum(predictions[:, :2]**2, axis=1))
    true_distance = np.sqrt(np.sum(ground_truth[:, :2]**2, axis=1))
    plt.plot(timestamps, pred_distance, label='Predicted')
    plt.plot(timestamps, true_distance, label='True')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.legend()
    plt.title('Distance')

    plt.suptitle(title)
    plt.tight_layout()
    plt.show()


def visualize_weights(filter_name, ekf=None, ukf=None):
    plt.figure(figsize=(10, 6))
    
    if filter_name == 'EKF':
        if ekf is None:
            raise ValueError("EKF object is required for EKF weight visualization")
        weights = ekf.covariance
        if weights.ndim == 2:
            weights = np.diag(weights)
        elif weights.ndim > 2:
            weights = weights.flatten()
        
        plt.bar(range(len(weights)), weights)
        plt.title('EKF Covariance Diagonal Elements or Flattened Weights')
        plt.xlabel('Index')
        plt.ylabel('Value')
        
    elif filter_name == 'UKF':
        if ukf is None:
            raise ValueError("UKF object is required for UKF weight visualization")
        weights_m = ukf.weights_m
        weights_c = ukf.weights_c
        
        plt.subplot(2, 1, 1)
        plt.bar(range(len(weights_m)), weights_m)
        plt.title('UKF Mean Weights')
        plt.xlabel('Sigma Point')
        plt.ylabel('Weight')
        
        plt.subplot(2, 1, 2)
        plt.bar(range(len(weights_c)), weights_c)
        plt.title('UKF Covariance Weights')
        plt.xlabel('Sigma Point')
        plt.ylabel('Weight')
    else:
        raise ValueError("Invalid filter name. Choose 'EKF' or 'UKF'")

    plt.tight_layout()
    plt.show()


def main():

    initial_x = processed_data_lead[0]['x'] - processed_data_follow[0]['x']
    initial_y = processed_data_lead[0]['y'] - processed_data_follow[0]['y']
    initial_v = sumo_ground[0]['relative_speed']
    initial_state = np.array([initial_x, initial_y, initial_v])
    
    initial_covariance = np.diag([0.1**2,0.1*2,0.05**2])
    process_noise = np.eye(3) * 0.01
    measurement_noise = np.diag([4**2, 2**2, 0.5**2])


    ekf = ExtendedKalmanFilter(initial_state, initial_covariance, process_noise, measurement_noise)
    ukf = UnscentedKalmanFilter(initial_state, initial_covariance, process_noise, measurement_noise, alpha=0.1, beta=2, kappa=0)


    def run_filter(filter_obj, is_ekf=True):
        predictions = []
        ground_truth = []
        timestamps = []
        last_timestamp = 0

        for i in range(len(sumo_ground)):
            current_timestamp = sumo_ground[i]['new_timestamp']
            dt = current_timestamp - last_timestamp
            last_timestamp = current_timestamp

            filter_obj.predict(dt)

            measurement = np.array([
                rssi_distance[i]['distance'],
                processed_data_follow[i]['lidar_avg_x'],
                processed_data_follow[i]['lidar_avg_y']
            ])

            filter_obj.update(measurement)

            state = filter_obj.get_state()
            predictions.append(state[:2])
            
            true_x = processed_data_lead[i]['x'] - processed_data_lead[i]['x']
            true_y = processed_data_lead[i]['y'] - processed_data_lead[i]['y']
            ground_truth.append([true_x, true_y])
            
            timestamps.append(current_timestamp)

        return np.array(predictions), np.array(ground_truth), np.array(timestamps)
    
    ekf_predictions, ground_truth, timestamps = run_filter(ekf)
    ukf_predictions, ground_truth, timestamps = run_filter(ukf, is_ekf=False)

    title_ekf = 'ekf result'
    title_ukf = 'ukf_result'
    evaluate_and_plot(ekf_predictions,ground_truth, timestamps,title_ekf)
    evaluate_and_plot(ukf_predictions,ground_truth, timestamps,title_ukf)

    visualize_weights('EKF', ekf=ekf)
    visualize_weights('UKF', ukf=ukf)

if __name__ == "__main__":
    main()