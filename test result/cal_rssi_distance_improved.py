import json
import math
import numpy as np
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, r2_score
import pickle
import os

def calculate_distance(rssi):
    tx_power = 13.01  # dBm
    f = 5.890e9  # Hz
    c = 3e8  # m/s
    d0 = 1  # m
    alpha = 2.0

    pl_d = tx_power - rssi
    pl_d0 = 20 * math.log10((4 * math.pi * d0 * f) / c)
    
    distance = d0 * 10**((pl_d - pl_d0) / (10 * alpha))
    return distance

def load_data(file_path):
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            try:
                item = json.loads(line.strip())
                data.append(item)
            except json.JSONDecodeError as e:
                print(f"Error decoding JSON in line: {line}")
                print(f"Error message: {str(e)}")
    return data

def process_rssi_data(input_file):
    processed_data = []
    with open(input_file, 'r') as f:
        new_timestamp = 0
        for line in f:
            try:
                item = json.loads(line.strip())
                rssi = item['rssi']
                distance = round(calculate_distance(rssi), 2)
                send_time = item['messageSendTime']
                receive_time = item['messageReceiveTime']['time']
                message_delay = round(receive_time - send_time, 6)

                processed_item = {
                    'new_timestamp': round(new_timestamp, 1),
                    'distance': distance,
                    'messageDelay': message_delay
                }
                processed_data.append(processed_item)
                new_timestamp += 0.1
            except (json.JSONDecodeError, KeyError) as e:
                print(f"Error processing line: {line}")
                print(f"Error message: {str(e)}")
    return processed_data

def polynomial_fitting(X, y, degree=2):
    poly_features = PolynomialFeatures(degree=degree, include_bias=False)
    X_poly = poly_features.fit_transform(X.reshape(-1, 1))
    
    model = LinearRegression()
    model.fit(X_poly, y)
    
    return model, poly_features

def save_model(model, poly_features, file_name='fitting_model.pkl'):
    with open(file_name, 'wb') as f:
        pickle.dump((model, poly_features), f)

def load_model(file_name='fitting_model.pkl'):
    with open(file_name, 'rb') as f:
        return pickle.load(f)


def main():
    rssi_input_file = 'output_node_1.json' # node id in veins, 1 for follow vehicle here
    cars_input_file = 'cars_distance.json' # carla distance here
    output_file = 'improved_rssi_distance.json'
    model_file = 'fitting_model.pkl'

    # get rssi data and calculate distance
    rssi_data = process_rssi_data(rssi_input_file)
    
    # if no model, then generate a new one
    if not os.path.exists(model_file):
        # actual distance data
        cars_data = load_data(cars_input_file)

        # align timestamps
        aligned_data = []
        for rssi_item in rssi_data:
            for cars_item in cars_data:
                if abs(rssi_item['new_timestamp'] - cars_item['new_timestamp']) < 0.05:
                    aligned_data.append({
                        'timestamp': rssi_item['new_timestamp'],
                        'rssi_distance': rssi_item['distance'],
                        'actual_distance': cars_item['distance'],
                        'messageDelay': rssi_item['messageDelay']
                    })
                    break

        X = np.array([item['rssi_distance'] for item in aligned_data])
        y = np.array([item['actual_distance'] for item in aligned_data])

        # polynomial fitting
        model, poly_features = polynomial_fitting(X, y)

        save_model(model, poly_features, model_file)
        
        # error statistic
        original_mse = mean_squared_error(y, X)
        improved_y = model.predict(poly_features.transform(X.reshape(-1, 1)))
        improved_mse = mean_squared_error(y, improved_y)
        improvement = (original_mse - improved_mse) / original_mse * 100

        print(f"origin MSE: {original_mse}")
        print(f"Improved MSE: {improved_mse}")
        print(f"Imporoved rate: {improvement:.2f}%")
    else:
        # if model exists, load it
        model, poly_features = load_model(model_file)
        print("Fitting model loaded")

    # apply the poly fitting calculation
    improved_data = []
    for item in rssi_data:
        X_pred = poly_features.transform([[item['distance']]])
        improved_distance = model.predict(X_pred)[0]
        improved_item = {
            'new_timestamp': item['new_timestamp'],
            'distance': round(improved_distance, 4),
            'messageDelay': item['messageDelay']
        }
        improved_data.append(improved_item)

    # save improve distance data
    with open(output_file, 'w') as f:
        for item in improved_data:
            json.dump(item, f)
            f.write('\n')

    print(f"Improved RSSI distance saved to {output_file}")

if __name__ == "__main__":
    main()
