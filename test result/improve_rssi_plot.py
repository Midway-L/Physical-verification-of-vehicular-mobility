import json
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, r2_score


def load_data(file_path):
    with open(file_path, 'r') as f:
        return [json.loads(line) for line in f]

rssi_data = load_data('rssi_distance.json')
cars_data = load_data('cars_distance.json')


timestamps = [d['new_timestamp'] for d in rssi_data]
rssi_distances = [d['distance'] for d in rssi_data]
actual_distances = [d['distance'] for d in cars_data]


X = np.array(rssi_distances).reshape(-1, 1)
y = np.array(actual_distances)

def polynomial_fitting(X, y, degree=2):
    poly_features = PolynomialFeatures(degree=degree, include_bias=False)
    X_poly = poly_features.fit_transform(X)
    
    model = LinearRegression()
    model.fit(X_poly, y)
    
    y_pred = model.predict(X_poly)
    
    mse = mean_squared_error(y, y_pred)
    r2 = r2_score(y, y_pred)
    
    return model, poly_features, y_pred, mse, r2

# poly fitting
poly_model, poly_features, poly_pred, poly_mse, poly_r2 = polynomial_fitting(X, y)

print(f"Polynomial Fitting MSE: {poly_mse}")
print(f"Polynomial Fitting R2: {poly_r2}")

def environmental_factor(X, y):
    factor = np.mean(y / X.flatten())
    y_pred = X.flatten() * factor
    
    mse = mean_squared_error(y, y_pred)
    r2 = r2_score(y, y_pred)
    
    return factor, y_pred, mse, r2

# envrionment factor
env_factor, env_pred, env_mse, env_r2 = environmental_factor(X, y)

print(f"Environmental Factor: {env_factor}")
print(f"Environmental Factor MSE: {env_mse}")
print(f"Environmental Factor R2: {env_r2}")

plt.figure(figsize=(15, 10))

plt.scatter(X, y, color='blue', label='Actual Data')

# poly fitting result
X_plot = np.linspace(X.min(), X.max(), 100).reshape(-1, 1)
X_poly_plot = poly_features.transform(X_plot)
y_poly_plot = poly_model.predict(X_poly_plot)
plt.plot(X_plot, y_poly_plot, color='red', label='Polynomial Fitting')

# envrionment factor result
plt.plot(X, env_pred, color='green', label='Environmental Factor')

plt.xlabel('RSSI Distance (m)')
plt.ylabel('Actual Distance (m)')
plt.title('Distance Estimation: RSSI vs Actual')
plt.legend()
plt.grid(True)

plt.savefig('distance_estimation.png')
plt.close()

# error anlysis
plt.figure(figsize=(15, 10))

plt.scatter(timestamps, y - X.flatten(), color='blue', label='Original Error')
plt.scatter(timestamps, y - poly_pred, color='red', label='Polynomial Fitting Error')
plt.scatter(timestamps, y - env_pred, color='green', label='Environmental Factor Error')

plt.xlabel('Timestamp')
plt.ylabel('Error (m)')
plt.title('Error Analysis')
plt.legend()
plt.grid(True)

plt.savefig('error_analysis.png')
plt.close()

original_mse = mean_squared_error(y, X.flatten())
original_r2 = r2_score(y, X.flatten())

print("Original Data:")
print(f"MSE: {original_mse}")
print(f"R2: {original_r2}")
print("\nPolynomial Fitting:")
print(f"MSE: {poly_mse}")
print(f"R2: {poly_r2}")
print(f"Improvement: {(original_mse - poly_mse) / original_mse * 100:.2f}%")
print("\nEnvironmental Factor:")
print(f"Factor: {env_factor}")
print(f"MSE: {env_mse}")
print(f"R2: {env_r2}")
print(f"Improvement: {(original_mse - env_mse) / original_mse * 100:.2f}%")
