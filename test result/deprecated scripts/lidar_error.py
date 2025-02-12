import json
import math
import os
import argparse
from typing import Dict, List, Tuple
from collections import defaultdict

def calculate_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_angle(x: float, y: float) -> float:
    return math.degrees(math.atan2(y, x))

def read_processed_data(file_path: str) -> Dict[float, Dict]:
    data = {}
    with open(file_path, 'r') as f:
        for line in f:
            entry = json.loads(line)
            data[entry['original_timestamp']] = entry
    return data

def read_lidar_data(file_path: str) -> Dict[float, List[Dict]]:
    data = {}
    with open(file_path, 'r') as f:
        for line in f:
            entry = json.loads(line)
            data[entry['timestamp']] = entry['vehicle_lidar_data']
    return data

def process_lidar_data(lidar_data: List[Dict]) -> Dict:
    if not lidar_data:
        return {
            'distance': 0,
            'angle': 0,
            'obj_idx': None,
            'num_points': 0,
            'avg_x': 0,
            'avg_y': 0
        }
    
    x_sum = 0
    y_sum = 0
    obj_idx_count = defaultdict(int)
    
    for point in lidar_data:
        x, y, _ = point['point']
        obj_idx = point['object_idx']
        x_sum += x
        y_sum += y
    
    num_points = len(lidar_data)
    avg_x = x_sum / num_points
    avg_y = y_sum / num_points
    distance = math.sqrt(avg_x**2 + avg_y**2)
    angle = calculate_angle(avg_x, avg_y)
    
    return {
        'distance': distance,
        'angle': angle,
        'obj_idx': obj_idx,
        'avg_x': avg_x,
        'avg_y': avg_y,
        'num_points': num_points
    }

def synchronize_data(processed_data: Dict[float, Dict], lidar_data: Dict[float, List[Dict]]) -> Dict[float, Tuple[Dict, List[Dict]]]:
    synchronized_data = {}
    processed_timestamps = set(processed_data.keys())
    lidar_timestamps = set(lidar_data.keys())
    
    common_timestamps = processed_timestamps.intersection(lidar_timestamps)
    sorted_timestamps = sorted(common_timestamps)
    
    for i, original_timestamp in enumerate(sorted_timestamps):
        new_timestamp = i * 0.1  # Generate new timestamps starting from 0 with 0.1 interval
        synchronized_data[new_timestamp] = (processed_data[original_timestamp], lidar_data[original_timestamp])
    
    return synchronized_data

def calculate_lidar_errors(synchronized_data: Dict[float, Tuple[Dict, List[Dict]]], vehicle2_data: Dict[float, Dict]) -> List[Dict]:
    errors = []
    v2_timestamps = list(vehicle2_data.keys())
    
    for new_ts, (v1_data, lidar_points) in synchronized_data.items():
        # Find the closest timestamp in vehicle2 data
        closest_v2_ts = min(v2_timestamps, key=lambda x: abs(x - v1_data['original_timestamp']))
        v2_data = vehicle2_data[closest_v2_ts]
        
        # Calculate true relative distance and angle
        true_dx = v2_data['x'] - v1_data['x']
        true_dy = v2_data['y'] - v1_data['y']
        true_distance = calculate_distance(v1_data['x'], v1_data['y'], v2_data['x'], v2_data['y'])
        true_angle = calculate_angle(true_dx, true_dy)
        
        # Process lidar data
        lidar_data = process_lidar_data(lidar_points)
        lidar_distance = lidar_data['distance']
        lidar_angle = lidar_data['angle']
        lidar_dx = lidar_data['avg_x']
        lidar_dy = lidar_data['avg_y']
        
        # Calculate errors
        error_distance = abs(lidar_distance - true_distance)
        error_angle = min(abs(lidar_angle - true_angle), 360 - abs(lidar_angle - true_angle))
        error_x = abs(lidar_dx - true_dx)
        error_y = abs(lidar_dy - true_dy)
        
        errors.append({
            'new_timestamp': new_ts,
            'original_timestamp': v1_data['original_timestamp'],
            'true_distance': true_distance,
            'true_angle': true_angle,
            'true_dx': true_dx,
            'true_dy': true_dy,
            'lidar_distance': lidar_distance,
            'lidar_angle': lidar_angle,
            'lidar_dx': lidar_dx,
            'lidar_dy': lidar_dy,
            'error_distance': error_distance,
            'error_angle': error_angle,
            'error_x': error_x,
            'error_y': error_y,
            'num_points': lidar_data['num_points']
        })
    
    return errors

def calculate_statistics(errors: List[Dict]) -> Dict:
    if not errors:
        return {}
    
    error_distance_values = [e['error_distance'] for e in errors]
    error_angle_values = [e['error_angle'] for e in errors]
    error_x_values = [e['error_x'] for e in errors]
    error_y_values = [e['error_y'] for e in errors]
    
    return {
        'mean_error_distance': sum(error_distance_values) / len(error_distance_values),
        'mean_error_angle': sum(error_angle_values) / len(error_angle_values),
        'mean_error_x': sum(error_x_values) / len(error_x_values),
        'mean_error_y': sum(error_y_values) / len(error_y_values),
        'max_error_distance': max(error_distance_values),
        'max_error_angle': max(error_angle_values),
        'max_error_x': max(error_x_values),
        'max_error_y': max(error_y_values),
        'min_error_distance': min(error_distance_values),
        'min_error_angle': min(error_angle_values),
        'min_error_x': min(error_x_values),
        'min_error_y': min(error_y_values)
    }

def process_lidar_errors(vehicle1_id: str, vehicle2_id: str, input_dir: str, output_dir: str):
    # Read processed data for both vehicles
    vehicle1_processed_file = os.path.join(input_dir, f"processed_data_{vehicle1_id}.json")
    vehicle2_processed_file = os.path.join(input_dir, f"processed_data_{vehicle2_id}.json")
    
    print(f"Reading processed data for vehicle {vehicle1_id}...")
    vehicle1_processed_data = read_processed_data(vehicle1_processed_file)
    print(f"Read {len(vehicle1_processed_data)} entries for vehicle {vehicle1_id}")
    
    print(f"Reading processed data for vehicle {vehicle2_id}...")
    vehicle2_processed_data = read_processed_data(vehicle2_processed_file)
    print(f"Read {len(vehicle2_processed_data)} entries for vehicle {vehicle2_id}")
    
    # Read lidar data for vehicle1
    vehicle1_lidar_file = os.path.join(input_dir, f"lidar_semantic_{vehicle1_id}.json")
    print(f"Reading lidar data for vehicle {vehicle1_id}...")
    vehicle1_lidar_data = read_lidar_data(vehicle1_lidar_file)
    print(f"Read {len(vehicle1_lidar_data)} lidar entries for vehicle {vehicle1_id}")
    
    # Synchronize processed data and lidar data for vehicle1
    print("Synchronizing data...")
    synchronized_data = synchronize_data(vehicle1_processed_data, vehicle1_lidar_data)
    print(f"Synchronized {len(synchronized_data)} data points")
    
    # Calculate errors
    print("Calculating errors...")
    errors = calculate_lidar_errors(synchronized_data, vehicle2_processed_data)
    print(f"Calculated {len(errors)} error entries")
    
    # Calculate statistics
    print("Calculating statistics...")
    statistics = calculate_statistics(errors)
    
    # Write errors and statistics to output file
    output_file = os.path.join(output_dir, f"lidar_error_{vehicle1_id}_{vehicle2_id}.json")
    print(f"Writing results to {output_file}...")
    with open(output_file, 'w') as f:
        # Write errors
        for error in errors:
            json.dump(error, f)
            f.write('\n')
        
        # Write statistics
        f.write('\n--- Statistics ---\n')
        json.dump(statistics, f, indent=2)
    
    print(f"Lidar error analysis saved to {output_file}")
    print("Error statistics:")
    for key, value in statistics.items():
        print(f"{key}: {value:.4f}")

def main():
    parser = argparse.ArgumentParser(description="Calculate lidar measurement errors between two vehicles.")
    parser.add_argument("vehicle1_id", help="ID of the first vehicle")
    parser.add_argument("vehicle2_id", help="ID of the second vehicle")
    parser.add_argument("--input_dir", default=".", help="Input directory containing processed data files")
    parser.add_argument("--output_dir", default=".", help="Output directory for error analysis")

    args = parser.parse_args()

    # Create output directory if it doesn't exist
    os.makedirs(args.output_dir, exist_ok=True)

    try:
        process_lidar_errors(args.vehicle1_id, args.vehicle2_id, args.input_dir, args.output_dir)
    except Exception as e:
        print(f"An error occurred: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()