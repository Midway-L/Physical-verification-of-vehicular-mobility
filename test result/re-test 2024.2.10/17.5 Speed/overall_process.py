import json
import math
from collections import defaultdict
import argparse
from typing import Dict, List, Tuple
import numpy as np
from sklearn.cluster import DBSCAN
import os

def calculate_angle(x, y):
    return np.arctan2(y, x)

def process_lidar_data(lidar_points, timestamp):
    
    points = [np.array(point['point']) for point in lidar_points]
    
    if len(points) < 2:  # 需要至少两个点来计算协方差
        return None  # 或者返回一个表示无效数据的特殊值

    points_array = np.array(points)
    valid_mask = np.all(np.isfinite(points_array), axis=1)
    valid_points = points_array[valid_mask]

    if len(valid_points) < 2:
        return None

    clustering = DBSCAN(eps=1.5, min_samples=3).fit(np.array(valid_points))
    labels = clustering.labels_
    
    unique_labels, counts = np.unique(labels, return_counts=True)
    if len(unique_labels) == 1 and unique_labels[0] == -1:  # 所有点都被视为噪声
        return None
    
    valid_labels = unique_labels[unique_labels != -1]
    if len(valid_labels) == 0:
        return None
    
    largest_cluster_idx = np.argmax(counts[unique_labels != -1])
    largest_cluster = valid_labels[largest_cluster_idx]
    
    # 获取最大聚类的点
    cluster_mask = (labels == largest_cluster)
    cluster_points = valid_points[cluster_mask]
    
    if len(cluster_points) < 2:
        return None
    
    # 计算平均位置
    mean_point = np.mean(cluster_points, axis=0)
    rx, ry, _ = mean_point
    
    relative_distance = np.sqrt(rx**2 + ry**2) + 2.5
    relative_angle = calculate_angle(rx, ry)
    
    return {
        'timestamp': timestamp,
        'rx': rx,
        'ry': ry,
        'distance': relative_distance,
        'angle': relative_angle,
    }


def read_json_file(file_path: str) -> Tuple[Dict[float, Dict], int]:
    data = {}
    total_count = 0
    with open(file_path, 'r') as f:
        for line in f:
            total_count += 1
            entry = json.loads(line)
            data[entry['timestamp']] = entry
    return data, total_count

def process_data(vehicle_id: str, input_dir: str, output_dir: str):
    # Define file paths based on vehicle_id
    std_file = os.path.join(input_dir, f"std_vehicle_2255.json")
    vehicle_file = os.path.join(input_dir, f"vehicle_{vehicle_id}.json")
    lidar_file = os.path.join(input_dir, f"lidar_semantic_{vehicle_id}.json")
    gnss_file = os.path.join(input_dir, f"gnss_sensor_{vehicle_id}.json")
    imu_file = os.path.join(input_dir, f"imu_sensor_{vehicle_id}.json")

    print(f"\nProcessing data for vehicle {vehicle_id}")
    
    # Read standard timestamps
    std_data, std_count = read_json_file(std_file)
    std_timestamps = set(std_data.keys())
    print(f"Standard file: {std_file}")
    print(f"Total entries: {std_count}")
    print(f"Unique timestamps: {len(std_timestamps)}")

    # Read and filter vehicle data
    vehicle_data, total_count = read_json_file(vehicle_file)
    filtered_vehicle_data = {ts: vehicle_data[ts] for ts in std_timestamps if ts in vehicle_data}
    print(f"Vehicle file: {vehicle_file}")
    print(f"Total entries: {total_count}")
    print(f"Matching entries: {len(filtered_vehicle_data)}")
    print(f"Removed entries: {total_count - len(filtered_vehicle_data)}")

    # Read and process sensor data
    sensor_files = {'lidar': lidar_file, 'gnss': gnss_file, 'imu': imu_file}
    sensor_data = {}
    for sensor, file in sensor_files.items():
        data, total_count = read_json_file(file)
        if sensor == 'lidar':
            processed_data = {ts: process_lidar_data(data[ts]['vehicle_lidar_data'], timestamp=ts) for ts in std_timestamps if ts in data}
        elif sensor == 'gnss':
            processed_data = {ts: {'location_x': data[ts]['location x'], 'location_y': data[ts]['location y']} for ts in std_timestamps if ts in data}
        elif sensor == 'imu':
            processed_data = {ts: {'acc_x': data[ts]['accelerometer'][0], 'acc_y': data[ts]['accelerometer'][1],'compass': data[ts]['compass']} for ts in std_timestamps if ts in data}
        
        sensor_data[sensor] = processed_data
        print(f"Sensor file ({sensor}): {file}")
        print(f"Total entries: {total_count}")
        print(f"Matching entries: {len(processed_data)}")
        print(f"Removed entries: {total_count - len(processed_data)}")

    # Merge all data
    merged_data = []
    new_timestamp = 0.0
    for ts in sorted(std_timestamps):
        entry = {
            'original_timestamp': ts,
            'new_timestamp': round(new_timestamp, 1),
            'x': vehicle_data[ts]['x'],
            'y': vehicle_data[ts]['y']
        }
        
        for sensor, data in sensor_data.items():
            if ts in data:
                entry.update({f'{sensor}_{k}': v for k, v in data[ts].items()})
        
        merged_data.append(entry)
        new_timestamp += 0.1

    # Write merged data to output file
    output_file = os.path.join(output_dir, f"processed_data_{vehicle_id}.json")
    with open(output_file, 'w') as f:
        for data in merged_data:
            json.dump(data, f)
            f.write('\n')

    print(f"\nProcessed data saved to {output_file}")
    print(f"Total merged data points: {len(merged_data)}")

def main():
    parser = argparse.ArgumentParser(description="Process and merge vehicle and sensor data for a single vehicle.")
    parser.add_argument("vehicle_id", help="Vehicle ID (e.g., 197 or 201)")
    parser.add_argument("--input_dir", default=".", help="Input directory containing all data files")
    parser.add_argument("--output_dir", default=".", help="Output directory for processed data")

    args = parser.parse_args()

    # Create output directory if it doesn't exist
    os.makedirs(args.output_dir, exist_ok=True)

    process_data(args.vehicle_id, args.input_dir, args.output_dir)

if __name__ == "__main__":
    main()
