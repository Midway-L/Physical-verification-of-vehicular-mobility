import json
import math
import argparse
from collections import defaultdict

def calculate_angle(x, y):
    if x == 0 and y == 0:
        return 0
    angle_rad = math.atan2(y, x)
    angle_deg = math.degrees(angle_rad)
    adjusted_angle = 90 - angle_deg
    if adjusted_angle > 180:
        adjusted_angle -= 360
    elif adjusted_angle < -180:
        adjusted_angle += 360
    return adjusted_angle

def process_data(id, input_dir, output_dir):
    lidar_file = f'{input_dir}/lidar_semantic_{id}.json'
    gnss_file = f'{input_dir}/gnss_sensor_{id}.json'
    imu_file = f'{input_dir}/imu_sensor_{id}.json'
    output_file = f'{output_dir}/processed_data_{id}.json'

    all_data = defaultdict(dict)

    # Read LIDAR data
    with open(lidar_file, 'r') as f:
        for line in f:
            data = json.loads(line)
            timestamp = data['timestamp']
            lidar_points = data['vehicle_lidar_data']
            if not lidar_points:
                all_data[timestamp].update({
                    'distance': 0,
                    'angle': 0,
                    'obj_idx': None,
                    'num_points': 0,
                    'avg_x': 0,
                    'avg_y': 0
                })
            else:
                distances = []
                angles = []
                x_distances = []
                y_distances = []
                obj_idx_count = defaultdict(int)
                for point in lidar_points:
                    x, y, _ = point['point']
                    obj_idx = point['object_idx']
                    distance = math.sqrt(x**2 + y**2)
                    angle = calculate_angle(x, y)
                    distances.append(distance)
                    angles.append(angle)
                    x_distances.append(abs(x))
                    y_distances.append(abs(y))
                    obj_idx_count[obj_idx] += 1
                all_data[timestamp].update({
                    'distance': sum(distances) / len(distances) if distances else 0,
                    'angle': sum(angles) / len(angles) if angles else 0,
                    'obj_idx': max(obj_idx_count, key=obj_idx_count.get) if obj_idx_count else None,
                    'num_points': len(lidar_points),
                    'avg_x': sum(x_distances) / len(x_distances) if x_distances else 0,
                    'avg_y': sum(y_distances) / len(y_distances) if y_distances else 0
                })

    # Read GNSS data
    with open(gnss_file, 'r') as f:
        for line in f:
            data = json.loads(line)
            timestamp = data['timestamp']
            all_data[timestamp].update({
                'location_x': data['location x'],
                'location_y': data['location y']
            })

    # Read IMU data
    with open(imu_file, 'r') as f:
        for line in f:
            data = json.loads(line)
            timestamp = data['timestamp']
            all_data[timestamp].update({
                'acc_x': data['accelerometer'][0],
                'acc_y': data['accelerometer'][1]
            })

    # Sort data by timestamp and create new timestamps
    sorted_data = sorted(all_data.items())
    merged_data = []
    for i, (original_timestamp, data) in enumerate(sorted_data):
        new_timestamp = round(i * 0.1, 1)
        merged_data.append({
            'timestamp': new_timestamp,
            'distance': data.get('distance', 0),
            'angle': data.get('angle', 0),
            'obj_idx': data.get('obj_idx', None),
            'num_points': data.get('num_points', 0),
            'avg_x': data.get('avg_x', 0),
            'avg_y': data.get('avg_y', 0),
            'location_x': data.get('location_x', None),
            'location_y': data.get('location_y', None),
            'acc_x': data.get('acc_x', None),
            'acc_y': data.get('acc_y', None),
        })

    # Write merged data to output file
    with open(output_file, 'w') as f:
        for data in merged_data:
            json.dump(data, f)
            f.write('\n')

    print(f"Processed data for ID {id} and saved to {output_file}")
    print(f"Total data points: {len(merged_data)}")

def main():
    parser = argparse.ArgumentParser(description="Process sensor data for a specific vehicle ID.")
    parser.add_argument("id", help="The ID of the vehicle to process")
    parser.add_argument("--input_dir", default=".", help="Directory containing input files (default: current directory)")
    parser.add_argument("--output_dir", default=".", help="Directory for output files (default: current directory)")
    
    args = parser.parse_args()
    
    process_data(args.id, args.input_dir, args.output_dir)

if __name__ == "__main__":
    main()