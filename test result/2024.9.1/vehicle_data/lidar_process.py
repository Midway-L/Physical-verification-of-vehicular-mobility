import json
import math
from collections import defaultdict

def calculate_angle(x, y):
    if x == 0 and y == 0:
        return 0  # 当x和y都为0时，返回0度
    angle_rad = math.atan2(y, x)
    angle_deg = math.degrees(angle_rad)
    adjusted_angle = 90 - angle_deg
    if adjusted_angle > 180:
        adjusted_angle -= 360
    elif adjusted_angle < -180:
        adjusted_angle += 360
    return adjusted_angle

def process_lidar_data(input_file, output_file):
    new_timestamp = 0.0
    input_count = 0
    output_count = 0
    empty_data_count = 0
    
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line in infile:
            input_count += 1
            try:
                data = json.loads(line)
                lidar_data = data['vehicle_lidar_data']
                
                if not lidar_data:
                    empty_data_count += 1
                    new_data = {
                        'timestamp': round(new_timestamp, 1),
                        'distance': 0,
                        'angle': 0,
                        'obj_idx': None,
                        'num_points': 0
                    }
                else:
                    distances = []
                    angles = []
                    obj_idx_count = defaultdict(int)
                    
                    for point_data in lidar_data:
                        x, y, _ = point_data['point']
                        obj_idx = point_data['object_idx']
                        
                        distance = math.sqrt(x**2 + y**2)
                        angle = calculate_angle(x, y)
                        
                        distances.append(distance)
                        angles.append(angle)
                        obj_idx_count[obj_idx] += 1
                    
                    avg_distance = sum(distances) / len(distances)
                    avg_angle = sum(angles) / len(angles)
                    most_common_obj_idx = max(obj_idx_count, key=obj_idx_count.get)
                    
                    new_data = {
                        'timestamp': round(new_timestamp, 1),
                        'distance': round(avg_distance, 2),
                        'angle': round(avg_angle, 2),
                        'obj_idx': most_common_obj_idx,
                        'num_points': len(lidar_data)
                    }
                
                json.dump(new_data, outfile)
                outfile.write('\n')
                new_timestamp += 0.1
                output_count += 1
                
            except Exception as e:
                print(f"Error processing line {input_count}: {e}")
                continue

    print(f"Input data points: {input_count}")
    print(f"Output data points: {output_count}")
    print(f"Empty data points: {empty_data_count}")

input_files = ['lidar_semantic_147.json', 'lidar_semantic_151.json']
output_files = ['processed_lidar_data_147.json', 'processed_lidar_data_151.json']

for input_file, output_file in zip(input_files, output_files):
    process_lidar_data(input_file, output_file)
    print(f"Processed {input_file} and saved results to {output_file}")
