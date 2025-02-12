import json
import math
from collections import defaultdict

def calculate_angle(x, y):
    # Calculate the angle in radians
    angle_rad = math.atan2(y, x)
    
    # Convert to degrees
    angle_deg = math.degrees(angle_rad)
    
    # Adjust the angle to match our desired orientation
    adjusted_angle = 90 - angle_deg
    
    # Normalize the angle to be between -180 and 180
    if adjusted_angle > 180:
        adjusted_angle -= 360
    elif adjusted_angle < -180:
        adjusted_angle += 360
    
    return adjusted_angle

def process_lidar_data(input_file, output_file):
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line in infile:
            data = json.loads(line)
            timestamp = data['timestamp']
            lidar_data = data['vehicle_lidar_data']
            
            if lidar_data:
                distances = []
                angles = []
                obj_idx_count = defaultdict(int)
                
                for point_data in lidar_data:
                    x, y, _ = point_data['point']
                    obj_idx = point_data['object_idx']
                    
                    # Calculate distance and angle
                    distance = math.sqrt(x**2 + y**2)
                    angle = calculate_angle(x, y)
                    
                    distances.append(distance)
                    angles.append(angle)
                    obj_idx_count[obj_idx] += 1
                
                # Calculate averages
                avg_distance = sum(distances) / len(distances)
                avg_angle = sum(angles) / len(angles)
                
                # Get the most common obj_idx
                most_common_obj_idx = max(obj_idx_count, key=obj_idx_count.get)
                
                # Create new data structure
                new_data = {
                    'timestamp': timestamp,
                    'vehicle_distance': round(avg_distance, 2),
                    'angle': round(avg_angle, 2),
                    'obj_idx': most_common_obj_idx,
                    'num_points': len(lidar_data)
                }
                
                # Write to output file
                json.dump(new_data, outfile)
                outfile.write('\n')

# Process both files
input_files = ['lidar_semantic_147.json', 'lidar_semantic_151.json']
output_files = ['processed_lidar_data_147.json', 'processed_lidar_data_151.json']

for input_file, output_file in zip(input_files, output_files):
    process_lidar_data(input_file, output_file)
    print(f"Processed {input_file} and saved results to {output_file}")