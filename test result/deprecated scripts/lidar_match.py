import json
import numpy as np

# get origin lidar data
lidar_data = []
with open('lidar_semantic_vehicle_id.json', 'r') as f:
    for line in f:
        lidar_data.append(json.loads(line.strip()))

# get processed vehicle data
processed_data = []
with open('processed_data_vehicle_id.json', 'r') as f:
    for line in f:
        processed_data.append(json.loads(line.strip()))

processed_timestamps = [frame['original_timestamp'] for frame in processed_data]

filtered_lidar_data = [frame for frame in lidar_data if frame['timestamp'] in processed_timestamps]

new_timestamps = np.arange(0, len(filtered_lidar_data) * 0.1, 0.1)

new_data = []
for i, (frame, new_timestamp) in enumerate(zip(filtered_lidar_data, new_timestamps)):
    new_frame = {
        'original_timestamp': frame['timestamp'],
        'new_timestamp': round(new_timestamp, 1),
        'vehicle_lidar_data': frame['vehicle_lidar_data']
    }
    new_data.append(new_frame)

with open('filtered_lidar_data_201.json', 'w') as f:
    for item in new_data:
        json.dump(item, f)
        f.write('\n')

print(f"Origin frame: {len(lidar_data)}")
print(f"processed_data frame: {len(processed_data)}")
print(f"Filtered frame: {len(filtered_lidar_data)}")
print(f"New frame: {len(new_data)}")
