import json
import numpy as np

# 读取 lidar_semantic_201.json 文件
lidar_data = []
with open('lidar_semantic_290.json', 'r') as f:
    for line in f:
        lidar_data.append(json.loads(line.strip()))

# 读取 processed_data_201.json 文件
processed_data = []
with open('processed_data_290.json', 'r') as f:
    for line in f:
        processed_data.append(json.loads(line.strip()))

# 获取 processed_data 中的时间戳
processed_timestamps = [frame['original_timestamp'] for frame in processed_data]

# 过滤 lidar_data
filtered_lidar_data = [frame for frame in lidar_data if frame['timestamp'] in processed_timestamps]

# 生成新的时间戳
new_timestamps = np.arange(0, len(filtered_lidar_data) * 0.1, 0.1)

# 创建新的数据结构
new_data = []
for i, (frame, new_timestamp) in enumerate(zip(filtered_lidar_data, new_timestamps)):
    new_frame = {
        'original_timestamp': frame['timestamp'],
        'new_timestamp': round(new_timestamp, 1),
        'vehicle_lidar_data': frame['vehicle_lidar_data']
    }
    new_data.append(new_frame)

# 保存新的 JSON 文件
with open('filtered_lidar_data_201.json', 'w') as f:
    for item in new_data:
        json.dump(item, f)
        f.write('\n')

# 输出相关信息
print(f"原始数据帧数: {len(lidar_data)}")
print(f"processed_data 帧数: {len(processed_data)}")
print(f"过滤后数据帧数: {len(filtered_lidar_data)}")
print(f"新数据帧数: {len(new_data)}")
