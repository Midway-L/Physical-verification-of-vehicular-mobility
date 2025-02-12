import json
import math

def read_json_file(file_path):
    with open(file_path, 'r') as file:
        data = [json.loads(line) for line in file]
    return data

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def round_timestamp(timestamp):
    return round(timestamp, 1)

def process_car_data(file1, file2):
    car1_data = read_json_file(file1)
    car2_data = read_json_file(file2)
    
    # 确保两个文件有相同的时间戳
    assert len(car1_data) == len(car2_data), "两个文件的数据点数量不一致"
    for c1, c2 in zip(car1_data, car2_data):
        assert round_timestamp(c1['timestamp']) == round_timestamp(c2['timestamp']), "时间戳不匹配"
    
    result = []
    for c1, c2 in zip(car1_data, car2_data):
        distance = calculate_distance(c1['x'], c1['y'], c2['x'], c2['y'])
        result.append({
            'timestamp': round_timestamp(c1['timestamp']),
            'car1': {'x': round(c1['x'], 4), 'y': round(c1['y'], 4)},
            'car2': {'x': round(c2['x'], 4), 'y': round(c2['y'], 4)},
            'distance': round(distance, 4)
        })
    
    return result

def save_json_file(data, output_file):
    with open(output_file, 'w') as file:
        json.dump(data, file, indent=2)

# 使用示例
file1 = 'vehicle_147.json'
file2 = 'vehicle_151.json'
output_file = 'cars_distance.json'

result = process_car_data(file1, file2)
save_json_file(result, output_file)

print(f"处理完成，结果已保存到 {output_file}")