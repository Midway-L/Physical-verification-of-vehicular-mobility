import json
import math

def read_json_file(file_path):
    with open(file_path, 'r') as file:
        data = [json.loads(line) for line in file]
    return data

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def process_car_data(file1, file2):
    car1_data = read_json_file(file1)
    car2_data = read_json_file(file2)
    
    assert len(car1_data) == len(car2_data), "两个文件的数据点数量不一致"
    
    result = []
    
    for c1, c2 in zip(car1_data, car2_data):
        distance = calculate_distance(c1['x'], c1['y'], c2['x'], c2['y'])
        result.append({
            'new_timestamp': c1['new_timestamp'],
            'distance': round(distance, 4)
        })
    
    return result

def save_json_file(data, output_file):
    with open(output_file, 'w') as file:
       for item in data:
            json.dump(item, file)
            file.write('\n')

file1 = 'processed_data_278.json'
file2 = 'processed_data_282.json'
output_file = 'carla_distance.json'

result = process_car_data(file1, file2)
save_json_file(result, output_file)

print(f"处理完成，结果已保存到 {output_file}")
