import json
from typing import List, Dict

# 文件名配置
INPUT_FILES = {
    'std': 'std_vehicle_2255.json',
    'speed_0': 'speed_0.json',
    'speed_1': 'speed_1.json',
    'distance': 'sumo_distance.json'
}
OUTPUT_FILE = 'sumo_ground.json'

def load_json_file(file_path: str) -> List[Dict]:
    with open(file_path, 'r') as f:
        return [json.loads(line) for line in f]

def filter_and_align_data(std_data: List[Dict], speed_0: List[Dict], speed_1: List[Dict], distance: List[Dict]) -> List[Dict]:
    std_timestamps = set(point['timestamp'] for point in std_data)

    speed_0_dict = {point['timestamp']: point['speed'] for point in speed_0 if point['timestamp'] in std_timestamps}
    speed_1_dict = {point['timestamp']: point['speed'] for point in speed_1 if point['timestamp'] in std_timestamps}
    distance_dict = {point['timestamp']: point['distance'] for point in distance if point['timestamp'] in std_timestamps}

    aligned_data = []
    new_timestamp = 0.0
    for point in std_data:
        ts = point['timestamp']     
        if ts in speed_0_dict and ts in speed_1_dict and ts in distance_dict:
            aligned_data.append({
                'original_timestamp': ts,
                'new_timestamp': round(new_timestamp, 1),  # 四舍五入到小数点后一位
                'leader_speed': speed_0_dict[ts],
                'following_speed': speed_1_dict[ts],
                'relative_speed': abs(speed_1_dict[ts] - speed_0_dict[ts]),
                'distance': distance_dict[ts]
            })
        new_timestamp += 0.1
    return aligned_data

def main():
    std_data = load_json_file(INPUT_FILES['std'])
    speed_0_data = load_json_file(INPUT_FILES['speed_0'])
    speed_1_data = load_json_file(INPUT_FILES['speed_1'])
    distance_data = load_json_file(INPUT_FILES['distance'])

    aligned_data = filter_and_align_data(std_data, speed_0_data, speed_1_data, distance_data)

    with open(OUTPUT_FILE, 'w') as f:
        for item in aligned_data:
            json.dump(item, f)
            f.write('\n')

    print(f"Processed data saved to {OUTPUT_FILE}")

if __name__ == "__main__":
    main()
