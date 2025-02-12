import json
import math

def estimate_distance(rssi, pt=13, pl_d0=47.86, alpha=2.0, d0=1):
    return d0 * 10**((pt - rssi - pl_d0) / (10 * alpha))

def process_json(input_file, output_file):
    with open(input_file, 'r') as f:
        data = json.load(f)

    processed_data = []

    for item in data:
        timestamp = round(item['timestamp'], 1)
        rssi = item['rssi']
        distance = round(estimate_distance(rssi), 2)
        send_time = item['messageSendTime']
        receive_time = item['messageReceiveTime']['time']
        message_delay = round(receive_time - send_time, 6)

        processed_item = {
            'timestamp': timestamp,
            'distance': distance,
            'messageDelay': message_delay
        }
        processed_data.append(processed_item)

    with open(output_file, 'w') as f:
        json.dump(processed_data, f, indent=2)

# 使用脚本
input_file = 'output_node_1.json'  # 替换为您的输入文件名
output_file = 'rssi_distance'  # 替换为您想要的输出文件名

process_json(input_file, output_file)
print(f"处理完成。结果已保存到 {output_file}")