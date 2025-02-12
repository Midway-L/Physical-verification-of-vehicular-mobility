import json
import math

def estimate_distance(rssi, pt=13, pl_d0=47.86, alpha=2.0, d0=1):
    return d0 * 10**((pt - rssi - pl_d0) / (10 * alpha))

def process_json(input_file, output_file):
    processed_data = []

    with open(input_file, 'r') as f:
        new_timestamp = 0
        for line in f:
            try:
                item = json.loads(line.strip())
                rssi = item['rssi']
                distance = round(estimate_distance(rssi), 2)
                send_time = item['messageSendTime']
                receive_time = item['messageReceiveTime']['time']
                message_delay = round(receive_time - send_time, 6)

                processed_item = {
                    'timestamp': round(new_timestamp, 1),
                    'distance': distance,
                    'messageDelay': message_delay
                }
                processed_data.append(processed_item)
                new_timestamp += 0.1
            except json.JSONDecodeError as e:
                print(f"Error decoding JSON in line: {line}")
                print(f"Error message: {str(e)}")
                continue
            except KeyError as e:
                print(f"Missing key in JSON object: {e}")
                print(f"Problematic line: {line}")
                continue

    with open(output_file, 'w') as f:
        for item in processed_data:
            json.dump(item, f)
            f.write('\n')

input_file = 'output_node_1.json'  
output_file = 'rssi_distance.json'  

process_json(input_file, output_file)
print(f"处理完成。结果已保存到 {output_file}")
