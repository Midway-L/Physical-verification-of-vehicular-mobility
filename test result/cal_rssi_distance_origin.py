import json
import math

def calculate_distance(rssi):
    tx_power = 13.01  # dBm
    f = 5.890e9  # Hz
    c = 3e8  # m/s
    d0 = 1  # m
    alpha = 2.0

    pl_d = tx_power - rssi
    pl_d0 = 20 * math.log10((4 * math.pi * d0 * f) / c)
    
    distance = d0 * 10**((pl_d - pl_d0) / (10 * alpha))
    return distance
    
def process_json(input_file, output_file):
    processed_data = []

    with open(input_file, 'r') as f:
        new_timestamp = 0
        for line in f:
            try:
                item = json.loads(line.strip())
                rssi = item['rssi']
                distance = round(calculate_distance(rssi), 2)
                send_time = item['messageSendTime']
                receive_time = item['messageReceiveTime']['time']
                message_delay = round(receive_time - send_time, 6)

                processed_item = {
                    'new_timestamp': round(new_timestamp, 1),
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

input_file = 'output_node_1.json'  # node id in veins, 1 here for lead vehicle
output_file = 'rssi_distance.json'  

process_json(input_file, output_file)
print(f"RSSI origin distance saved to {output_file}")
