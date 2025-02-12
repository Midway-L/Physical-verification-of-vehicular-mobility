import json
from collections import defaultdict
from decimal import Decimal, ROUND_HALF_UP
from operator import itemgetter

def round_to_first_decimal(value):
    return float(Decimal(str(value)).quantize(Decimal('0.1'), rounding=ROUND_HALF_UP))

def parse_vec_file(file_path):
    data = defaultdict(lambda: defaultdict(list))
    vector_map = {}

    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('vector'):
                parts = line.split()
                vector_id = int(parts[1])
                node_id = int(parts[2].split('[')[1].split(']')[0])
                var_name = parts[3]
                vector_map[vector_id] = (node_id, var_name)
            elif line[0].isdigit():
                parts = line.split()
                vector_id = int(parts[0])
                timestamp = float(parts[2])
                value = float(parts[3])
                
                node_id, var_name = vector_map[vector_id]
                data[node_id][var_name].append((timestamp, value))

    return data

def organize_data(raw_data):
    organized_data = defaultdict(list)
    
    for node_id, node_data in raw_data.items():
        other_node_id = 1 if node_id == 0 else 0
        
        send_times = sorted(set(t for t, _ in node_data.get("messageSendTime", [])))
        
        for i, send_time in enumerate(send_times):
            entry = {
                "timestamp": send_time,
                "messageSendTime": send_time,
                "posx": next((v for t, v in node_data["posx"] if t == send_time), None),
                "posy": next((v for t, v in node_data["posy"] if t == send_time), None),
                "rssi": None,
                "messageReceiveTime": None
            }

            # 对于 rssi 和 messageReceiveTime，我们使用下一个时间点的数据（如果存在）
            if i + 1 < len(send_times):
                next_send_time = send_times[i + 1]
                rounded_next_send_time = round_to_first_decimal(next_send_time)
                
                entry["rssi"] = next((v for t, v in node_data.get("rssi", []) if round_to_first_decimal(t) == rounded_next_send_time), None)
                
                receive_time = next((v for t, v in raw_data[other_node_id].get("messageReceiveTime", []) if round_to_first_decimal(t) == rounded_next_send_time), None)
                if receive_time is not None:
                    entry["messageReceiveTime"] = {
                        "time": receive_time,
                        "receiverNodeId": other_node_id
                    }

            organized_data[node_id].append(entry)

    return organized_data


def save_to_json(data, output_prefix):
    for node_id, node_data in data.items():
        filename = f"{output_prefix}_node_{node_id}.json"
        # 按时间戳排序
        sorted_data = sorted(node_data, key=itemgetter('timestamp'))
        with open(filename, 'w') as f:
            for entry in sorted_data:
                json_string = json.dumps(entry, separators=(',', ':'))
                f.write(json_string + '\n')
        print(f"Saved data for node {node_id} to {filename}")

# 主程序
vec_file_path = "Default-#0.vec"  # 请确保这是正确的文件路径
raw_data = parse_vec_file(vec_file_path)
organized_data = organize_data(raw_data)
save_to_json(organized_data, "output")
