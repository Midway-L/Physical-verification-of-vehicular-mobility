import json
import multiprocessing

def process_json_file(input_file, output_file):
    # 读取 JSON 文件
    with open(input_file, 'r') as file:
        data = [json.loads(line) for line in file]

    # 重新编排时间戳
    new_timestamp = 0
    for item in data:
        # 只更新时间戳，保留其他所有字段
        item["timestamp"] = round(new_timestamp, 1)
        new_timestamp += 0.1

    # 将处理后的数据写入新文件
    with open(output_file, 'w') as file:
        for item in data:
            json.dump(item, file)
            file.write('\n')

    print(f"save to {output_file}")

if __name__ == '__main__':
    # 定义输入和输出文件
    file_pairs = [
        ('vehicle_147.json', 'vehicle_147_processed.json'),
        ('vehicle_151.json', 'vehicle_151_processed.json')
    ]

    # 创建进程池
    with multiprocessing.Pool(processes=2) as pool:
        # 并行执行处理任务
        pool.starmap(process_json_file, file_pairs)

print("done")