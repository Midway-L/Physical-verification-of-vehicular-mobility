import json
import math
from collections import defaultdict
import argparse
from typing import Dict, List, Tuple
import os


def read_json_file(file_path: str) -> Tuple[Dict[float, Dict], int]:
    data = {}
    total_count = 0
    with open(file_path, 'r') as f:
        for line in f:
            total_count += 1
            entry = json.loads(line)
            data[entry['timestamp']] = entry
    return data, total_count

def process_data(vehicle_id: str, input_dir: str, output_dir: str):
    # Define file paths based on vehicle_id
    std_file = os.path.join(input_dir, f"std_vehicle_446.json")
    vehicle_file = os.path.join(input_dir, f"vehicle_{vehicle_id}.json")

    # Read standard timestamps
    std_data, std_count = read_json_file(std_file)
    std_timestamps = set(std_data.keys())
    print(f"Standard file: {std_file}")
    print(f"Total entries: {std_count}")
    print(f"Unique timestamps: {len(std_timestamps)}")

    # Read and filter vehicle data
    vehicle_data, total_count = read_json_file(vehicle_file)
    filtered_vehicle_data = {ts: vehicle_data[ts] for ts in std_timestamps if ts in vehicle_data}
    print(f"Vehicle file: {vehicle_file}")
    print(f"Total entries: {total_count}")
    print(f"Matching entries: {len(filtered_vehicle_data)}")
    print(f"Removed entries: {total_count - len(filtered_vehicle_data)}")


    # Merge all data
    merged_data = []
    new_timestamp = 0.0
    for ts in sorted(std_timestamps):
        entry = {
            'original_timestamp': ts,
            'new_timestamp': round(new_timestamp, 1),
            'x': vehicle_data[ts]['x'],
            'y': vehicle_data[ts]['y']
        }
        
        merged_data.append(entry)
        new_timestamp += 0.1

    # Write merged data to output file
    output_file = os.path.join(output_dir, f"processed_data_{vehicle_id}.json")
    with open(output_file, 'w') as f:
        for data in merged_data:
            json.dump(data, f)
            f.write('\n')

    print(f"\nProcessed data saved to {output_file}")
    print(f"Total merged data points: {len(merged_data)}")

def main():
    parser = argparse.ArgumentParser(description="Process and merge vehicle and sensor data for a single vehicle.")
    parser.add_argument("vehicle_id", help="Vehicle ID (e.g., 197 or 201)")
    parser.add_argument("--input_dir", default=".", help="Input directory containing all data files")
    parser.add_argument("--output_dir", default=".", help="Output directory for processed data")

    args = parser.parse_args()

    # Create output directory if it doesn't exist
    os.makedirs(args.output_dir, exist_ok=True)

    process_data(args.vehicle_id, args.input_dir, args.output_dir)

if __name__ == "__main__":
    main()
