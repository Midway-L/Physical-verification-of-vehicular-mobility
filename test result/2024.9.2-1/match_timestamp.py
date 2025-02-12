import json
import argparse
from typing import Dict, List
from collections import defaultdict

def load_json_file(file_path: str) -> List[Dict]:
    with open(file_path, 'r') as f:
        return [json.loads(line) for line in f]

def find_duplicates(data: List[Dict]) -> Dict[float, List[Dict]]:
    timestamp_dict = defaultdict(list)
    for point in data:
        timestamp_dict[point['timestamp']].append(point)
    return {ts: points for ts, points in timestamp_dict.items() if len(points) > 1}

def compare_timestamps(file1_data: List[Dict], file2_data: List[Dict]) -> Dict:
    file1_dict = {point['original_timestamp']: point for point in file1_data}
    file2_dict = defaultdict(list)
    for point in file2_data:
        file2_dict[point['timestamp']].append(point)
    
    mismatched_points = []
    extra_in_file2 = []
    missing_in_file2 = []
    duplicates_in_file2 = find_duplicates(file2_data)
    
    for timestamp, points in file2_dict.items():
        if timestamp not in file1_dict:
            extra_in_file2.extend(points)
        else:
            point1 = file1_dict[timestamp]
            for point2 in points:
                if point1['x'] != point2['x'] or point1['y'] != point2['y']:
                    mismatched_points.append({
                        'file1_point': point1,
                        'file2_point': point2
                    })
    
    for timestamp, point1 in file1_dict.items():
        if timestamp not in file2_dict:
            missing_in_file2.append(point1)
    
    return {
        'mismatched': mismatched_points,
        'extra_in_file2': extra_in_file2,
        'missing_in_file2': missing_in_file2,
        'duplicates_in_file2': duplicates_in_file2
    }

def main():
    parser = argparse.ArgumentParser(description='Compare timestamps in two JSON files')
    parser.add_argument('file1', help='Path to the first JSON file')
    parser.add_argument('file2', help='Path to the second JSON file')
    args = parser.parse_args()

    file1_data = load_json_file(args.file1)
    file2_data = load_json_file(args.file2)

    print(f"File 1 contains {len(file1_data)} objects")
    print(f"File 2 contains {len(file2_data)} objects")

    comparison_results = compare_timestamps(file1_data, file2_data)

    if comparison_results['mismatched']:
        print(f"\nFound {len(comparison_results['mismatched'])} mismatched points:")
        for point in comparison_results['mismatched']:
            print(f"File 1: {point['file1_point']}")
            print(f"File 2: {point['file2_point']}")
            print("---")
    else:
        print("\nNo mismatched points found.")

    if comparison_results['extra_in_file2']:
        print(f"\nFound {len(comparison_results['extra_in_file2'])} extra points in File 2:")
        for point in comparison_results['extra_in_file2']:
            print(point)
            print("---")
    else:
        print("\nNo extra points found in File 2.")

    if comparison_results['missing_in_file2']:
        print(f"\nFound {len(comparison_results['missing_in_file2'])} points missing in File 2:")
        for point in comparison_results['missing_in_file2']:
            print(point)
            print("---")
    else:
        print("\nNo missing points found in File 2.")

    if comparison_results['duplicates_in_file2']:
        print(f"\nFound {len(comparison_results['duplicates_in_file2'])} duplicate timestamps in File 2:")
        for timestamp, points in comparison_results['duplicates_in_file2'].items():
            print(f"Timestamp {timestamp} appears {len(points)} times:")
            for point in points:
                print(point)
            print("---")
    else:
        print("\nNo duplicate timestamps found in File 2.")

if __name__ == "__main__":
    main()
