import json
import matplotlib.pyplot as plt

def read_json_file(filename):
    timestamps = []
    distances = []
    with open(filename, 'r') as f:
        for line in f:
            data = json.loads(line)
            timestamps.append(data['timestamp'])
            distances.append(data['distance'])
    return timestamps, distances

filenames = ['cars_distance.json', 'processed_lidar_data_147.json', 'processed_lidar_data_151.json', 'rssi_distance.json']
colors = ['red', 'blue', 'green', 'purple']
labels = ['ground truth', 'vehicle 147 lidar', 'vehicle 151 lidar', 'rssi']

plt.figure(figsize=(12, 6))

for filename, color, label in zip(filenames, colors, labels):
    timestamps, distances = read_json_file(filename)
    plt.plot(timestamps, distances, color=color, label=label)

plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.title('Distance vs Time')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

output_filename = 'distance_vs_time.png'
plt.savefig(output_filename, dpi=300, bbox_inches='tight')
print(f"save to {output_filename}")
