import json
import math

def read_json_file(filename):
    data = []
    with open(filename, 'r') as f:
        for line in f:
            data.append(json.loads(line))
    return data

def calculate_distance(pos1, pos2):
    return math.sqrt((pos1['posx'] - pos2['posx'])**2 + (pos1['posy'] - pos2['posy'])**2)

# Read input files
node_0_data = read_json_file('output_node_0.json')
node_1_data = read_json_file('output_node_1.json')

# Ensure both files have the same number of entries
assert len(node_0_data) == len(node_1_data), "Files have different number of entries"

# Calculate relative distances
relative_distances = []
new_timestamp = 0
for entry_0, entry_1 in zip(node_0_data, node_1_data):
    # Ensure timestamps match
    assert entry_0['timestamp'] == entry_1['timestamp'], f"Timestamps don't match: {entry_0['timestamp']} != {entry_1['timestamp']}"
    
    timestamp = entry_0['timestamp']
    distance = calculate_distance(entry_0, entry_1)
    
    relative_distances.append({
        "new_timestamp": round(new_timestamp,1),
        "distance": distance
    })
    new_timestamp += 0.1 
# Save results to veins_distance.json
with open('veins_distance.json', 'w') as f:
    for entry in relative_distances:
        json.dump(entry, f)
        f.write('\n')

print("Relative distances calculated and saved to veins_distance.json")
