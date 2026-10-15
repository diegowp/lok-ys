import struct
import json
import numpy as np

with open('assets/models/cessna172.glb', 'rb') as f:
    data = f.read()

pos = 12
chunk_len, chunk_type = struct.unpack('<II', data[pos:pos+8])
json_data = json.loads(data[pos+8:pos+8+chunk_len].decode('utf-8'))
nodes = json_data.get('nodes', [])

def get_node_matrix(node):
    if 'matrix' in node:
        # gltf 16 floats
        m = node['matrix']
        # in row-vector convention: Row 0 is m[0..3], Row 1 is m[4..7], Row 2 is m[8..11], Row 3 is m[12..15]
        return np.array([
            [m[0], m[1], m[2], m[3]],
            [m[4], m[5], m[6], m[7]],
            [m[8], m[9], m[10], m[11]],
            [m[12], m[13], m[14], m[15]]
        ], dtype=np.float32)
    else:
        # Identity or TRS
        # For this model, no TRS
        return np.eye(4, dtype=np.float32)

# Compute world transform for each node: child = parent * child (or child * parent)
# For row vectors: v_world = v_local * M_local * M_parent
# So M_world = M_local @ M_parent
world_mats = [None] * len(nodes)

def compute_world(idx, parent_world):
    local_m = get_node_matrix(nodes[idx])
    world_m = local_m @ parent_world
    world_mats[idx] = world_m
    for c in nodes[idx].get('children', []):
        compute_world(c, world_m)

scene = json_data.get('scene', 0)
for r in json_data.get('scenes', [])[scene].get('nodes', []):
    compute_world(r, np.eye(4, dtype=np.float32))

print("World matrix for Node 8 (interior paddle):")
print(world_mats[8])
print("\nWorld matrix for Node 50 (exterior body):")
print(world_mats[50])
print("\nWorld matrix for Node 68 (exterior body part):")
print(world_mats[68])
