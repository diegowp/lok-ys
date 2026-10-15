import struct
import json
import numpy as np

with open('assets/models/cessna172.glb', 'rb') as f:
    data = f.read()

pos = 12
chunk_len, chunk_type = struct.unpack('<II', data[pos:pos+8])
json_data = json.loads(data[pos+8:pos+8+chunk_len].decode('utf-8'))
bin_offset = (pos + 8 + chunk_len + 3) & ~3
bin_chunk_len, bin_chunk_type = struct.unpack('<II', data[bin_offset:bin_offset+8])
bin_data = data[bin_offset+8:bin_offset+8+bin_chunk_len]

nodes = json_data.get('nodes', [])
meshes = json_data.get('meshes', [])
accessors = json_data.get('accessors', [])
buffer_views = json_data.get('bufferViews', [])

def get_node_matrix(node):
    if 'matrix' in node:
        m = node['matrix']
        return np.array([
            [m[0], m[1], m[2], m[3]],
            [m[4], m[5], m[6], m[7]],
            [m[8], m[9], m[10], m[11]],
            [m[12], m[13], m[14], m[15]]
        ], dtype=np.float32)
    else:
        return np.eye(4, dtype=np.float32)

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

# Now test transformed vertices of wings (Node 74, Mesh 26) and wheels (Node 114, Mesh 46)
def get_transformed_bounds(node_idx):
    n = nodes[node_idx]
    m_idx = n['mesh']
    prim = meshes[m_idx]['primitives'][0]
    pos_acc = accessors[prim['attributes']['POSITION']]
    bv = buffer_views[pos_acc['bufferView']]
    offset = bv.get('byteOffset', 0) + pos_acc.get('byteOffset', 0)
    count = pos_acc['count']
    
    w_mat = world_mats[node_idx]
    min_pt = np.array([1e30, 1e30, 1e30])
    max_pt = np.array([-1e30, -1e30, -1e30])
    
    for i in range(count):
        vx, vy, vz = struct.unpack('<fff', bin_data[offset + i*12 : offset + i*12 + 12])
        p = np.array([vx, vy, vz, 1.0], dtype=np.float32)
        pw = p @ w_mat
        min_pt = np.minimum(min_pt, pw[:3])
        max_pt = np.maximum(max_pt, pw[:3])
        
    return min_pt, max_pt

min_wing, max_wing = get_transformed_bounds(74)
print(f"WING (Node 74) World Bounds: min={min_wing}, max={max_wing}")

min_wheel, max_wheel = get_transformed_bounds(114)
print(f"WHEEL (Node 114) World Bounds: min={min_wheel}, max={max_wheel}")
