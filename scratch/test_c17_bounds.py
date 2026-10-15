import struct
import json
import numpy as np

with open('assets/models/c-17a_globemaster_iii.glb', 'rb') as f:
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
    elif 'translation' in node or 'rotation' in node or 'scale' in node:
        t = node.get('translation', [0, 0, 0])
        r = node.get('rotation', [0, 0, 0, 1])
        s = node.get('scale', [1, 1, 1])
        # build matrix
        t_mat = np.eye(4, dtype=np.float32)
        t_mat[3, 0:3] = t
        s_mat = np.diag([s[0], s[1], s[2], 1.0]).astype(np.float32)
        # quat to rot
        qx, qy, qz, qw = r
        r_mat = np.eye(4, dtype=np.float32)
        r_mat[0, 0] = 1 - 2*(qy*qy + qz*qz)
        r_mat[0, 1] = 2*(qx*qy + qw*qz)
        r_mat[0, 2] = 2*(qx*qz - qw*qy)
        r_mat[1, 0] = 2*(qx*qy - qw*qz)
        r_mat[1, 1] = 1 - 2*(qx*qx + qz*qz)
        r_mat[1, 2] = 2*(qy*qz + qw*qx)
        r_mat[2, 0] = 2*(qx*qz + qw*qy)
        r_mat[2, 1] = 2*(qy*qz - qw*qx)
        r_mat[2, 2] = 1 - 2*(qx*qx + qy*qy)
        return s_mat @ r_mat @ t_mat
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

min_total = np.array([1e30, 1e30, 1e30])
max_total = np.array([-1e30, -1e30, -1e30])

for idx, n in enumerate(nodes):
    m_idx = n.get('mesh')
    if m_idx is not None:
        for prim in meshes[m_idx]['primitives']:
            pos_acc_idx = prim['attributes']['POSITION']
            pos_acc = accessors[pos_acc_idx]
            bv = buffer_views[pos_acc['bufferView']]
            offset = bv.get('byteOffset', 0) + pos_acc.get('byteOffset', 0)
            count = pos_acc['count']
            w_mat = world_mats[idx]
            for i in range(min(count, 50)): # sample
                vx, vy, vz = struct.unpack('<fff', bin_data[offset + i*12 : offset + i*12 + 12])
                p = np.array([vx, vy, vz, 1.0], dtype=np.float32)
                pw = p @ w_mat
                min_total = np.minimum(min_total, pw[:3])
                max_total = np.maximum(max_total, pw[:3])

print(f"C-17 Globemaster Sample World Bounds:")
print(f"  Min: {min_total}")
print(f"  Max: {max_total}")
print(f"  Dim: {max_total - min_total}")
