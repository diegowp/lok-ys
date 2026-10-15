import struct
import json

with open('assets/models/cessna172.glb', 'rb') as f:
    data = f.read()

pos = 12
chunk_len, chunk_type = struct.unpack('<II', data[pos:pos+8])
json_data = json.loads(data[pos+8:pos+8+chunk_len].decode('utf-8'))
nodes = json_data.get('nodes', [])

def print_node_tree(idx, indent=0):
    node = nodes[idx]
    prefix = "  " * indent
    name = node.get('name', '')
    mesh = node.get('mesh')
    has_mat = 'matrix' in node
    has_rot = 'rotation' in node
    has_trans = 'translation' in node
    has_scale = 'scale' in node
    
    info = []
    if mesh is not None: info.append(f"mesh={mesh}")
    if has_mat: info.append(f"matrix={node['matrix']}")
    if has_rot: info.append(f"rot={node['rotation']}")
    if has_trans: info.append(f"trans={node['translation']}")
    if has_scale: info.append(f"scale={node['scale']}")
    
    print(f"{prefix}Node {idx} '{name}': {', '.join(info)}")
    for child in node.get('children', []):
        print_node_tree(child, indent + 1)

scene = json_data.get('scene', 0)
for r in json_data.get('scenes', [])[scene].get('nodes', []):
    print_node_tree(r)
