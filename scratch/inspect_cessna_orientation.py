import struct
import json

with open('assets/models/cessna172.glb', 'rb') as f:
    data = f.read()

pos = 12
chunk_len, chunk_type = struct.unpack('<II', data[pos:pos+8])
json_data = json.loads(data[pos+8:pos+8+chunk_len].decode('utf-8'))

nodes = json_data.get('nodes', [])
for idx, node in enumerate(nodes):
    for k in node.keys():
        if k not in ['name', 'mesh', 'children', 'skin']:
            print(f"Node {idx} has key '{k}': {node[k]}")
