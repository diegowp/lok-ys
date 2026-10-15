import struct
import json

with open('assets/models/cessna172.glb', 'rb') as f:
    data = f.read()

pos = 12
chunk_len, chunk_type = struct.unpack('<II', data[pos:pos+8])
json_data = json.loads(data[pos+8:pos+8+chunk_len].decode('utf-8'))
bin_offset = pos + 8 + chunk_len
bin_offset = (bin_offset + 3) & ~3
bin_chunk_len, bin_chunk_type = struct.unpack('<II', data[bin_offset:bin_offset+8])
bin_data = data[bin_offset+8:bin_offset+8+bin_chunk_len]

nodes = json_data.get('nodes', [])
meshes = json_data.get('meshes', [])
accessors = json_data.get('accessors', [])
buffer_views = json_data.get('bufferViews', [])

# Find wheel and wing meshes
for n_idx, n in enumerate(nodes):
    name = n.get('name', '')
    m_idx = n.get('mesh')
    if m_idx is not None:
        m_name = meshes[m_idx].get('name', '')
        # print some names
        if any(keyword in name.lower() or keyword in m_name.lower() for keyword in ['wheel', 'tire', 'gear', 'wing', 'fuselage', 'rudder', 'propeller', 'body']):
            prim = meshes[m_idx]['primitives'][0]
            pos_acc = accessors[prim['attributes']['POSITION']]
            bv = buffer_views[pos_acc['bufferView']]
            byte_offset = bv.get('byteOffset', 0) + pos_acc.get('byteOffset', 0)
            count = pos_acc['count']
            # read first 5 vertices
            verts = []
            for i in range(min(count, 5)):
                vx, vy, vz = struct.unpack('<fff', bin_data[byte_offset + i*12 : byte_offset + i*12 + 12])
                verts.append((vx, vy, vz))
            min_v = pos_acc.get('min')
            max_v = pos_acc.get('max')
            print(f"Node {n_idx} '{name}' -> Mesh {m_idx} '{m_name}': min={min_v}, max={max_v}")
