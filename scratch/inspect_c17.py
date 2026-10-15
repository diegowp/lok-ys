import struct
import json

def inspect_c17(filename):
    print(f"Inspecting {filename}...")
    with open(filename, 'rb') as f:
        data = f.read()
        
    magic, version, length = struct.unpack('<III', data[:12])
    print(f"Magic: {hex(magic)}, Version: {version}, Length: {length}, File size: {len(data)}")
    
    pos = 12
    json_data = None
    while pos < length:
        chunk_len, chunk_type = struct.unpack('<II', data[pos:pos+8])
        pos += 8
        if chunk_type == 0x4E4F534A:
            json_text = data[pos:pos+chunk_len].decode('utf-8')
            json_data = json.loads(json_text)
            break
        pos += chunk_len
        pos = (pos + 3) & ~3
        
    if not json_data:
        print("No JSON chunk found!")
        return
        
    print(f"Nodes: {len(json_data.get('nodes', []))}")
    print(f"Meshes: {len(json_data.get('meshes', []))}")
    print(f"Accessors: {len(json_data.get('accessors', []))}")
    print(f"BufferViews: {len(json_data.get('bufferViews', []))}")
    print(f"Buffers: {len(json_data.get('buffers', []))}")
    
    # Check if there are extensions or unsupported features
    if 'extensionsRequired' in json_data:
        print(f"extensionsRequired: {json_data['extensionsRequired']}")
    if 'extensionsUsed' in json_data:
        print(f"extensionsUsed: {json_data['extensionsUsed']}")
        
    # Check node 0
    nodes = json_data.get('nodes', [])
    for idx, node in enumerate(nodes[:10]):
        print(f"Node {idx} '{node.get('name')}': mesh={node.get('mesh')}, matrix={node.get('matrix') is not None}, rot={node.get('rotation') is not None}")

if __name__ == '__main__':
    inspect_c17('assets/models/c-17a_globemaster_iii.glb')
