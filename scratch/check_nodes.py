import re

with open('Ysflight/aircraft/cessna172r.dnm', 'r', encoding='latin1') as f:
    text = f.read()

nodes = re.findall(r'SRF \"([^\"]+)\".*?(?=END)', text, re.DOTALL)
print('Found nodes count:', len(nodes))
for n in nodes:
    lines = n.splitlines()
    name = lines[0]
    pos = (0,0,0)
    cla = 'None'
    for line in lines:
        parts = line.strip().split()
        if len(parts) >= 4 and parts[0] == 'POS':
            pos = (float(parts[1]), float(parts[2]), float(parts[3]))
        if len(parts) >= 2 and parts[0] == 'CLA':
            cla = parts[1]
    print(f'{name:20} CLA={cla:4} POS={pos}')
