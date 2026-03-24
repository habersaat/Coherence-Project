import sys

data = []
with open('test.hex') as f:
    for line in f:
        line = line.strip()
        if line.startswith('@'):
            addr = int(line[1:], 16)
            while len(data) < addr:
                data.append(0)
        else:
            for b in line.split():
                data.append(int(b, 16))

# Pad to word boundary
while len(data) % 4:
    data.append(0)

with open('test_words.hex', 'w') as f:
    for i in range(0, len(data), 4):
        word = data[i] | (data[i+1] << 8) | (data[i+2] << 16) | (data[i+3] << 24)
        f.write(f'{word:08x}\n')
