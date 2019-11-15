from PIL import Image
import numpy as np
import math

n, h, alpha, height, width = input().split()
n = int(n)
h = int(h)
alpha = int(alpha)*math.pi/180
height = int(height)
width = int(width)
b = 2*h*math.tan(alpha/2)
pixels = []
for i in range(n):
    pixels.append(input().split())
shots = []
for s in range(n):
    img = []
    count = 0
    for i in range(height):
        row = []
        for j in range(width):
            colors = (int(pixels[s][count][0:2], 16), int(pixels[s][count][2:4], 16), int(pixels[s][count][4:6], 16))
            row.append(colors)
            count += 1
        img.append(row)
    shots.append(img)

# print(shots[1])
array = np.array(shots[0], dtype=np.uint8)
image = Image.fromarray(array)
# image.show()

# for s in range(n):
for i in range(height):
    for j in range(width):
        if shots[n-1][i][j] == shots[0][math.floor(height/2-1)][math.floor(width/2-1)]:
            # print(s, i, j)
            dist_px = math.floor(width/2-1) - i
            dist_py = math.floor(height/2-1) - j
            dist_mx = b * dist_px / width
            dist_my = b * dist_py / height
            if dist_mx != 0 and dist_my != 0:
                print("%.1f" % dist_mx, "%.1f" % dist_my)
                exit(0)
