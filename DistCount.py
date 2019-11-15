import math
# mport matplotlib.pyplot as plt


def dist(Mx, My, x1, y1, x2, y2):
    A = y1 - y2
    B = x2 - x1
    C = x1 * y2 - x2 * y1
    d = abs(A*Mx + B*My + C)/math.sqrt(A**2 + B**2)
    return d


x1, y1 = input().split()
x1 = int(x1)/1000
y1 = int(y1)/1000
x2, y2 = input().split()
x2 = int(x2)/1000
y2 = int(y2)/1000
n = int(input())
angle = 0
prev_path = 0
x = 0
y = 0
r = 0.028
B = 0.170
min_dist = dist(0, 0, x1, y1, x2, y2)
xa = []
ya = []
for i in range(n):
    left, right = input().split()
    left = int(left)*math.pi/180
    right = int(right)*math.pi/180
    path = (right + left)*(r/2)
    dpath = path - prev_path
    prev_path = path
    angle = (right - left)*(r/B)
    x += math.cos(angle)*dpath
    y += math.sin(angle)*dpath
    cur_dist = dist(x, y, x1, y1, x2, y2)
    if cur_dist < min_dist:
        min_dist = cur_dist
    xa.append(x)
    ya.append(y)
print(int(min_dist*1000))
# plt.plot(xa, ya)
# plt.show()
