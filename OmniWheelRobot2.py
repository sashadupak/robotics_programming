from math import *
import numpy as np
from operator import itemgetter

d, w1, t1, w2, t2, w3, t3 = input().split()
r = int(d)/2
t1 = int(t1)
t2 = int(t2)
t3 = int(t3)
alpha = 0
v1 = float(w1)*r
v2 = float(w2)*r
v3 = float(w3)*r
data = [[v1, t1], [v2, t2], [v3, t3]]
ds = sorted(data, key=itemgetter(1))
t = [ds[0][1], ds[1][1]-ds[0][1], ds[2][1]-ds[1][1]]
x = 0
y = 0
# print(ds)
for i in range(3):
    # print(data)
    # print(t[i])
    y_dot = data[0][0]*sin(alpha) + data[1][0]*(-sin(alpha)/2 - sqrt(3)*cos(alpha)/2) + data[2][0]*(-sin(alpha)/2 + sqrt(3)*cos(alpha)/2)
    x_dot = data[0][0]*cos(alpha) + data[1][0]*(sqrt(3)*sin(alpha)/2 - cos(alpha)/2) + data[2][0]*(-sqrt(3)*sin(alpha)/2 - cos(alpha)/2)
    y += y_dot*t[i]
    x += x_dot*t[i]
    for j in range(3):
        if ds[i][1] == data[j][1]:
            data[j][0] = 0
    # print(x, y)
print("%.2f" % (x/2), "%.2f" % (y/2))

# angle = 0:
# y_dot = - v2 * sqrt(3) / 2 + v3 * sqrt(3) / 2
# x_dot = v1 - v2 / 2 - v3 / 2
# angle = 90:
# y_dot = v1 - v2/2 - v3/2
# x_dot = v2 *sqrt(3)/2 - v3 * sqrt(3)/2
