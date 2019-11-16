from math import *
import numpy as np

d, w1, w2, w3, t = input().split()
r = int(d)/2
v1 = float(w1)*r
v2 = float(w2)*r
v3 = float(w3)*r
t = int(t)
alpha = 0
y_dot = data[0][0] * sin(alpha) + data[1][0] * (-sin(alpha) / 2 - sqrt(3) * cos(alpha) / 2) + data[2][0] * (
            -sin(alpha) / 2 + sqrt(3) * cos(alpha) / 2)
x_dot = data[0][0] * cos(alpha) + data[1][0] * (sqrt(3) * sin(alpha) / 2 - cos(alpha) / 2) + data[2][0] * (
            -sqrt(3) * sin(alpha) / 2 - cos(alpha) / 2)
# y_dot = - v2*sqrt(3)/2 + v3*sqrt(3)/2
# x_dot = v1 - v2 / 2 - v3 / 2
y = y_dot*t
x = x_dot*t
print("%.2f" % x, "%.2f" % y)
