import math
import matplotlib.pyplot as plt


def line_cross(x1a, y1a, x2a, y2a, x1b, y1b, x2b, y2b):
    A1 = y1a - y2a
    B1 = x2a - x1a
    C1 = x1a * y2a - x2a * y1a
    A2 = y1b - y2b
    B2 = x2b - x1b
    C2 = x1b * y2b - x2b * y1b
    # x = (Bb*Ca - Ba*Cb) / (Ba*Ab - Bb*Aa)
    # y = (-Aa*x-Ca)/Ba
    if B1 * A2 - B2 * A1 != 0:
        # y = (C2 * A1 - C1 * A2) / (B1 * A2 - B2 * A1)
        # x = (-C1 - B1 * y) / A1
        x_c = (B1 * C2 - B2 * C1) / (B2 * A1 - B1 * A2)
        y_c = (-A1 * x_c - C1) / B1
        if min(x1a, x2a) <= x_c <= max(x1a, x2a) and min(y1a, y2a) <= y_c <= max(y1a, y2a) and  \
                min(x1b, x2b) <= x_c <= max(x1b, x2b) and min(y1b, y2b) <= y_c <= max(y1b, y2b):
            return True, x_c, y_c
        else:
            return False, 0, 0
    else:
        return False, 0, 0


N, dt = input().split()
N = int(N)
dt = float(dt.replace(",", "."))
x = []
y = []
angle = 0
path = 0
x_old = 0
y_old = 0
for i in range(N):
    v, w = input().split()
    v = float(v.replace(",", "."))
    w = float(w.replace(",", "."))
    angle = angle + w*dt
    dpath = v*dt
    x.append(x_old + dpath*math.cos(angle))
    y.append(y_old + dpath*math.sin(angle))
    for j in range(1, i-1):
        cross, x_cross, y_cross = line_cross(x[i], y[i], x_old, y_old, x[j], y[j], x[j-1], y[j-1])
        if cross:
            #print(x_cross, y_cross)
            dist = math.sqrt(x_cross ** 2 + y_cross ** 2)
            print(int(dist))
            exit(0)
    x_old = x[i]
    y_old = y[i]
#print(x_cross, y_cross)
#plt.plot(x, y)
#plt.show()

