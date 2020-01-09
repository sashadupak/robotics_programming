from math import *
# import matplotlib.pyplot as plt
from operator import itemgetter

table = {}
for i in range(1, 13):
    table[i] = [[], []]
n = int(input())
for i in range(n):
    index, ice_x, ice_y, drone_x, drone_y, height = input().split()
    index = int(index)
    ice_x = int(ice_x)
    ice_y = int(ice_y)
    drone_x = float(drone_x)
    drone_y = float(drone_y)
    height = float(height) - 4
    pixel_size = height/480
    # a = 2*height*tan(22.5*pi/180)  # стороны картинки в метрах
    # b = 2*height*tan(21.145*pi/180)
    # ice_x = drone_x + a*(ice_x - 640/2)/640  # координаты людин в м от точки отсчета базовой СК
    # ice_y = drone_y + b*(-ice_y + 480/2)/480
    ice_x = drone_x + (ice_x - 320)*pixel_size
    ice_y = drone_y + (240 - ice_y)*pixel_size
    table[index][0].append(ice_x)
    table[index][1].append(ice_y)

colors = ['bo', 'go', 'ro', 'co', 'mo', 'yo', 'ko', 'bs', 'gs', 'rs', 'cs', 'ms']

# отсекаем лишние точки
for p in range(1, 13):
    while max(table[p][0]) - min(table[p][0]) > 5 or max(table[p][1]) - min(table[p][1]) > 20:
        if len(table[p][0]) <= 6:
            break
        if max(table[p][0]) - min(table[p][0]) > 5:
            i_max = table[p][0].index(max(table[p][0]))
            i_min = table[p][0].index(min(table[p][0]))
            d_sum = 0
            for j in range(len(table[p][0])):
                d_sum += sqrt((table[p][0][i_max] - table[p][0][j]) ** 2 + (table[p][1][i_max] - table[p][1][j]) ** 2)
            d_avr = d_sum / len(table[p][0])
            for j in range(len(table[p][0])):
                d_sum += sqrt((table[p][0][i_min] - table[p][0][j]) ** 2 + (table[p][1][i_min] - table[p][1][j]) ** 2)
            d_avr1 = d_sum / len(table[p][0])
            if d_avr1 > d_avr:
                table[p][0].remove(table[p][0][i_min])
                table[p][1].remove(table[p][1][i_min])
            else:
                table[p][0].remove(table[p][0][i_max])
                table[p][1].remove(table[p][1][i_max])
        elif max(table[p][1]) - min(table[p][1]) > 20:
            i_max = table[p][1].index(max(table[p][1]))
            i_min = table[p][1].index(min(table[p][1]))
            d_sum = 0
            for j in range(len(table[p][0])):
                d_sum += sqrt((table[p][0][i_max] - table[p][0][j]) ** 2 + (table[p][1][i_max] - table[p][1][j]) ** 2)
            d_avr = d_sum / len(table[p][0])
            for j in range(len(table[p][0])):
                d_sum += sqrt((table[p][0][i_min] - table[p][0][j]) ** 2 + (table[p][1][i_min] - table[p][1][j]) ** 2)
            d_avr1 = d_sum / len(table[p][0])
            if d_avr1 > d_avr:
                table[p][0].remove(table[p][0][i_min])
                table[p][1].remove(table[p][1][i_min])
            else:
                table[p][0].remove(table[p][0][i_max])
                table[p][1].remove(table[p][1][i_max])

    # plt.plot(table[p][0], table[p][1], colors[p-1])
    # находим средние точки и выводим
    x_avr = sum(table[p][:][0])/len(table[p][:][0])
    y_avr = sum(table[p][:][1])/len(table[p][:][1])
    table[p] = [x_avr, y_avr]
    # plt.plot(x_avr, y_avr, colors[p-1][0] + '+')
    # plt.text(x_avr, y_avr, str(p))

# дальше пытаемся разбить точки по секторам
points = []
for i, [x, y] in table.items():
    points.append([i, x, y])
points.sort(key=lambda row: row[2], reverse=True)

lines = []
lines.append(sorted([points[0], points[1], points[2]], key=itemgetter(1)))
lines.append(sorted([points[3], points[4], points[5]], key=itemgetter(1)))
lines.append(sorted([points[6], points[7], points[8]], key=itemgetter(1)))
lines.append(sorted([points[9], points[10], points[11]], key=itemgetter(1)))
for i in range(4):
    print(lines[i][0][0], lines[i][1][0], lines[i][2][0])
    # for j in range(3):
    #     plt.text(lines[i][j][1], lines[i][j][2], str(lines[i][j][0]))
# plt.show()
