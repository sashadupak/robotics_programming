import math

m, n, alpha, h = input().split()
m = int(m)
n = int(n)
alpha = float(alpha)*math.pi/180
h = float(h)/100
k = int(input())
t = 1
b = 2*h*math.tan(alpha/2)  # m в метрах
shots = []
for i in range(k):
    shot = []
    for j in range(m):
        line = input()
        shot.append(line)
    shots.append(shot)
default_line = []
for j in range(m):
    default_line.append(shots[k-1][j])

i = k - 2
for j in range(m):
    if default_line[0] in shots[i][j]:
        correct = 0
        for line in range(1, m-j):
            if default_line[line] == shots[i][j+line]:
                correct += 1
        if correct == m - j - 1:
            dist_p = j/(k - 1 - i)  # расстояние в пикселях за одну секунду
            # print(dist_p)
            # здесь нужно провеирть что на последующих кадрах все теже самые строки начиная от dist_p/2,3,4,...
            for s in range(i, -1, -1):
                ok = -1
                # if ok >= 0:
                #     break

                dist_s = math.ceil(dist_p * (k - s - 1))
                # print(dist_s)
                if dist_s >= m:
                    # dist_p = m - 1
                    ok = 1
                    # print("min")
                    # reverse
                    break

                for r in range(dist_s, m):
                    if ok == 1:
                        break
                    if default_line[0] in shots[s][r]:
                        # print("next")
                        correct = 0
                        for line in range(1, m - r):
                            if default_line[line] == shots[s][r + line]:
                                correct += 1
                        if correct == m - r - 1:
                            # print("ok")
                            ok = 1
                            break
                        else:
                            # print("not ok")
                            ok = 0
                            break
                if ok == -1:
                    # print("no")
                    # dist_p = m - 1
                    ok = 1
                    break

            # print(dist_p)
            dist_m = b * dist_p / m  # расстояние за один кадр в метрах
            # print(100*b/m)
            # print("%.2f" % (dist_m * 100))
            dist_m = dist_m * k
            print("%.2f" % (dist_m*100))
            exit(0)
