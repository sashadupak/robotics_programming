import numpy
from math import *

q1, q2, q3 = input().split()
q1 = float(q1)
q2 = float(q2)
q3 = float(q3)
a1 = 0
a2 = 2
a3 = 1
d1 = 3
d2 = 0
d3 = 0
alpha1 = pi/2
alpha2 = 0
alpha3 = 0
T1 = numpy.mat([[cos(q1), -sin(q1)*cos(alpha1), sin(q1)*sin(alpha1), a1*cos(q1)],
               [sin(q1), cos(q1)*cos(alpha1), -cos(q1)*sin(alpha1), a1*sin(q1)],
               [0, sin(alpha1), cos(alpha1), d1],
               [0, 0, 0, 1]])
T2 = numpy.mat([[cos(q2), -sin(q2)*cos(alpha2), sin(q2)*sin(alpha2), a2*cos(q2)],
               [sin(q2), cos(q2)*cos(alpha2), -cos(q2)*sin(alpha2), a2*sin(q2)],
               [0, sin(alpha2), cos(alpha2), d2],
               [0, 0, 0, 1]])
T3 = numpy.mat([[cos(q3), -sin(q3)*cos(alpha3), sin(q3)*sin(alpha3), a3*cos(q3)],
               [sin(q3), cos(q3)*cos(alpha3), -cos(q3)*sin(alpha3), a3*sin(q3)],
               [0, sin(alpha3), cos(alpha3), d3],
               [0, 0, 0, 1]])
T = T1*T2*T3
x = T.item((0, 3))
y = T.item((1, 3))
z = T.item((2, 3))
print("%.2f" % x, "%.2f" % y, "%.2f" % z)
