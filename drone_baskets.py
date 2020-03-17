import sim as vrep
import time
from math import *
import numpy as np
import cv2
from operator import itemgetter

# just in case, close all opened connections
vrep.simxFinish(-1)

# connect to V-REP
# port is in remoteApiConnections.txt
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID != -1:
    print('Connected to remote API server')

# get handles
err, QuadricopterT = vrep.simxGetObjectHandle(
    clientID, 'Quadricopter_target', vrep.simx_opmode_oneshot_wait)
if err == -1:
    print("No Quadricopter")

err, Quadricopter = vrep.simxGetObjectHandle(
    clientID, 'Quadricopter', vrep.simx_opmode_oneshot_wait)
if err == -1:
    print("No Quadricopter")

err, cameraID = vrep.simxGetObjectHandle(
    clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)
if err == -1:
    print("No camera")

err, cameraPos = vrep.simxGetObjectPosition(
    clientID, cameraID, -1, vrep.simx_opmode_oneshot_wait)

err, Qpos = vrep.simxGetObjectPosition(
    clientID, Quadricopter, -1, vrep.simx_opmode_oneshot_wait)

err = vrep.simxSetObjectPosition(
        clientID, QuadricopterT, -1, (Qpos[0], Qpos[1], Qpos[2]), vrep.simx_opmode_oneshot)

err, Qang = vrep.simxGetObjectOrientation(
    clientID, Quadricopter, -1, vrep.simx_opmode_oneshot_wait)

err = vrep.simxSetObjectOrientation(
            clientID, QuadricopterT, -1, (0, 0, Qang[2]), vrep.simx_opmode_oneshot)

vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
print("start simulation")

max_vel = 0.005/0.02

def set_drone_position(point):
    err, Qpos0 = vrep.simxGetObjectPosition(
        clientID, Quadricopter, -1, vrep.simx_opmode_oneshot_wait)
    dist = sqrt((point[0] - Qpos[0])**2 + (point[1] - Qpos[1])**2 + (point[2] - Qpos[2])**2)
    tp = dist/max_vel
    t0 = time.time()
    while (time.time() - t0 < tp):
        t = time.time() - t0
        for j in range(3):
            Qpos[j] = ((point[j] - Qpos0[j])/2)*(1 - cos(pi*t/tp)) + Qpos0[j]
        err = vrep.simxSetObjectPosition(
            clientID, QuadricopterT, -1, (Qpos[0], Qpos[1], Qpos[2]), vrep.simx_opmode_oneshot)
        time.sleep(0.02)
    time.sleep(1)


def set_drone_orientation(angle):
    err, Qang = vrep.simxGetObjectOrientation(
        clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)
    i = Qang[2]
    while abs(Qang[2] - angle) > radians(2):
        err, Qang = vrep.simxGetObjectOrientation(
            clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)
        i += copysign(radians(1), angle - Qang[2])
        err = vrep.simxSetObjectOrientation(
            clientID, QuadricopterT, -1, (0, 0, i), vrep.simx_opmode_oneshot)
        time.sleep(0.1)
    time.sleep(1)


def get_image():
    err, resolution, image = vrep.simxGetVisionSensorImage(
        clientID, cameraID, 0, vrep.simx_opmode_streaming)
    time.sleep(0.1)
    err, resolution, image = vrep.simxGetVisionSensorImage(
        clientID, cameraID, 0, vrep.simx_opmode_buffer)
    img = np.array(image, dtype=np.uint8)
    img.resize([256, 256, 3])
    time.sleep(0.1)
    return img


def get_objects(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    """
    hsv_min = np.array((0, 0, 5), np.uint8)
    hsv_max = np.array((255, 250, 215), np.uint8)
    thresh = cv2.inRange(hsv, hsv_min, hsv_max)
    edged = cv2.Canny(thresh, 50, 100)
    # edged = cv2.dilate(edged, None, iterations=1)
    # edged = cv2.erode(edged, None, iterations=1)
    cdst = cv2.cvtColor(edged, cv2.COLOR_GRAY2BGR)
    lines = cv2.HoughLines(edged, 2, pi / 180, 150, None, 0, 0)
    if lines is not None:
        th = []
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = cos(theta)
            b = sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
            th.append(theta)
    theta = 0
    m = radians(360)
    for t in th:
        if abs(t) < m:
            theta = t
            m = abs(t)
    """
    err, Qang = vrep.simxGetObjectOrientation(
        clientID, Quadricopter, -1, vrep.simx_opmode_oneshot_wait)
    theta =  radians(130)+Qang[2]
    # print(theta)
    # cv2.imwrite("lines.png", cdst)
    bounds = [150, 30, 75]
    objects = {}
    for j in range(3):
        hsv_min = np.array((bounds[j]-10, 0, 0), np.uint8)
        hsv_max = np.array((bounds[j]+10, 255, 255), np.uint8)
        thresh = cv2.inRange(hsv, hsv_min, hsv_max)
        edged = cv2.Canny(thresh, 50, 100)
        edged = cv2.dilate(edged, None, iterations=1)
        edged = cv2.erode(edged, None, iterations=1)
        cnts, h = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        max_radius = 1
        for c in cnts:
            if cv2.contourArea(c) < 1:
                continue
            (x,y),radius = cv2.minEnclosingCircle(c)
            center = (int(x),int(y))
            radius = int(radius)
            cv2.circle(img,center,radius,(0,255,0),2)
            cv2.circle(img, center, 2, (0, 255, 0),2)
            if radius > max_radius:
                x = int(x) - 256/2
                y = int(y) - 256/2
                x = x*cos(theta) - y*sin(theta)
                y = x*sin(theta) + y*cos(theta)
                objects[j] = [x, y]
                max_radius = radius
    # cv2.imwrite("objects.png", img)
    # print(objects)
    return objects


set_drone_orientation(radians(-130))
set_drone_position([0, 0, 5])
err, Qang = vrep.simxGetObjectOrientation(
        clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)
img = get_image()
objects = get_objects(img)
if len(objects.keys()) < 3:
    set_drone_orientation(radians(50))
    img = get_image()
    objects1 = get_objects(img)
    for k, v in objects1.items():
        if k in objects.keys():
            continue
        objects[k] = v
objects = sorted(objects.items(), key=itemgetter(1), reverse=False)
describtion = ["Small", "Middle", "Big"]
print("Basket arrangement sequence:")
for obj in objects:
    print(str(obj[0]+1) + ". " + describtion[obj[0]])
    if obj[0] == 2:
        obj_pos_pixel = obj[1]
# print(obj_pos_pixel)

err, Qpos = vrep.simxGetObjectPosition(
    clientID, Quadricopter, -1, vrep.simx_opmode_oneshot_wait)
err, Qang = vrep.simxGetObjectOrientation(
        clientID, QuadricopterT, -1, vrep.simx_opmode_oneshot_wait)
pixel_size = (2*(5 - 0.25)/sqrt(3))/256
x = Qpos[0] + obj_pos_pixel[0]*pixel_size
y = Qpos[1] + obj_pos_pixel[1]*pixel_size
set_drone_position([x, y, 1])
img = get_image()
objects = get_objects(img)
objects = sorted(objects.items(), key=itemgetter(0), reverse=True)
obj_pos_pixel = objects[0][1]
pixel_size = (2*(1-0.25)/sqrt(3))/256
err, cameraPos = vrep.simxGetObjectPosition(
    clientID, cameraID, -1, vrep.simx_opmode_oneshot_wait)
x = cameraPos[0] + obj_pos_pixel[0]*pixel_size
y = cameraPos[1] + obj_pos_pixel[1]*pixel_size
objects = set_drone_position([x, y, 0.4])
time.sleep(2)
err, Qpos = vrep.simxGetObjectPosition(
    clientID, Quadricopter, -1, vrep.simx_opmode_oneshot_wait)

# Throw ball
emptyBuff = bytearray()
res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'Quadricopter', vrep.sim_scripttype_childscript,
                                                                             'ThrowBallFunction', [], [], [""], emptyBuff, vrep.simx_opmode_blocking)
time.sleep(1)
img = get_image()
cv2.imwrite("ball.png", img)

# finish work
vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)
time.sleep(1)
vrep.simxFinish(clientID)
print("stop simulation")
