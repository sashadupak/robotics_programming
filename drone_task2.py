#!/usr/bin/env python
# coding: utf-8
import rospy
import threading
import time
import sys
import tf.transformations as tftr
from numpy import *

from geometry_msgs.msg import Pose, Point, Vector3
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty
from tello_driver.msg import TelloStatus


lock = threading.Lock()
file_obj = open("drone_data.txt", "w")


class Controller:

    def __init__(self):
        self.status = None
        self.odometry = None
        self.i = 0
        self.start_odom = None

        "ROS stuff"
        self.status_sub = rospy.Subscriber("/tello/status", TelloStatus, self.tello_status_callback)
        self.odom_sub = rospy.Subscriber("/tello/odom", Odometry, self.odometry_callback)
        self.imu_sub = rospy.Subscriber("/tello/imu", Imu, self.imu_callback)
        self.takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)
        #self.emergency_pub = rospy.Publisher('/tello/emergency', Empty, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.way_points = [[-1.25, -1.25, -2]]
        self.max_velocity = 1
        self.position_error = 0
        self.first_point = True
        self.vel = [0, 0, 0]
        self.flag = 1
        print("Start")
        self.step = 0
        self.zero_t = 0
        self.f1 = 0

    def start(self):
        attempt = 1
        while (self.status == None) or not self.status.is_flying:
            time.sleep(1)
            print("Trying to take off... (atempt: " + str(attempt) + ")")
            self.takeoff_pub.publish(Empty())
            attempt += 1
            if attempt > 10:
                print("Cannot take off. Please check battery power.")
                exit(0)

    def tello_status_callback(self, msg):
        lock.acquire()
        self.status = msg
        lock.release()

    def odometry_callback(self, msg):
        lock.acquire()
        self.odometry = msg
        lock.release()

    def imu_callback(self, msg):
        lock.acquire()
        self.imu = msg
        lock.release()

    def stop(self):
        time.sleep(1)
        self.land_pub.publish(Empty())
        time.sleep(5)
        self.status_sub.unregister()
        self.odom_sub.unregister()
        self.imu_sub.unregister()

    def setup(self, i):
        start_odom_xyz = self.start_odom.pose.pose.position
        xyz = self.odometry.pose.pose.position
        xyz.x -= start_odom_xyz.x
        xyz.y -= start_odom_xyz.y

        #self.start_position = [xyz.x, xyz.y, xyz.z]
        if i == 0:
            self.start_position = [0, 0, 0]
        else:
            self.start_position = self.way_points[i-1]
        self.desired_position = self.way_points[i]
        dist = sqrt((self.desired_position[0] - self.start_position[0])**2 +
                    (self.desired_position[1] - self.start_position[1])**2 +
                    (self.desired_position[2] - self.start_position[2])**2)
        self.tp = dist/self.max_velocity
        #print("tp", self.tp)
        self.t0 = time.time() - self.start_time

    def rotate(self, x, y, z, yaw, pitch, roll):
        Rx = mat([[1, 0, 0],
             [0, cos(roll), -sin(roll)],
             [0, sin(roll), cos(roll)]])
        Ry = mat([[cos(pitch), 0, sin(pitch)],
             [0, 1, 0],
             [-sin(pitch), 0, cos(pitch)]])
        Rz = mat([[cos(yaw), -sin(yaw), 0],
             [sin(yaw), cos(yaw), 0],
             [0, 0, 1]])
        R = Rz*Ry*Rx
        X = R*mat([[x], [y], [z]])
        return X

    def update(self, dt, t):
        t1 = time.time() - self.start_time
        if self.odometry == None:
            return

        #if self.start_odom == None:
        #    self.start_odom = self.odometry
        #    self.setup(0)

        if t > 90:
            self.stop()
            exit(0)

        velocity = Twist()
        if self.step == 0: #set h
            xyz = self.odometry.pose.pose.position
            des_h = self.way_points[0][2]
            h_err = xyz.z - des_h
            velocity.linear.x = 0
            velocity.linear.y = 0
            velocity.linear.z = 1*h_err
            if abs(h_err) < 0.15:
                print("h<0.15")
                if self.f1 == 0:
                    print("zero_time")
                    self.zero_t = time.time()
                    self.f1 = 1
                else:
                    print("next step)
                    if time.time() - self.zero_t > 10:
                        self.step = 1
        elif self.step == 1: #set pos
            print("Next")
            xyz = self.odometry.pose.pose.position
            q = self.odometry.pose.pose.orientation     # quaternion
            rpy = tftr.euler_from_quaternion((q.x, q.y, q.z, q.w))  # roll pitch yaw
            if (time.time() - self.start_time < self.tp):
                v = []
                for j in range(3):
                    v.append((math.pi*(self.desired_position[j] - self.start_position[j])*math.sin((math.pi*(t1 - self.t0))/self.tp))/(2*self.tp))
                velocity.linear.x = v[0] #+ 0.1*(v[0] - X[0])
                velocity.linear.y = v[1] #+ 0.1*(v[1] - X[1])
                velocity.linear.z = 0
            else:
                velocity.linear.x = 0.0
                velocity.linear.y = 0.0
                #velocity.angular.z = 0.0
                print("Complete1")

                #ang
                velocity.angular.z = 1*(radians(180) - rpy[2])
        self.cmd_vel_pub.publish(velocity)
        #test = str(self.t0) + ' ' + str(t1) + ' ' + str(self.tp)
        pos = str(self.desired_position[j] - (xyz.x - self.x_offset)) + ' ' + str(xyz.y) + ' ' + str(xyz.z)
        #ang = str(rpy[0]) + ' ' + str(rpy[1]) + ' ' + str(rpy[2])
        #vels = str(X[0][0]) + ' ' + str(X[1][0]) + ' ' + str(X[2][0])
        #traj = str(v[0]) + ' ' + str(v[1]) + ' ' + str(v[2])
        file_obj.write(pos + ' ' + str(t) + '\n \n')


if __name__ == '__main__':
    rospy.init_node('drone_control_node')
    controller = None

    try:
        controller = Controller()

        time.sleep(1)
        controller.start()
        time.sleep(3)

        start = rospy.get_time()
        previous = rospy.get_time()
        controller.start_time = time.time()
        controller.t0 = controller.start_time
        while not rospy.is_shutdown():
            time.sleep(0.02)

            t = rospy.get_time() - start

            controller.update(t - previous, t)
            previous = t

            #if (t > 10):
            #    break
    finally:
        time.sleep(1)
        controller.stop()
        del controller
