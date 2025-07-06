#!/usr/bin/env python
# -*- coding: utf-8 -*-

import csv
import math
import os
import sys
from threading import Lock

import rospy
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion


def get_theta(amcl):
    q = (amcl.pose.pose.orientation.x,
         amcl.pose.pose.orientation.y,
         amcl.pose.pose.orientation.z,
         amcl.pose.pose.orientation.w)
    euler = euler_from_quaternion(q)
    return euler[2]


def get_pose_from_amcl_msg(msg):
    pose = Pose2D()
    pose.x = msg.pose.pose.position.x
    pose.y = msg.pose.pose.position.y
    pose.theta = get_theta(msg)
    return pose


class SensorMessageGetter(object):
    def __init__(self, topic, msg_type, msg_converter):
        self.msg_converter = msg_converter
        self.lock = Lock()
        self.msg = None
        self.sub = rospy.Subscriber(topic, msg_type, self.on_msg)

    def on_msg(self, msg):
        self.lock.acquire()
        self.msg = msg
        self.lock.release()

    def get(self):
        val = None
        self.lock.acquire()
        if self.msg is not None:
            val = self.msg_converter(self.msg)
        self.lock.release()
        return val


def get_pose(time_limit, pose='/amcl_pose'):
    func = sys._getframe().f_code.co_name
    rospy.loginfo('Executing ' + func)
    time = 0
    start_time = rospy.get_time()
    sensor_msg = SensorMessageGetter(
        pose, PoseWithCovarianceStamped, get_pose_from_amcl_msg)
    rate = rospy.Rate(10)
    pose_pre = None
    thresh = 0.2
    id = 1
    while pose_pre is None:
        pose_pre = sensor_msg.get()

    csvfile = open('path.csv', 'w', newline='', encoding='utf-8')
    csv_writer = csv.writer(csvfile)

    while time < time_limit and not rospy.is_shutdown():
        pose = sensor_msg.get()
        if pose is not None:
            dx = pose.x - pose_pre.x
            dy = pose.y - pose_pre.y
            d = math.hypot(dx, dy)
            if d > thresh:
                csv_writer.writerow([id, pose.x, pose.y, thresh])
                id = id + 1
                rospy.loginfo('(x, y, theta) = (%.2f,%.2f,%.2f)' %
                              (pose.x, pose.y, math.degrees(pose.theta)))
                pose_pre = pose

        time = rospy.get_time() - start_time
        rate.sleep()
    csvfile.close()


def main():
    script_name = os.path.basename(__file__)
    node_name = os.path.splitext(script_name)[0]
    rospy.init_node(node_name)
    rospy.sleep(1)
    get_pose(9999)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
