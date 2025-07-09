#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler

seq = 0


def exec_first_run(rate, time_limit, linear_vel=0.4, cmd_vel="/cmd_vel", zero_velocity_pub_times=5):
    rospy.loginfo('Executing exec_first_run %f(m/sec) * %f(sec)' %
                  (linear_vel, time_limit))
    pub = rospy.Publisher(cmd_vel, Twist, queue_size=10)
    vel = Twist()
    vel.linear.x = linear_vel
    diff = 0
    start_time = rospy.get_time()
    while diff < time_limit:
        pub.publish(vel)
        rate.sleep()
        diff = rospy.get_time() - start_time
    vel.linear.x = 0.0
    for i in range(zero_velocity_pub_times):
        pub.publish(vel)
        rate.sleep()


def build_initial_pose_msg(x, y, yaw, x_sigma=0.5, y_sigma=0.5, yaw_sigma=math.radians(15)):
    global seq
    seq = seq + 1
    msg = PoseWithCovarianceStamped()
    msg.header.seq = seq
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0
    q = quaternion_from_euler(0, 0, yaw)
    msg.pose.pose.orientation.x = q[0]
    msg.pose.pose.orientation.y = q[1]
    msg.pose.pose.orientation.z = q[2]
    msg.pose.pose.orientation.w = q[3]
    msg.pose.covariance = [x_sigma * x_sigma, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, y_sigma * y_sigma, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, yaw_sigma * yaw_sigma]
    return msg


def main():
    rospy.init_node('amcl_estimate')
    rospy.sleep(rospy.Duration(1))
    x = rospy.get_param('~x', 0)
    y = rospy.get_param('~y', 0)
    yaw = rospy.get_param('~yaw', 0)
    x_sigma = rospy.get_param('~x_sigma', 0.5)
    y_sigma = rospy.get_param('~y_sigma', 0.5)
    yaw_sigma = rospy.get_param('~yaw_sigma', math.radians(60))
    hz = rospy.get_param('~hz', 10)
    publish_time = rospy.get_param('~publish_time', 2)
    update_time = rospy.get_param('~update_time', 3)
    first_run = rospy.get_param('~first_run', 0.0)
    first_run_vel = rospy.get_param('~first_run_vel', 0.4)
    pub_initialpose = rospy.Publisher(
        'initialpose', PoseWithCovarianceStamped, queue_size=10)
    rate = rospy.Rate(hz)
    if first_run > 0:
        try:
            first_run_sec = first_run / first_run_vel
            exec_first_run(rate, first_run_sec, first_run_vel)
        except Exception as e:
            rospy.logerr(e)
    tm_start = rospy.Time.now()
    if publish_time > 0:
        while True:
            elapsed = (rospy.Time.now() - tm_start).to_sec()
            if elapsed > publish_time:
                break
            pub_initialpose.publish(build_initial_pose_msg(
                x, y, yaw, x_sigma, y_sigma, yaw_sigma))
            rate.sleep()
    rospy.loginfo(rospy.get_name() + ' publish finished')
    if update_time > 0:
        rospy.wait_for_service('request_nomotion_update')
        tm_start = rospy.Time.now()
        while True:
            elapsed = (rospy.Time.now() - tm_start).to_sec()
            if elapsed > update_time:
                break
            try:
                request_nomotion_update = rospy.ServiceProxy(
                    'request_nomotion_update', Empty)
                request_nomotion_update()
            except Exception as e:
                rospy.logerr(e)
            rate.sleep()
        rospy.loginfo(rospy.get_name() + ' update finished')


if __name__ == '__main__':
    main()
