#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import threading

import actionlib
import actionlib.msg
import angles
import rospy
from geometry_msgs.msg import (Pose2D, PoseWithCovarianceStamped, Quaternion,
                               Twist)
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from waypoint_reader import read_waypoints


class WayPointNavigation(object):
    def __init__(self):
        csv_path = rospy.get_param("~csv_path", "")
        self.waypoints = read_waypoints(csv_path, 2.0)
        self.start_index = rospy.get_param("~start_index", 0)
        self.crnt_index = -1
        self.target_angle = 0
        # start_id が指定された場合はそれを優先する。
        start_id = str(rospy.get_param("~start_id", ''))
        if start_id:
            target_index = get_index_from_id(self.waypoints, start_id)
            if target_index is not None:
                self.start_index = int(target_index)
                rospy.logwarn('start_id is specified %s' % start_id)

        process_rate = rospy.get_param('~process_rate', 20.0)
        self.manual = rospy.get_param('~manual', True)
        self.linear_max = rospy.get_param('~linear_max', 0.4)
        self.angular_max = rospy.get_param('~angular_max', math.radians(60))
        self.linear_gain = rospy.get_param('~linear_gain', 1.0)
        self.angular_gain = rospy.get_param('~angular_gain', 0.8)
        self.pose_filter = rospy.get_param('~pose_filter', 10)
        self.loop = rospy.get_param('~loop', True)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(process_rate)
        self.amcl_lock = threading.Lock()
        self.amcl_pose = Pose2D()
        self.amcl_pose_queue = []
        self.connect_move_base()
        self.subscribe_amcl()

    def spin(self):
        if self.crnt_index < 0:
            # start_indexが-1のとき、closest_point(一番近いwaypointのindex)
            if self.start_index == -1:
                x, y, _ = self.get_amcl_pose()
                self.start_index = closest_waypoint(x, y, self.waypoints)
            self.crnt_index = self.start_index
            rospy.loginfo('Set crnt index to %d' % (self.crnt_index))
            return
        if self.crnt_index >= len(self.waypoints):
            if self.loop:
                rospy.loginfo('Finish track, goto start.')
                self.crnt_index = 0
            else:
                return False
        _, x, y, thresh, _ = self.waypoints[self.crnt_index]
        if self.crnt_index < len(self.waypoints) - 1:
            _, x2, y2, _, _ = self.waypoints[self.crnt_index + 1]
            self.target_angle = math.atan2(y2 - y, x2 - x)
        rospy.loginfo('goto [%d] %f, %f, %f' % (
            self.crnt_index, x, y, math.degrees(self.target_angle)))
        if self.manual:
            self.move_to_goal_manual(x, y, self.target_angle, thresh)
        else:
            self.move_to_goal(x, y, self.target_angle, thresh)
        self.crnt_index = self.crnt_index + 1
        return True

    def connect_move_base(self):
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        while not self.ac.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Waiting for the move_base action server to come up")
        rospy.loginfo("The server comes up")

    def amcl_to_pose2d(self, msg):
        pose = Pose2D()
        pose.x = msg.pose.pose.position.x
        pose.y = msg.pose.pose.position.y
        q = (msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(q)
        pose.theta = euler[2]
        return pose

    def smoothing(self):
        self.amcl_pose = Pose2D()
        for p in self.amcl_pose_queue:
            self.amcl_pose.x += p.x
            self.amcl_pose.y += p.y
            self.amcl_pose.theta += p.theta
        size = len(self.amcl_pose_queue)
        self.amcl_pose.x = self.amcl_pose.x / size
        self.amcl_pose.y = self.amcl_pose.y / size
        self.amcl_pose.theta = self.amcl_pose.theta / size

    def cb_amcl(self, msg):
        self.amcl_lock.acquire()  # Lock
        self.amcl_pose_queue.append(self.amcl_to_pose2d(msg))
        while len(self.amcl_pose_queue) > self.pose_filter:
            self.amcl_pose_queue.pop(0)
        self.smoothing()
        self.amcl_lock.release()  # Release

    def get_amcl_pose(self):
        # より高速に現在の姿勢を得る
        copy = Pose2D()
        self.amcl_lock.acquire()  # Lock
        copy.x = self.amcl_pose.x
        copy.y = self.amcl_pose.y
        copy.theta = self.amcl_pose.theta
        self.amcl_lock.release()  # Release
        return copy.x, copy.y, copy.theta

    def subscribe_amcl(self):
        self.sub_amcl = rospy.Subscriber(
            'amcl_pose', PoseWithCovarianceStamped, self.cb_amcl)

    def limit_vel(self, v, abs_v):
        return max(min(v, abs_v), -abs_v)

    def send_velocity(self, linear, angular, times=1, interval=0.05):
        twist = Twist()
        twist.linear.x = self.limit_vel(linear, self.linear_max)
        twist.angular.z = self.limit_vel(angular, self.angular_max)
        for i in range(times):
            self.pub_cmd_vel.publish(twist)
            rospy.Duration(interval).sleep()

    def move_to_goal_manual(self, x, y, theta, thresh, time_limit=30):
        start_time = rospy.get_time()
        try:
            while rospy.get_time() - start_time < time_limit:
                nx, ny, na = self.get_amcl_pose()
                dx = x - nx
                dy = y - ny
                d = math.hypot(dx, dy)
                if thresh > d:
                    # rospy.loginfo('%f, %f, %f' % (dx, dy, d))
                    return
                theta = angles.normalize_angle(math.atan2(dy, dx) - na)
                ratio = 1.0 - min(1.0, abs(theta) / self.angular_max)
                # if abs(theta) > math.radians(90):
                #    return
                twist = Twist()
                twist.linear.x = self.limit_vel(
                    d * self.linear_gain * ratio, self.linear_max)
                twist.angular.z = self.limit_vel(
                    theta * self.angular_gain, self.angular_max)
                self.pub_cmd_vel.publish(twist)
                self.rate.sleep()
        except Exception as e:
            rospy.logerr(Ellipsis)

    def move_to_goal(self, x, y, theta, thresh, time_limit=30):
        coord_type = "map"
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = coord_type
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = Quaternion(
            *quaternion_from_euler(0, 0, theta))

        rospy.loginfo("Sending goal")
        self.ac.send_goal(goal)
        # 5秒前に送信したゴールを全てキャンセルする
        try:
            rospy.logwarn("Cancel old goals")
            self.ac.cancel_goals_at_and_before_time(
                rospy.Time.now() - rospy.Duration(5))
        except Exception as e:
            rospy.logerr(str(e))

        start_time = rospy.get_time()
        while rospy.get_time() - start_time < time_limit:
            nx, ny, _ = self.get_amcl_pose()
            dx = x - nx
            dy = y - ny
            d = math.hypot(dx, dy)
            if thresh > d:
                return
            state = self.ac.get_state()
            if state is actionlib.GoalStatus.SUCCEEDED:
                rospy.logwarn('!!!Navigation SUCCEEDED')
                return
            if state is actionlib.GoalStatus.ABORTED:
                rospy.logwarn('!!!Navigation ABORTED')
                return
            self.rate.sleep()


def get_yaw_from_quaternion(orientation):
    _, _, yaw = euler_from_quaternion(orientation)
    return yaw


def get_yaw(odom):
    q = (odom.pose.pose.orientation.x,
         odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z,
         odom.pose.pose.orientation.w)
    euler = euler_from_quaternion(q)
    return euler[2]


def closest_waypoint(x, y, waypoints):
    min_index = None
    min_distance = float('inf')
    for i, waypoint in enumerate(waypoints):
        # 最初の2つの要素のみを使用
        way_x, way_y = waypoint[1], waypoint[2]
        dx = x - way_x
        dy = y - way_y
        distance = math.hypot(dx, dy)
        if min_index is None or distance < min_distance:
            min_distance = distance
            min_index = i
    return min_index


def get_index_from_id(waypoints, id):
    for i, w in enumerate(waypoints):
        s1 = w[0].strip()
        id = id.strip()
        if s1 == id:
            return i
    return None


def main():
    rospy.init_node('waypoint_navigation')
    rospy.sleep(0.5)
    nav = WayPointNavigation()
    rospy.sleep(1)
    rospy.loginfo('Start Waypoint Navigation')

    try:
        while not rospy.is_shutdown():
            if nav.spin() == False:
                break
        nav.send_velocity(0, 0, 5)
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(e)


if __name__ == '__main__':
    main()
