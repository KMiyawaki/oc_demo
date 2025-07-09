#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion


def get_yaw_from_quaternion(orientation):
    _, _, yaw = euler_from_quaternion(orientation)
    return yaw


def get_current_pose(listener, time_limit=10.0, target='map', source='base_link'):
    try:
        listener.waitForTransform(
            target, source, rospy.Time(), rospy.Duration(time_limit))
        (trans, rot) = listener.lookupTransform(target, source, rospy.Time(0))
        yaw = get_yaw_from_quaternion(rot)
        return (trans[0], trans[1], yaw)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        raise e


class SensorMessageGetter(object):
    def __init__(self, topic, msg_type, msg_wait=1.0):
        self.msg_wait = msg_wait
        self.topic = topic
        self.msg_type = msg_type

    def get_msg(self):
        message = None
        try:
            message = rospy.wait_for_message(
                self.topic, self.msg_type, self.msg_wait)
        except rospy.exceptions.ROSException as e:
            rospy.logdebug(e)
        return message


def main():
    rospy.init_node('get_robot_pose')
    mode = 'amcl'
    mode = rospy.get_param('mode', mode)
    x = 0
    y = 0
    yaw = 0
    if mode == 'tf':
        listener = tf.TransformListener()
        x, y, yaw = get_current_pose(listener)
    else:
        amcl_pose = SensorMessageGetter('amcl_pose', PoseWithCovarianceStamped)
        while rospy.is_shutdown() == False:
            msg = amcl_pose.get_msg()
            if msg is not None:
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                q = (msg.pose.pose.orientation.x,
                     msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z,
                     msg.pose.pose.orientation.w)
                euler = euler_from_quaternion(q)
                yaw = euler[2]
                break
    rospy.loginfo('OK. got current pose (x, y, yaw)')
    rospy.loginfo('%f, %f, %f (rad)' % (x, y, yaw))


if __name__ == '__main__':
    main()
