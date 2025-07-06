#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import os

import actionlib
import rospy
from geometry_msgs.msg import Quaternion
from japanese_text_to_speech.msg import SpeakAction, SpeakGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


def text_to_speech(ac, text, speed_rate=1.0, time_limit=20):
    goal = SpeakGoal()
    goal.text = text
    goal.speed_rate = speed_rate
    ac.send_goal(goal)
    ac.wait_for_result(rospy.Duration(time_limit))


def goto_point(ac, x, y, theta):
    # ナビゲーションする関数
    # 第1引数：生成済みAction Client
    # 第2,3引数：目標の座標（単位：メートル）
    # 第4引数：ゴール到着時の姿勢（単位：ラジアン）
    # ゴールの生成
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()  # 現在時刻

    goal.target_pose.pose.position.x = x  # ゴールのx座標
    goal.target_pose.pose.position.y = y  # ゴールのy座標
    q = quaternion_from_euler(0, 0, theta)  # ゴール到着時に向かせたい方向
    goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    rospy.loginfo('Sending goal')
    ac.send_goal(goal)  # ゴールを送信する。
    finished = ac.wait_for_result(rospy.Duration(30))  # 何等かの結果が出るまで待つ（30秒以内）
    state = ac.get_state()
    if finished:
        rospy.loginfo('Finished: (%d)', state)
    else:
        rospy.loginfo('Time out: (%d)', state)


def main():
    script_name = os.path.basename(__file__)
    node_name = os.path.splitext(script_name)[0]
    rospy.init_node(node_name)
    rospy.sleep(1)
    # 自律移動用 Action Client
    ac_nav = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    while not ac_nav.wait_for_server(rospy.Duration(5)):
        rospy.loginfo('Waiting for the move_base action server to come up')
    rospy.loginfo('The move_base server comes up')

    # 音声合成用 Action Client
    ac_tts = actionlib.SimpleActionClient(
        'japanese_text_to_speech', SpeakAction)
    while not ac_tts.wait_for_server(rospy.Duration(5)):
        rospy.loginfo(
            'Waiting for the japanese_text_to_speech action server to come up')
    rospy.loginfo('The japanese_text_to_speech server comes up')

    # 目標地点は地図に応じて適宜変更すること
    text_to_speech(ac_tts, '出発します！')
    # 座標（7.10, 2.1）に90度の向きで止まるようにナビゲーションする
    goto_point(ac_nav, 7.10, 2.1, math.radians(90))
    text_to_speech(ac_tts, '帰ります！')
    goto_point(ac_nav, 1.33, 0.0, math.radians(0))
    text_to_speech(ac_tts, '終わります！')


if __name__ == '__main__':
    main()
