#!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
import csv
import math
import sys

import rospy


def read_waypoints(csv_path, default_thresh=3.0):
    waypoints = []
    line_num = 0
    with open(csv_path) as f:
        reader = csv.reader(f)
        for row in reader:
            line_num = line_num + 1
            new_row = []
            if row[0].startswith('#'):
                continue
            # ID, x, y, thresh, command
            cell_size = len(row)
            if cell_size < 3:
                rospy.logerr('Bad csv format. file:%s line:%d' %
                             (csv_path, line_num))
                continue
            new_row.append(row[0])
            new_row.append(float(row[1]))
            new_row.append(float(row[2]))
            thresh = default_thresh  # default threshold
            if cell_size > 3 and len(row[3]) > 0:
                thresh = float(row[3])
            new_row.append(thresh)
            command = ''  # empty command
            if cell_size > 4 and len(row[4]) > 0:
                command = row[4]
            new_row.append(command)
            # check element size
            if len(new_row) < 5:
                rospy.logerror('read_waypoints: Bad element size. file:%s line:%d' %
                               (csv_path, line_num))
                sys.exit(1)
            waypoints.append(new_row)
    return waypoints


def get_waypoint_index_from_id(waypoints, id):
    for i, w in enumerate(waypoints):
        s1 = w[0].strip()
        id = id.strip()
        if s1 == id:
            return i
    return None


def calc_distance(waypoints, start_id='', end_id=''):
    s_idx = 0
    if start_id:
        s_idx = get_waypoint_index_from_id(waypoints, start_id)
        if s_idx == None:
            s_idx = 0
    e_idx = len(waypoints) - 1
    if end_id:
        e_idx = get_waypoint_index_from_id(waypoints, end_id)
        if e_idx == None:
            e_idx = len(waypoints) - 1
    if e_idx < s_idx:
        s_idx, e_idx = e_idx, s_idx

    sum = 0
    pre = None
    try:
        for i in range(s_idx, e_idx + 1):
            w = waypoints[i]
            if pre == None:
                pre = w
            elif len(w) >= 3:
                dx = w[1] - pre[1]
                dy = w[2] - pre[2]
                sum += math.hypot(dx, dy)
                pre = w
    except Exception as e:
        rospy.logerr(e)
        return -1
    return sum


def main():
    parser = argparse.ArgumentParser(description='waypoint reader')
    parser.add_argument('-c', '--csv', help='CSVファイルのパス', default='')
    parser.add_argument(
        '-s', '--start', help='距離計算始点ウェイポイントID', default='')
    parser.add_argument(
        '-e', '--end', help='距離計算終点ウェイポイントID', default='')
    args = parser.parse_args()
    waypoints = read_waypoints(args.csv)
    distance = calc_distance(waypoints, args.start, args.end)
    print(args.csv)
    print('distance from', end=' ')
    if args.start:
        print(args.start, end=' ')
    else:
        print('start', end=' ')
    print('to', end=' ')
    if args.end:
        print(args.end, end=' is ')
    else:
        print('end', end=' is ')
    print(distance, '(m)')


if __name__ == '__main__':
    main()
