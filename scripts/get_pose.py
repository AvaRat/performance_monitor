#!/usr/bin/env python
from os import path
import rosbag
import tf
import csv
import rospy
import tf2_ros
from pickle import dump

dir_path = '/home/marcel/ros_ws/analytics_ws/src/performance_monitor/data'
car_path_file = 'pose_data.csv'

x = []
y = []
t = []

def parse_2_file(file):
    csv_writer = csv.writer(file, delimiter=',')
    csv_writer.writerow(['t', 'x_pos', 'y_pos'])
    for x_,y_,t_ in zip(x,y,t):
        csv_writer.writerow([t_, x_, y_])
    print('parsed to file')

def tf_callback(trans):
    x.append(trans.transform.translation.x)
    y.append(trans.transform.translation.y)
    t.append(trans.header.stamp.to_sec())

if __name__ == '__main__':
    rospy.init_node('get_pose')

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform('map',
                                               'base_link',
                                               rospy.Time(0))
            tf_callback(trans)       
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass#rospy.logwarn('Transform lookup failed') 
    with open(path.join(dir_path, car_path_file), 'w') as f:
        parse_2_file(f)