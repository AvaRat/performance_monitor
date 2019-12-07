#!/usr/bin/env python
from os.path import dirname, abspath, join
import csv

import rospy
import tf
import tf2_ros
from std_msgs.msg import Float64
import numpy as np


class Monitor:
    def __init__(self, s_begin, s_end, data_file):
        self.s_begin = s_begin
        self.s_end = s_end
        self.data_file = join(join(dirname(dirname(abspath(__file__))),'data'), data_file)
        self.track_progress = []
        self.deviation = []
        self.last_deviation = 0
        self.time = []
        self.begin_time = rospy.Time.now()
        self.speed = []
        self.last_speed = 0
        self.position = []
        self.last_position = {'x':0, 'y':0}
        self.steering_angle = []
        self.init_time = 0

        

            
    def s_callback(self, msg):
        if(msg.data < s_begin):
            pass
        elif(msg.data <= self.s_end):
            if self.init_time == 0:
                self.init_time = rospy.Time.now()
            self.track_progress.append(msg.data)
            t = (rospy.Time.now()-self.init_time).to_sec()
            self.time.append(t)
            self.deviation.append(self.last_deviation)
            self.speed.append(self.last_speed)
            self.position.append(self.last_position)


    def d_callback(self, msg):
        if(msg.data < s_begin):
            pass
        else:
            self.last_deviation = msg.data
    
    def speed_callback(self, msg):
        if(msg.data < self.s_begin):
            pass
        else:
            self.last_speed = msg.data

    def tf_callback(self, trans):
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        self.last_position = {'x':x, 'y':y}
    
    def print_summary(self):
        print('writing to data file....')
        with open(self.data_file, 'w+') as file:
            csv_writer = csv.writer(file, delimiter=',')
            csv_writer.writerow(['t', 'speed', 'x_pos', 'y_pos', 'track_progress', 'deviation'])
            min_el = np.min([len(self.time), len(self.speed), len(self.position)])

            for i in range(min_el):
                csv_writer.writerow([self.time[i], self.speed[i], self.position[i]['x'], self.position[i]['y'], self.track_progress[i], self.deviation[i]])
        print('distance covered: ' +repr(self.track_progress[-1]) + '\ntime: '+ repr(self.time[-1]))
        print('max deviation: ' + repr(np.max(self.deviation)) + '\ndeviation sum: ' + repr(np.sum(self.deviation)) + '\ndeviation std: '+ repr(np.std(self.deviation)))

if __name__ == '__main__':
    rospy.init_node('performance_monitor')


    # Skidpad parameters
    s_begin = rospy.get_param('~s_begin', 0.0)
    s_end = rospy.get_param('~s_end', 5.0)
    data_file = rospy.get_param('~file_name', 'data.csv')
    monitor = Monitor(s_begin, s_end, data_file)

    rospy.Subscriber('/track_progress', Float64, monitor.s_callback)
    rospy.Subscriber('/deviation', Float64, monitor.d_callback)
    rospy.Subscriber('/speed', Float64, monitor.speed_callback)

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform('skidpad',
                                               'base_link',
                                               rospy.Time(0))
            monitor.tf_callback(trans)       
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass#rospy.logwarn('Transform lookup failed') 

    monitor.print_summary()