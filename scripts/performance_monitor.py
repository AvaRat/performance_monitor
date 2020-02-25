#!/usr/bin/env python
from os.path import dirname, abspath, join
import csv
import pickle
import math
import rospy
import tf
import tf2_ros
from std_msgs.msg import Float32
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu
import numpy as np


class Monitor:
    def __init__(self, end_distance, data_file):
        self.end_distance = end_distance    #currently not used
        self.data_file = join(join(dirname(dirname(abspath(__file__))),'data'), data_file)  # file where data will be saved  csv format
        self.path_file = join(join(dirname(dirname(abspath(__file__))),'data'), 'path.pkl') # file where path will be saved in .pkl format
        
        self.last_track_progress = 0
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

        self.path = []

        self.last_closest_path = []
        self.init_time = 0

        self.last_lin_acc = {'x':0, 'y':0, 'z':0}
        self.lin_acc = []

        
    def get_deviation(self, p):
        pl1 = self.last_closest_path[0]
        pl2 = self.last_closest_path[1]
        x_diff = pl2['x'] - pl1['x']
        y_diff = pl2['y'] - pl1['y']
        num = math.fabs(y_diff*p['x'] - x_diff*p['y'] + pl2['x']*pl1['y'] - pl2['y']*pl1['x'])
        den = math.sqrt(y_diff**2 + x_diff**2)
        return num / den

            
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

    
    def speed_callback(self, msg):
        if(self.init_time != 0):
            self.last_speed = msg.data

    def path_callback(self, path_msg):
        self.path = []
        for point in path_msg.poses:
            self.path.append({'x':point.pose.position.x, 'y':point.pose.position.y})

    def closest_points_callback(self, path_msg):
        for point in path_msg.poses:
            self.last_closest_path.append({'x':point.pose.position.x, 'y':point.pose.position.y})

    def tf_callback(self, trans):
        #first callback
        if(self.init_time == 0):
            self.init_time = rospy.Time.now()
        else:
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            t = (rospy.Time.now()-self.init_time).to_sec()
            self.position.append({'x':x, 'y':y})
            self.time.append(t)
            self.speed.append(self.last_speed)
            deviation = self.get_deviation({'x':x, 'y':y})
            self.deviation.append(deviation)
            self.last_deviation = deviation

            self.lin_acc.append(self.last_lin_acc)
            # track_progress = ?
            #self.track_progress.append(self.get_track_progress())

    def imu_callback(self, imu_msg):
        self.last_lin_acc = {'x':imu_msg.linear_acceleration.x, 'y':imu_msg.linear_acceleration.y, 'z':imu_msg.linear_acceleration.z}
    


    def print_summary(self):
        with open(self.path_file, 'wb') as f:
            pickle.dump(self.path, f)

        print('writing to data file '+repr(self.data_file)+' ...')
        with open(self.data_file, 'w+') as file:
            csv_writer = csv.writer(file, delimiter=',')
            csv_writer.writerow(['t', 'speed', 'x_pos', 'y_pos', 'deviation', 'x_lin_acc', 'y_lin_acc', 'z_lin_acc'])
            n_el = len(self.position)
            for i in range(n_el):
                csv_writer.writerow([self.time[i], self.speed[i], self.position[i]['x'], self.position[i]['y'], self.deviation[i], self.lin_acc[i]['x'], self.lin_acc[i]['y'], self.lin_acc[i]['z']])
        try:
            print('time: '+ repr(self.time[-1])) # +repr(self.track_progress[-1]) 
           # print('max deviation: ' + repr(np.max(self.deviation)) + '\ndeviation sum: ' + repr(np.sum(self.deviation)) + '\ndeviation std: '+ repr(np.std(self.deviation)))
        except:
            pass

if __name__ == '__main__':
    rospy.init_node('performance_monitor')


    # Skidpad parameters
    end_distance = rospy.get_param('~end_distance', 1.0)
    BASE_FRAME = rospy.get_param('~base_frame', 'map')
    TARGET_FRAME = rospy.get_param('~target_frame', 'base_link')

    data_file = rospy.get_param('~file_name', 'performance_data.csv')
    monitor = Monitor(end_distance, data_file)


  #  rospy.Subscriber('/track_progress', Float32, monitor.s_callback)
  #  rospy.Subscriber('/deviation', Float32, monitor.d_callback)
    rospy.Subscriber('/speed', Float32, monitor.speed_callback)
    rospy.Subscriber('/path', Path, monitor.path_callback)
    rospy.Subscriber('/closest_path_points', Path, monitor.closest_points_callback)
    rospy.Subscriber('/imu', Imu, monitor.imu_callback)

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform(BASE_FRAME,
                                               TARGET_FRAME,
                                               rospy.Time(0))
            monitor.tf_callback(trans)       
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass#rospy.logwarn('Transform lookup failed') 

    monitor.print_summary()