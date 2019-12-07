#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
from os.path import dirname, abspath, join
from scipy import interpolate
import numpy as np
import pandas as pd


class Converter:
    def __init__(self, angle_file_path, speed_file_path, angle_publisher, speed_publisher):
        with open(angle_file_path, 'r') as f:
            angle_lookup_table = pd.read_csv(f)
            
        with open(speed_file_path, 'r') as f:
            speed_lookup_table = pd.read_csv(f)
         
        self.f_angle = interpolate.interp1d(angle_lookup_table['delta_measured'], angle_lookup_table['steering_angle'], fill_value='extrapolate', assume_sorted='false')
        self.angle_pub = angle_publisher

        self.speed_pub = speed_publisher
        self.f_speed = interpolate.interp1d(speed_lookup_table['delta'], speed_lookup_table['speed_measured'], fill_value='extrapolate')

        self.current_steering_angle = 0


    def target_drive_callback(self, msg):
        self.current_steering_angle = msg.drive.steering_angle
        angle_converted = self.f_angle(msg.drive.steering_angle)
        msg.drive.steering_angle = angle_converted
        self.angle_pub.publish(msg)
    
    def shaft_speed_callback(self, msg):
        speed_converted = msg.data * self.f_speed(np.abs(self.current_steering_angle))
        msg.data = speed_converted
        self.speed_pub.publish(msg)



## main
if __name__ == '__main__':
    rospy.init_node('delta_compensator', anonymous=True)

    angle_lookup_table = rospy.get_param('~angle_lookup_table', 1)
    speed_lookup_table = rospy.get_param('~speed_lookup_table', 1)

    angle_lookup_table = join(join(dirname(dirname(abspath(__file__))),'data'), angle_lookup_table)
    speed_lookup_table = join(join(dirname(dirname(abspath(__file__))),'data'), speed_lookup_table)

    rospy.loginfo(angle_lookup_table)
    rospy.loginfo(speed_lookup_table)

    cmd_pub = rospy.Publisher('/raw_drive', AckermannDriveStamped, queue_size=1)
    speed_pub = rospy.Publisher('/speed', Float32, queue_size=1)

    converter = Converter(angle_lookup_table, speed_lookup_table, cmd_pub, speed_pub)

    cmd_sub = rospy.Subscriber('/target_drive', AckermannDriveStamped, converter.target_drive_callback)
    speed_sub = rospy.Subscriber('/shaft_speed', Float32, converter.shaft_speed_callback)
    
    rospy.spin()
