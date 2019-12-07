#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from os.path import dirname, abspath, join
from scipy import interpolate
import pandas as pd


class Converter:
    def __init__(self, file_path, publisher):
        with open(file_path, 'r') as f:
            self.lookup_table = pd.read_csv(f)
        self.x = self.lookup_table['steering_angle']
        self.y = self.lookup_table['delta_measured']
        self.f = interpolate.interp1d(self.y, self.x, fill_value='extrapolate', assume_sorted='false')
        self.pub = publisher

    def target_drive_callback(self, msg):
        angle_converted = self.f(msg.drive.steering_angle)
        msg.drive.steering_angle = angle_converted
        self.pub.publish(msg)

## main
if __name__ == '__main__':
    rospy.init_node('delta_compensator', anonymous=True)

    lookup_table = rospy.get_param('~lookup_table', 1)
    lookup_table = join(dirname(dirname(abspath(__file__))), lookup_table)

    cmd_pub = rospy.Publisher('/raw_drive', AckermannDriveStamped, queue_size=1)
    converter = Converter(lookup_table, cmd_pub)
    cmd_sub = rospy.Subscriber('/target_drive', AckermannDriveStamped, converter.target_drive_callback)
    
    rospy.spin()
