#!/usr/bin/env python
import roslib
import rospy
import csv
from os.path import dirname, abspath, join
from std_msgs.msg import Float32

sum = 0
counter = 0
data = []
time = []
time_init = 0

def handle_speed(msg):
    global sum, counter, data, time, time_init
    sum += msg.data
    counter += 1

    if counter == 35:
        print(sum/counter)
        data.append(sum/counter)
        time.append(rospy.Time.now().to_sec() - time_init)
        sum = 0
        counter = 0

while not rospy.is_shutdown():
    rospy.init_node("calibration")
    rospy.Subscriber("/speed", Float32, handle_speed)
    time_init = rospy.Time.now().to_sec()
    rospy.spin()
    file_name = 'speed_data.csv'
    with open(join(dirname(abspath(__file__)), file_name), 'w+') as file:
        csv_writer = csv.writer(file, delimiter=',')
        csv_writer.writerow(['speed', 'time'])
        for (speed, t) in zip(data, time):
            csv_writer.writerow([speed, t])
