#!/usr/bin/env python
from os import path
import rosbag
import tf
import tf2_ros
from pickle import dump

dir_path = '/home/marcel/s6_12_grudnia/rosbags'
file_path = 'corn_sfty_200_speed_3.bag'

speed = []
distance = []
path_ = []
time = []
car_positions = []
data = {'time':time,'speed':speed, 'distance':distance, 'path':path_, 'car_path':car_positions}
n=0


bag = rosbag.Bag(path.join(dir_path, file_path))
for topic, msg, t in bag.read_messages():
    if(topic=='speed'):
        speed.append(msg.data)
        time.append(t.to_sec())
    if(topic=='tf'):
        for tf_msg in msg.transforms:
            if(tf_msg.child_frame_id == 'odom' and tf_msg.header.frame_id =='map'):
                car_positions.append({'x':tf_msg.transform.translation.x, 'y':tf_msg.transform.translation.x})
            #x = trans.transform.translation.x
            #y = trans.transform.translation.y
            #position.append({'x':x, 'y':y})

for topic, msg, t in bag.read_messages(topics='path'):
    for point in msg.poses:
        path_.append({'x':point.pose.position.x, 'y':point.pose.position.y})
    

with open('/home/marcel/ros_ws/analytics_ws/src/performance_monitor/data/from_bag.pkl', 'wb') as f:
    dump(data, f)
bag.close()