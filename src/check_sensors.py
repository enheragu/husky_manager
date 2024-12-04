#!/usr/bin/env python3
# encoding: utf-8
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
import time

from tabulate import tabulate

# Define a callback function for each topic 
class check():
    def __init__(self):
        self.sub1=rospy.Subscriber("/ouster/points", PointCloud2, self.topic1_callback)
        self.sub2=rospy.Subscriber("/gnss/fix", NavSatFix, self.topic2_callback)
        self.sub3=rospy.Subscriber("/imu/data", Imu, self.topic3_callback)
        self.message_counts = {
            '/ouster/points': 0,
            '/gnss/fix': 0,
            '/imu/data': 0
        }
        self.start_time_lidar = time.time()
        self.start_time_gps = time.time()
        self.start_time_imu = time.time()

        self.topic_info = {'F LIDAR (Hz)': ["  None  "], 
                           'F GPS (Hz)': ["  None  "], 
                           'Mode GPS': ["  None  "], 
                           'COV GPS': ["  (None,None)  "], 
                           'F IMU (Hz)': ["  None  "]}

    def topic1_callback(self,msg):
        self.message_counts['/ouster/points']+=1
        if time.time()-self.start_time_lidar>=1:
            self.topic_info['F LIDAR (Hz)'] = [self.message_counts['/ouster/points']]
            self.start_time_lidar = time.time()
            self.message_counts['/ouster/points'] = 0

    def topic2_callback(self,msg):
        self.message_counts['/gnss/fix']+=1
        if time.time()-self.start_time_gps>=1:
            self.topic_info['F GPS (Hz)'] = [self.message_counts['/gnss/fix']]
            self.topic_info['Mode GPS'] = [msg.status.status]
            self.topic_info['COV GPS'] = [(msg.position_covariance[0], msg.position_covariance[4])]
            self.start_time_gps = time.time()
            self.message_counts['/gnss/fix'] = 0


    def topic3_callback(self,msg):
        self.message_counts['/imu/data']+=1
        if time.time()-self.start_time_imu>=1:
            self.topic_info['F IMU (Hz)'] = [self.message_counts['/imu/data']]
            self.start_time_imu = time.time()
            self.message_counts[ '/imu/data'] = 0

    def print_titles(self):
        formated_table = tabulate(self.topic_info, headers="keys", numalign=("center"))
        lines = formated_table.splitlines()
        print("\n")
        print(f"\033[96m{'-'*len(lines[1])}\033[0m")
        print(f"\033[96m {lines[0]}\033[0m")
        print(f"\033[96m{'-'*len(lines[1])}\033[0m")
        # print(lines[2], end="", flush=True)

    def update_print(self):
        formated_table = tabulate(self.topic_info, headers="keys", numalign=("center"))
        endl = f"{formated_table.splitlines()[-1]}"
        print("\r"+" "*60, end="", flush=True)
        print(f"\033[96m\r{endl}\033[0m", end="", flush=True)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('frequency_checker')
    check_obj = check()

    rate = rospy.Rate(1) # ROS Rate at 5Hz

    check_obj.print_titles()
    while not rospy.is_shutdown():
        check_obj.update_print()
        rate.sleep()

