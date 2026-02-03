#!/usr/bin/env python3
# encoding: utf-8
import shutil
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image, CompressedImage
from multiespectral_acquire.msg import ImageWithMetadata
import time

from tabulate import tabulate

class MonitorHz:
    def __init__(self, friendly_name, topic, message_type):
        self.friendly_name = friendly_name
        self.topic = topic
        self.message_type = message_type

        self.frequency = "N/A"
        self.history = []
        self.history_window = 5
        self.message_counts = 0

        self.start_time = rospy.Time.now().to_sec()

        self.sub = rospy.Subscriber(self.topic, self.message_type, self.callback)

        # Configure
        self.reset_timeout = 3.0 # Timeout in seconds to reset the message counts and metrics
        self.metric_update = 1.0 # Update the metrics every N seconds

    def callback(self, msg):
        self.message_counts += 1
        current_time = rospy.Time.now().to_sec()
        if current_time - self.start_time >= self.metric_update:
            self.history.append(self.message_counts)
            if len(self.history) > self.history_window:
                self.history.pop(0)  # Averages only last N frequencies (N = history_window)
            self.frequency = sum(self.history) / len(self.history)
            self.message_counts = 0
            self.start_time = current_time
    
    def getValue(self):
        # Handle timetout and reset messages if callback was not called
        current_time = rospy.Time.now().to_sec()
        if current_time - self.start_time >= self.reset_timeout:
            self.message_counts = 0
            self.frequency = "N/A"

        freq_msg = f"{self.frequency:.2f} Hz" if self.frequency != 'N/A' else self.frequency
        return {self.friendly_name: [freq_msg]}


class GPSMonitorHz(MonitorHz):
     
    def __init__(self, friendly_name, topic, message_type):
        super().__init__(friendly_name, topic, message_type)
        self.mode_gps = "N/A"
        self.cov_gps = f"[N/A,N/A]"

        self.gps_modes = {
                '-1': "No Fix",
                '0': "Fix",
                '1': "SBAS Fix",
                '2': "GBAS Fix"
            }
    def callback(self,msg):
        super().callback(msg)
        self.mode_gps = msg.status.status if str(msg.status.status) not in self.gps_modes.keys() else f"{self.gps_modes[str(msg.status.status)]} [{msg.status.status}]"
        self.cov_gps = [msg.position_covariance[0], msg.position_covariance[4]]

    def getValue(self):
        data = super().getValue()
        
        # Handle timetout and reset messages if callback was not called
        current_time = rospy.Time.now().to_sec()
        if current_time - self.start_time >= self.reset_timeout:
            self.mode_gps = "N/A"
            self.cov_gps = f"[N/A,N/A]"
        data.update({'Mode GPS': [self.mode_gps], 'COV GPS': [self.cov_gps]})
        return data
         
# Define a callback function for each topic 
class Monitor():
    def __init__(self):
        self.monitor_lidar = MonitorHz('LIDAR', "/ouster/points", PointCloud2)
        self.monitor_gps = GPSMonitorHz('GPS', "/gnss/fix", NavSatFix)
        self.monitor_imu = MonitorHz('IMU' ,"/imu/data", Imu)

        self.monitor_multiespectral_visible = MonitorHz('Mult. Visible', "/Multiespectral/visible_camera/image_with_metadata_sync", ImageWithMetadata)
        self.monitor_multiespectral_lwir = MonitorHz('Mult. Lwir', "/Multiespectral/lwir_camera/image_with_metadata_sync", ImageWithMetadata)
        self.monitor_fisheye_frontal = MonitorHz('Fish. Frontal', "/Fisheye/frontal_camera/image_with_metadata_sync", ImageWithMetadata)
        self.monitor_fisheye_rear = MonitorHz('Fish. Rear', "/Fisheye/rear_camera/image_with_metadata_sync", ImageWithMetadata)

        # Monitor store status
        self.recording_enabled = False
        self.control_sub = rospy.Subscriber('/Multiespectral/recording_enabled', Bool, self.recording_control_callback)

    def recording_control_callback(self, msg):
        self.recording_enabled = msg.data


    def updateTable(self):
        dict_data = {}
        dict_data.update(self.monitor_lidar.getValue())
        dict_data.update(self.monitor_gps.getValue())
        dict_data.update(self.monitor_imu.getValue())
        
        camera_data = {}
        camera_data.update(self.monitor_multiespectral_visible.getValue())
        camera_data.update(self.monitor_multiespectral_lwir.getValue())
        camera_data.update(self.monitor_fisheye_frontal.getValue())
        camera_data.update(self.monitor_fisheye_rear.getValue())

        # Añadir estado de grabación multiespectral
        if self.recording_enabled is not None:
            camera_data['Store file'] = [str(self.recording_enabled)]
        else:
            camera_data['Store file'] = ['N/A']


        formated_table = tabulate(dict_data, headers="keys", numalign="center", tablefmt="fancy_grid")
        camera_table = tabulate(camera_data, headers="keys", numalign="center", tablefmt="fancy_grid")

        terminal_width = shutil.get_terminal_size().columns
        lines = formated_table.splitlines() + [""] + camera_table.splitlines()
        
        print("\033[H\033[J", end="")
        print("\n\033[96mRobot sensor stauts summary:\n\033[0m")
        for line in lines:
            print(f"\033[96m{line.center(terminal_width)}\033[0m")
        
        print(f"\033[{len(lines)}A", end="", flush=True)
        

        # Log for conky GUI :)
        for file, data in [("/tmp/husky_sensor_status.txt", dict_data), ("/tmp/husky_cameras_status.txt", camera_data)]:
            with open(file, "w") as f:
                for key, value in data.items():
                    f.write(f"{key}: {value[0]}\n")

                
if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('frequency_checker')
    check_obj = Monitor()

    rate = rospy.Rate(1) # ROS Rate at 5Hz

    while not rospy.is_shutdown():
        check_obj.updateTable()
        rate.sleep()

