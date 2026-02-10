#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Configuration file for Husky Web Manager.
Define topics, processes and their display settings here.
"""

from sensor_msgs.msg import PointCloud2, NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

# Try to import custom messages, fallback if not available
try:
    from multiespectral_acquire.msg import ImageWithMetadata
    from temperature_driver.msg import TemperatureHumidity
except ImportError:
    from sensor_msgs.msg import Image as ImageWithMetadata

# =============================================================================
# TOPIC MONITORING CONFIGURATION
# =============================================================================
# Structure: {group_name: {readable_name: {"topic": topic_path, "show": "hz"|"data", "msg_type": MsgType, "data_field": field_path}}}
# - "hz": Shows publication frequency
# - "data": Shows actual data value (requires data_field)

TOPIC_GROUPS = {
    "sensors": {
        "LIDAR": {
            "topic": "/ouster/points",
            "show": "hz",
            "msg_type": PointCloud2
        },
        "GPS": {
            "topic": "/gnss/fix",
            "show": "hz",
            "msg_type": NavSatFix,
            "extra_fields": {
                "Mode GPS": {
                    "field": "status.status",
                    "mapping": {
                        -1: "No Fix",
                        0: "Fix",
                        1: "SBAS Fix",
                        2: "GBAS Fix"
                    }
                },
                "COV GPS": {
                    "field": "position_covariance",
                    "indices": [0, 4]
                },
                "Lat/Lon": {
                    "fields": ["latitude", "longitude"],
                    "format": "{0:.6f}, {1:.6f}"
                }
            }
        },
        "IMU": {
            "topic": "/imu/data",
            "show": "hz",
            "msg_type": Imu
        },
        "DHT22": {
            "topic": "/dht22/data",
            "show": "hz",
            "msg_type": TemperatureHumidity,
            "extra_fields": {
                "Data": {
                    "fields": ["temperature", "humidity"],
                    "format": "{0:.2f} °C, {1:.2f} %"
                }
            }
        }
    },
    "cameras": {
        "Mult. Visible": {
            "topic": "/Multiespectral/visible_camera/image_with_metadata_sync",
            "show": "hz",
            "msg_type": ImageWithMetadata
        },
        "Mult. LWIR": {
            "topic": "/Multiespectral/lwir_camera/image_with_metadata_sync",
            "show": "hz",
            "msg_type": ImageWithMetadata
        },
        "Fish. Frontal": {
            "topic": "/Fisheye/frontal_camera/image_with_metadata_sync",
            "show": "hz",
            "msg_type": ImageWithMetadata
        },
        "Fish. Rear": {
            "topic": "/Fisheye/rear_camera/image_with_metadata_sync",
            "show": "hz",
            "msg_type": ImageWithMetadata
        },
        "Storing": {
            "topic": "/Multiespectral/recording_enabled",
            "show": "data",
            "msg_type": Bool,
            "data_field": "data"
        }
    },
    "fusion": {
        "Odometry": {
            "topic": "/odometry/combined",
            "show": "hz",
            "msg_type": Odometry,
            "extra_fields": {
                "Position": {
                    "field": "pose.pose.position",
                    "format": "[{x:.2f}, {y:.2f}, {z:.2f}]"
                },
                "Orientation": {
                    "field": "pose.pose.orientation",
                    "format": "[{x:.2f}, {y:.2f}, {z:.2f}, {w:.2f}]"
                }
            }
        },
        "GPS Odom (Navsat)": {
            "topic": "/odometry/gps_aided",
            "show": "hz",
            "msg_type": Odometry
        },
        "Odom (interoceptive)": {
            "topic": "/odometry/dead_reckoning",
            "show": "hz",
            "msg_type": Odometry
        }
    }
}

# =============================================================================
# PROCESS MONITORING CONFIGURATION
# =============================================================================
# Structure: {readable_name: {"command": process_command, "service": systemd_service_name}}
# If "service" is provided, relaunch will use systemctl restart
# Otherwise, it will try to launch using roslaunch
# NOTE: Order matters! Processes are displayed in definition order.
#       roscore must be first as it's required by all other ROS nodes.

PROCESSES = {
    "PTP Master": {
        "command": "/usr/sbin/ptp4l",
        "service": "ptp_cameras_lidar",
        "description": "IEEE 1588 PTP daemon - Synchronizes hardware clocks of cameras and LIDAR."
    },
    "PTP Sync (phc2sys)": {
        "command": "/usr/sbin/phc2sys",
        "service": "ptp_phc2sys",
        "description": "Synchronizes NIC PHC with system CLOCK_REALTIME."
    },
    "ROS Core": {
        "command": "/opt/ros/noetic/bin/roscore",
        "service": "roscore",
        "description": "ROS Master - Central coordination of ROS nodes."
    },
    "Husky Base": {
        "command": "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch husky_base base.launch",
        "service": "husky_base",
        "description": "Husky base driver - Motor control, teleoperation and wheel odometry."
    },
    "Husky Sensors": {
        "command": "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch husky_manager sensors_manager.launch",
        "service": "sensors",
        "description": "LIDAR Ouster, GPS and IMU."
    },
    "Husky Localization": {
        "command": "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch husky_manager localization_manager.launch",
        "service": "localization",
        "description": "EKF fusion of odometry, IMU, and GPS."
    },
    "Multiespectral Cameras": {
        "command": "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch multiespectral_fb multiespectral.launch",
        "service": "multiespectral_cameras",
        "description": "Visible and LWIR cameras with synchronization buffer (LIDAR synced and cut to FOV and DHT22 sensor)."
    },
    "Fisheye Cameras": {
        "command": "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch husky_manager fisheye_cameras.launch",
        "service": "fisheye_cameras",
        "description": "Front and rear fisheye cameras with synced LIDAR."
    }
}

# =============================================================================
# FLASK SERVER CONFIGURATION
# =============================================================================
FLASK_CONFIG = {
    "host": "0.0.0.0",
    "port": 5050,  # Puerto 5050 para evitar colisiones con otras GUIs
    "debug": False,
    "url_prefix": "/manager"  # Access via localhost:5050/manager
}

# =============================================================================
# SUB-GUI LINKS CONFIGURATION
# =============================================================================
# Configure the camera GUIs accessible from the main web manager
# Each GUI runs on a different port to avoid collisions
GUI_LINKS = {
    "multiespectral": {
        "name": "Multiespectral Camera GUI",
        "port": 5051,
        "path": "/",
        "service": "multiespectral_gui"
    },
    "fisheye": {
        "name": "Fisheye Camera GUI",
        "port": 5052,
        "path": "/",
        "service": "fisheye_gui"
    }
}
