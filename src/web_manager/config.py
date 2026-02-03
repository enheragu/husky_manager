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

PROCESSES = {
    "Husky Base": {
        "command": "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch husky_base base.launch",
        "service": "husky_base"
    },
    "Husky Sensors": {
        "command": "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch husky_manager sensors_manager.launch",
        "service": "sensors"
    },
    "Husky Localization": {
        "command": "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch husky_manager localization_manager.launch",
        "service": "localization"
    },
    "Multiespectral Cameras": {
        "command": "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch multiespectral_fb multiespectral.launch",
        "service": "multiespectral_cameras"
    },
    "Fisheye Cameras": {
        "command": "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch husky_manager fisheye_cameras.launch",
        "service": "fisheye_cameras"
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
