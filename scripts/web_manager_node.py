#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Husky Web Manager - ROS Node
Main entry point that starts the Flask server and ROS monitoring.

Usage:
    rosrun husky_manager web_manager_node.py
    roslaunch husky_manager web_manager.launch
"""

import signal
import sys
import rospy

from web_manager.config import TOPIC_GROUPS, PROCESSES, FLASK_CONFIG
from web_manager.ros_monitor import HuskyMonitor
from web_manager.app import create_app, set_monitor, FlaskServerThread


def signal_handler(sig, frame):
    """Handle shutdown signals."""
    rospy.loginfo("Shutting down Husky Web Manager...")
    sys.exit(0)


def main():
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Initialize ROS node
    rospy.init_node('husky_web_manager', anonymous=False)
    rospy.loginfo("Starting Husky Web Manager...")
    
    # Create the monitor
    monitor = HuskyMonitor(TOPIC_GROUPS, PROCESSES)
    rospy.loginfo(f"Initialized monitor with {len(TOPIC_GROUPS)} topic groups and {len(PROCESSES)} processes")
    
    # Create Flask app
    app = create_app(monitor)
    
    # Get configuration
    host = rospy.get_param('~host', FLASK_CONFIG['host'])
    port = rospy.get_param('~port', FLASK_CONFIG['port'])
    
    # Start Flask server in a thread
    server_thread = FlaskServerThread(app, host=host, port=port)
    server_thread.start()
    
    rospy.loginfo(f"Web Manager running at http://{host}:{port}/manager")
    rospy.loginfo("Press Ctrl+C to stop")
    
    # Keep the node running
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rospy.loginfo("Shutting down Flask server...")
        server_thread.shutdown()


if __name__ == '__main__':
    main()
