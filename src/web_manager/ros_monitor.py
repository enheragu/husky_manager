#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS Topic and Process Monitor Module.
Handles real-time monitoring of ROS topics and system processes.
"""

import rospy
import subprocess
import threading
import time
from typing import Dict, Any, Optional
from collections import deque


class TopicMonitor:
    """Monitor a single ROS topic for frequency and/or data."""
    
    def __init__(self, name: str, topic: str, msg_type, show: str = "hz",
                 data_field: str = None, extra_fields: dict = None):
        self.name = name
        self.topic = topic
        self.msg_type = msg_type
        self.show = show
        self.data_field = data_field
        self.extra_fields = extra_fields or {}
        
        # Frequency tracking
        self.frequency = None
        self.history = deque(maxlen=5)
        self.message_count = 0
        self.last_update = time.time()
        self.last_msg_time = time.time()
        
        # Data tracking
        self.last_data = None
        self.extra_data = {}
        
        # Configuration
        self.reset_timeout = 3.0
        self.metric_update = 1.0
        
        # Lock for thread safety
        self.lock = threading.Lock()
        
        # Subscribe to topic
        try:
            self.subscriber = rospy.Subscriber(
                self.topic, self.msg_type, self._callback, queue_size=1
            )
        except Exception as e:
            rospy.logwarn(f"Failed to subscribe to {topic}: {e}")
            self.subscriber = None
    
    def _get_nested_attr(self, obj, path: str):
        """Get nested attribute from object using dot notation."""
        try:
            for part in path.split('.'):
                obj = getattr(obj, part)
            return obj
        except AttributeError:
            return None
    
    def _callback(self, msg):
        """Handle incoming messages."""
        with self.lock:
            current_time = time.time()
            self.message_count += 1
            self.last_msg_time = current_time
            
            # Update frequency periodically
            if current_time - self.last_update >= self.metric_update:
                self.history.append(self.message_count)
                if len(self.history) > 0:
                    self.frequency = sum(self.history) / len(self.history)
                self.message_count = 0
                self.last_update = current_time
            
            # Extract data if needed
            if self.show == "data" and self.data_field:
                self.last_data = self._get_nested_attr(msg, self.data_field)
            
            # Extract extra fields
            for field_name, field_config in self.extra_fields.items():
                # Handle multiple fields (e.g., "fields": ["latitude", "longitude"])
                if "fields" in field_config:
                    try:
                        values = [self._get_nested_attr(msg, f) for f in field_config["fields"]]
                        if "format" in field_config and all(v is not None for v in values):
                            # Use positional format: "{0:.6f}, {1:.6f}"
                            self.extra_data[field_name] = field_config["format"].format(*values)
                        else:
                            self.extra_data[field_name] = values
                    except Exception:
                        self.extra_data[field_name] = None
                    continue
                
                value = self._get_nested_attr(msg, field_config.get("field", ""))
                
                # Apply mapping if exists
                if "mapping" in field_config and value is not None:
                    mapped = field_config["mapping"].get(int(value), f"Unknown ({value})")
                    self.extra_data[field_name] = f"{mapped} [{value}]"
                elif "indices" in field_config and value is not None:
                    indices = field_config["indices"]
                    try:
                        self.extra_data[field_name] = [value[i] for i in indices]
                    except (IndexError, TypeError):
                        self.extra_data[field_name] = None
                elif "format" in field_config and value is not None:
                    # Apply format template - extracts attributes from object
                    # Example: "[{x:.2f}, {y:.2f}, {z:.2f}]" for Position
                    try:
                        # Build dict from object attributes for formatting
                        fmt_dict = {}
                        if hasattr(value, '__slots__'):
                            for attr in value.__slots__:
                                fmt_dict[attr] = getattr(value, attr, 0)
                        elif hasattr(value, '__dict__'):
                            fmt_dict = {k: v for k, v in value.__dict__.items() if not k.startswith('_')}
                        else:
                            # Try common ROS message attributes
                            for attr in ['x', 'y', 'z', 'w']:
                                if hasattr(value, attr):
                                    fmt_dict[attr] = getattr(value, attr)
                        self.extra_data[field_name] = field_config["format"].format(**fmt_dict)
                    except (KeyError, AttributeError, ValueError) as e:
                        self.extra_data[field_name] = str(value)
                else:
                    self.extra_data[field_name] = value
    
    def get_status(self) -> Dict[str, Any]:
        """Get current status of the monitored topic."""
        with self.lock:
            current_time = time.time()
            
            # Check for timeout (only reset frequency for hz topics, keep data for data topics)
            timed_out = current_time - self.last_msg_time >= self.reset_timeout
            if timed_out:
                self.frequency = None
                # Don't reset last_data for "data" topics - keep last known value
                if self.show == "hz":
                    self.extra_data = {}
            
            # For data topics, check if we ever received a message
            has_data = self.last_data is not None
            is_active = (self.frequency is not None) if self.show == "hz" else has_data
            
            result = {
                "name": self.name,
                "topic": self.topic,
                "show": self.show,
                "active": is_active
            }
            
            if self.show == "hz":
                result["value"] = f"{self.frequency:.2f} Hz" if self.frequency else "N/A"
            else:
                result["value"] = str(self.last_data) if self.last_data is not None else "N/A"
            
            # Add extra fields
            for field_name, field_value in self.extra_data.items():
                if isinstance(field_value, str):
                    # Already formatted (e.g., from format template)
                    result[field_name] = field_value
                elif isinstance(field_value, list):
                    formatted = [f"{v:.4f}" if isinstance(v, float) else str(v) for v in field_value]
                    result[field_name] = f"[{', '.join(formatted)}]"
                elif field_value is not None:
                    result[field_name] = str(field_value)
                else:
                    result[field_name] = "N/A"
            
            return result


class ProcessMonitor:
    """Monitor and control system processes."""
    
    def __init__(self, processes: Dict[str, Dict[str, str]]):
        self.processes = processes
        self.status_cache = {}
        self.cache_time = 0
        self.cache_duration = 1.0  # Cache status for 1 second
        self.lock = threading.Lock()
    
    def _check_process(self, name: str, config: Dict[str, str]) -> bool:
        """Check if a process is running.
        
        First tries systemctl if a service is configured,
        then falls back to pgrep command matching.
        """
        service = config.get("service")
        
        # If service is configured, use systemctl is-active (more reliable)
        if service:
            try:
                if not service.endswith('.service'):
                    service = f"{service}.service"
                result = subprocess.run(
                    ["systemctl", "is-active", service],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                return result.stdout.strip() == "active"
            except (subprocess.TimeoutExpired, Exception):
                pass  # Fall back to pgrep
        
        # Fallback: use pgrep with partial match
        command = config.get("command", "")
        if command:
            try:
                result = subprocess.run(
                    ["pgrep", "-f", command],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                return result.returncode == 0 and result.stdout.strip() != ""
            except (subprocess.TimeoutExpired, Exception):
                return False
        
        return False
    
    def get_all_status(self) -> Dict[str, Dict[str, Any]]:
        """Get status of all monitored processes."""
        with self.lock:
            current_time = time.time()
            
            # Use cache if fresh
            if current_time - self.cache_time < self.cache_duration:
                return self.status_cache
            
            status = {}
            for name, config in self.processes.items():
                is_active = self._check_process(name, config)
                status[name] = {
                    "active": is_active,
                    "service": config.get("service"),
                    "can_relaunch": "service" in config
                }
            
            self.status_cache = status
            self.cache_time = current_time
            return status
    
    def relaunch_process(self, name: str) -> Dict[str, Any]:
        """Relaunch a process using systemctl."""
        if name not in self.processes:
            return {"success": False, "message": f"Unknown process: {name}"}
        
        config = self.processes[name]
        service = config.get("service")
        
        if not service:
            return {"success": False, "message": "No service configured for this process"}
        
        # Ensure service name ends with .service for sudoers compatibility
        if not service.endswith('.service'):
            service = f"{service}.service"
        
        try:
            # Use systemctl restart with sudo (requires NOPASSWD in sudoers)
            result = subprocess.run(
                ["sudo", "-n", "systemctl", "restart", service],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode == 0:
                return {"success": True, "message": f"Service {service} restarted successfully"}
            else:
                error_msg = result.stderr.strip()
                # Provide clearer message if it's a sudo permission issue
                if "password is required" in error_msg or "a terminal is required" in error_msg:
                    return {
                        "success": False, 
                        "message": f"Sudo permission required. Configure sudoers with: sudo visudo -f /etc/sudoers.d/husky-manager"
                    }
                return {"success": False, "message": f"Failed to restart: {error_msg}"}
        except subprocess.TimeoutExpired:
            return {"success": False, "message": "Restart timed out"}
        except Exception as e:
            return {"success": False, "message": str(e)}

    def get_logs(self, name: str, lines: int = 100) -> Dict[str, Any]:
        """Get recent logs for a process using journalctl."""
        if name not in self.processes:
            return {"success": False, "message": f"Unknown process: {name}", "logs": ""}
        
        config = self.processes[name]
        service = config.get("service")
        
        if not service:
            return {"success": False, "message": "No service configured for this process", "logs": ""}
        
        # Ensure service name ends with .service
        if not service.endswith('.service'):
            service = f"{service}.service"
        
        try:
            result = subprocess.run(
                ["journalctl", "-u", service, "-n", str(lines), "--no-pager"],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode == 0:
                return {"success": True, "logs": result.stdout}
            else:
                return {"success": False, "message": result.stderr.strip(), "logs": ""}
        except subprocess.TimeoutExpired:
            return {"success": False, "message": "Log fetch timed out", "logs": ""}
        except Exception as e:
            return {"success": False, "message": str(e), "logs": ""}


class HuskyMonitor:
    """Main monitor class that aggregates all monitoring functionality."""
    
    def __init__(self, topic_groups: Dict, processes: Dict):
        from std_msgs.msg import Bool
        
        self.topic_monitors = {}
        self.process_monitor = ProcessMonitor(processes)
        self.bool_publishers = {}  # Store latched publishers for Bool topics
        
        # Create topic monitors from config
        for group_name, topics in topic_groups.items():
            self.topic_monitors[group_name] = {}
            for topic_name, config in topics.items():
                monitor = TopicMonitor(
                    name=topic_name,
                    topic=config["topic"],
                    msg_type=config["msg_type"],
                    show=config.get("show", "hz"),
                    data_field=config.get("data_field"),
                    extra_fields=config.get("extra_fields")
                )
                self.topic_monitors[group_name][topic_name] = monitor
                
                # Create latched publisher for Bool data topics and publish initial False
                if config.get("show") == "data" and config.get("msg_type") == Bool:
                    topic_path = config["topic"]
                    pub = rospy.Publisher(topic_path, Bool, queue_size=1, latch=True)
                    self.bool_publishers[topic_path] = pub
                    # Publish initial False value after a short delay
                    rospy.Timer(rospy.Duration(1.0), 
                               lambda event, p=pub: self._publish_initial_bool(p), 
                               oneshot=True)
    
    def _publish_initial_bool(self, publisher):
        """Publish initial False value to a Bool topic."""
        from std_msgs.msg import Bool
        try:
            msg = Bool()
            msg.data = False
            publisher.publish(msg)
            rospy.loginfo(f"Published initial False to {publisher.name}")
        except Exception as e:
            rospy.logwarn(f"Failed to publish initial Bool: {e}")
    
    def get_topics_status(self) -> Dict[str, Dict[str, Any]]:
        """Get status of all monitored topics grouped."""
        result = {}
        for group_name, monitors in self.topic_monitors.items():
            result[group_name] = {}
            for topic_name, monitor in monitors.items():
                result[group_name][topic_name] = monitor.get_status()
        return result
    
    def get_processes_status(self) -> Dict[str, Dict[str, Any]]:
        """Get status of all monitored processes."""
        return self.process_monitor.get_all_status()
    
    def relaunch_process(self, name: str) -> Dict[str, Any]:
        """Relaunch a specific process."""
        return self.process_monitor.relaunch_process(name)
    
    def get_process_logs(self, name: str, lines: int = 100) -> Dict[str, Any]:
        """Get logs for a specific process from journalctl."""
        return self.process_monitor.get_logs(name, lines)
    
    def get_network_status(self) -> Dict[str, Dict[str, Any]]:
        """Get network interface statistics."""
        interfaces = {
            "enp1s0": "Cameras",
            "enp2s0": "LIDAR", 
            "wlp3s0": "WiFi"
        }
        result = {}
        
        for iface, label in interfaces.items():
            try:
                # Read from /sys/class/net for current bytes
                rx_path = f"/sys/class/net/{iface}/statistics/rx_bytes"
                tx_path = f"/sys/class/net/{iface}/statistics/tx_bytes"
                
                with open(rx_path) as f:
                    rx_bytes = int(f.read().strip())
                with open(tx_path) as f:
                    tx_bytes = int(f.read().strip())
                
                # Check if interface is up
                operstate_path = f"/sys/class/net/{iface}/operstate"
                with open(operstate_path) as f:
                    state = f.read().strip()
                
                result[iface] = {
                    "label": label,
                    "rx_bytes": rx_bytes,
                    "tx_bytes": tx_bytes,
                    "active": state == "up"
                }
            except FileNotFoundError:
                result[iface] = {
                    "label": label,
                    "rx_bytes": 0,
                    "tx_bytes": 0,
                    "active": False
                }
        
        return result
    
    def get_system_info(self) -> Dict[str, Any]:
        """Get system configuration info (IPs, ports, PTP status)."""
        import os
        
        # Environment variables
        env_vars = {
            "Wifi IP": os.environ.get("HUSKY_WIFI_IP", "Not set"),
            "OBC IP": os.environ.get("HUSKY_OBC_IP", "Not set"),
            "LIDAR IP": os.environ.get("HUSKY_LIDAR_IP", "Not set"),
            "LIDAR Dest IP": os.environ.get("HUSKY_LIDAR_IP_DEST", "Not set"),
            "GPS Port": os.environ.get("HUSKY_GPS_PORT", "Not set"),
            "IMU Port": os.environ.get("HUSKY_IMU_PORT", "Not set"),
        }
        
        # Check PTP status
        ptp_active = False
        ptp_offsets = {}
        try:
            result = subprocess.run(
                ["pgrep", "-f", "ptp4l"],
                capture_output=True, timeout=2
            )
            ptp_active = result.returncode == 0
            
            if ptp_active:
                # Try to get PTP offsets from journal
                for iface in ["enp1s0", "enp2s0"]:
                    try:
                        result = subprocess.run(
                            ["journalctl", "-u", "ptp_phc2sys", "-n", "100", "--no-pager"],
                            capture_output=True, text=True, timeout=5
                        )
                        for line in reversed(result.stdout.split('\n')):
                            if iface in line and "offset" in line:
                                import re
                                match = re.search(r'offset\s+([-\d]+)', line)
                                if match:
                                    ptp_offsets[iface] = f"{match.group(1)} ns"
                                    break
                    except:
                        pass
        except:
            pass
        
        return {
            "env": env_vars,
            "ptp_active": ptp_active,
            "ptp_offsets": ptp_offsets
        }
    
    def get_full_status(self) -> Dict[str, Any]:
        """Get complete monitoring status."""
        return {
            "topics": self.get_topics_status(),
            "processes": self.get_processes_status(),
            "network": self.get_network_status(),
            "system": self.get_system_info(),
            "timestamp": time.time()
        }

    def publish_bool_topic(self, group: str, name: str, value: bool = None) -> Dict[str, Any]:
        """Publish a boolean value to a topic (toggle if value not provided)."""
        from std_msgs.msg import Bool
        
        # Find the topic monitor
        if group not in self.topic_monitors:
            return {"success": False, "message": f"Unknown group: {group}"}
        
        if name not in self.topic_monitors[group]:
            return {"success": False, "message": f"Unknown topic: {name}"}
        
        monitor = self.topic_monitors[group][name]
        topic_path = monitor.topic
        
        # Determine the value to publish
        if value is None:
            # Toggle: get current value and invert
            current = monitor.last_data
            if current is None:
                value = True  # Default to True if no current value
            else:
                value = not bool(current)
        
        try:
            # Create a publisher for this topic
            pub = rospy.Publisher(topic_path, Bool, queue_size=1, latch=True)
            rospy.sleep(0.1)  # Give time for publisher to register
            
            msg = Bool()
            msg.data = bool(value)
            pub.publish(msg)
            
            return {
                "success": True, 
                "message": f"Published {value} to {topic_path}",
                "value": value
            }
        except Exception as e:
            return {"success": False, "message": str(e)}
