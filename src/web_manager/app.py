#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Flask Web Application for Husky Manager.
Provides a web interface for monitoring ROS topics and processes.
"""

import os
import threading
from flask import Flask, Blueprint, jsonify, render_template, request, after_this_request

# Blueprint for the manager routes (enables /manager prefix)
manager_bp = Blueprint('manager', __name__, 
                       url_prefix='/manager',
                       template_folder='templates',
                       static_folder='static')


@manager_bp.after_request
def add_no_cache_headers(response):
    """Add headers to prevent caching of API responses."""
    if request.path.startswith('/manager/api'):
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Pragma'] = 'no-cache'
        response.headers['Expires'] = '0'
    return response

# Global monitor instance (set during initialization)
_monitor = None


def set_monitor(monitor):
    """Set the global monitor instance."""
    global _monitor
    _monitor = monitor


@manager_bp.route('/')
def index():
    """Render the main dashboard page."""
    return render_template('index.html')


@manager_bp.route('/api/status')
def api_status():
    """Get full monitoring status (topics + processes)."""
    if _monitor is None:
        return jsonify({"error": "Monitor not initialized"}), 503
    
    try:
        status = _monitor.get_full_status()
        return jsonify(status)
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@manager_bp.route('/api/topics')
def api_topics():
    """Get topics status grouped."""
    if _monitor is None:
        return jsonify({"error": "Monitor not initialized"}), 503
    
    try:
        status = _monitor.get_topics_status()
        return jsonify(status)
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@manager_bp.route('/api/topics/<group>')
def api_topics_group(group):
    """Get topics status for a specific group."""
    if _monitor is None:
        return jsonify({"error": "Monitor not initialized"}), 503
    
    try:
        status = _monitor.get_topics_status()
        if group in status:
            return jsonify(status[group])
        else:
            return jsonify({"error": f"Unknown group: {group}"}), 404
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@manager_bp.route('/api/processes')
def api_processes():
    """Get processes status."""
    if _monitor is None:
        return jsonify({"error": "Monitor not initialized"}), 503
    
    try:
        status = _monitor.get_processes_status()
        return jsonify(status)
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@manager_bp.route('/api/processes/<name>/relaunch', methods=['POST'])
def api_relaunch_process(name):
    """Relaunch a specific process."""
    if _monitor is None:
        return jsonify({"error": "Monitor not initialized"}), 503
    
    try:
        result = _monitor.relaunch_process(name)
        status_code = 200 if result.get("success") else 500
        return jsonify(result), status_code
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500


@manager_bp.route('/api/processes/<name>/stop', methods=['POST'])
def api_stop_process(name):
    """Stop a specific process gracefully (allows ROS nodes to shutdown cleanly)."""
    if _monitor is None:
        return jsonify({"error": "Monitor not initialized"}), 503
    
    try:
        result = _monitor.stop_process(name)
        status_code = 200 if result.get("success") else 500
        return jsonify(result), status_code
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500


@manager_bp.route('/api/processes/<name>/logs')
def api_process_logs(name):
    """Get logs for a specific process."""
    if _monitor is None:
        return jsonify({"error": "Monitor not initialized"}), 503
    
    lines = request.args.get('lines', 100, type=int)
    
    try:
        result = _monitor.get_process_logs(name, lines)
        return jsonify(result)
    except Exception as e:
        return jsonify({"success": False, "logs": str(e)}), 500


@manager_bp.route('/api/topics/<group>/<name>/toggle', methods=['POST'])
def api_toggle_topic(group, name):
    """Toggle a boolean topic value."""
    if _monitor is None:
        return jsonify({"error": "Monitor not initialized"}), 503
    
    try:
        # Handle both empty body and JSON body safely
        data = {}
        if request.content_length and request.content_length > 0:
            data = request.get_json(silent=True) or {}
        value = data.get('value', None)
        result = _monitor.publish_bool_topic(group, name, value)
        return jsonify(result)
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500


@manager_bp.route('/api/config')
def api_config():
    """Get current configuration (topic groups structure)."""
    from .config import TOPIC_GROUPS, PROCESSES
    
    # Return serializable version of config
    config = {
        "topic_groups": list(TOPIC_GROUPS.keys()),
        "processes": list(PROCESSES.keys())
    }
    return jsonify(config)


@manager_bp.route('/api/guis')
def api_guis():
    """Get available sub-GUI links configuration."""
    from .config import GUI_LINKS
    return jsonify(GUI_LINKS)


def create_app(monitor=None):
    """Create and configure the Flask application."""
    app = Flask(__name__,
                template_folder=os.path.join(os.path.dirname(__file__), 'templates'),
                static_folder=os.path.join(os.path.dirname(__file__), 'static'))
    
    # Configure app
    app.config['SECRET_KEY'] = 'husky-manager-secret-key'
    app.config['JSON_SORT_KEYS'] = False
    
    # Set monitor if provided
    if monitor:
        set_monitor(monitor)
    
    # Register blueprint
    app.register_blueprint(manager_bp)
    
    # Root redirect to manager
    @app.route('/')
    def root_redirect():
        return '<script>window.location.href="/manager";</script>'
    
    return app


class FlaskServerThread(threading.Thread):
    """Thread to run Flask server in background."""
    
    def __init__(self, app, host='0.0.0.0', port=5050):
        super().__init__(daemon=True)
        self.app = app
        self.host = host
        self.port = port
        self.server = None
    
    def run(self):
        """Start the Flask server."""
        from werkzeug.serving import make_server
        self.server = make_server(self.host, self.port, self.app, threaded=True)
        self.server.serve_forever()
    
    def shutdown(self):
        """Shutdown the Flask server."""
        if self.server:
            self.server.shutdown()
