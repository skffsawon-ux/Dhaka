#!/usr/bin/env python3
"""
Health Monitor Node - Monitors critical ROS services and reports system health.

This lightweight node provides:
1. A /system/health service that returns the status of critical services
2. A /system/health topic that publishes periodic health updates
3. Captures and reports startup errors from other nodes

The app can call /system/health to get detailed error information when
services like /brain/get_available_directives are unavailable.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json
import os
import glob
from datetime import datetime
import subprocess


class HealthMonitorNode(Node):
    """
    Monitors system health and reports status of critical services.
    """

    # Services that should be available for the system to be considered healthy
    CRITICAL_SERVICES = [
        "/brain/get_available_directives",
        "/brain/get_chat_history", 
        "/brain/reset_brain",
        "/brain/set_brain_active",
    ]

    # Nodes that should be running
    CRITICAL_NODES = [
        "brain_client_node",
        "primitive_execution_action_server",
        "rosbridge_websocket",
    ]

    def __init__(self):
        super().__init__("health_monitor")
        
        # Health status publisher (publishes every 5 seconds)
        self.health_pub = self.create_publisher(String, "/system/health", 10)
        self.health_timer = self.create_timer(5.0, self.publish_health_status)
        
        # Health service for on-demand status checks
        self.health_srv = self.create_service(
            Trigger, "/system/health", self.handle_health_request
        )
        
        # Store recent errors found in logs
        self.recent_errors = []
        self.last_log_check = None
        
        # ROS log directory
        self.ros_log_dir = os.path.expanduser("~/.ros/log")
        
        self.get_logger().info("Health Monitor started - monitoring critical services")

    def check_service_available(self, service_name: str) -> bool:
        """Check if a service is available."""
        try:
            # Get list of available services
            service_names = self.get_service_names_and_types()
            available_services = [name for name, _ in service_names]
            return service_name in available_services
        except Exception as e:
            self.get_logger().debug(f"Error checking service {service_name}: {e}")
            return False

    def check_node_running(self, node_name: str) -> bool:
        """Check if a node is running."""
        try:
            node_names = self.get_node_names()
            return node_name in node_names
        except Exception as e:
            self.get_logger().debug(f"Error checking node {node_name}: {e}")
            return False

    def get_recent_crash_logs(self) -> list:
        """
        Scan recent ROS logs for crash information.
        Returns a list of error messages from crashed nodes.
        """
        errors = []
        
        try:
            # Find log directories from the last hour
            if not os.path.exists(self.ros_log_dir):
                return errors
            
            # Get all log directories, sorted by modification time
            log_dirs = glob.glob(os.path.join(self.ros_log_dir, "2*"))
            log_dirs.sort(key=os.path.getmtime, reverse=True)
            
            # Check the 5 most recent log directories
            for log_dir in log_dirs[:5]:
                launch_log = os.path.join(log_dir, "launch.log")
                if os.path.exists(launch_log):
                    try:
                        with open(launch_log, 'r') as f:
                            content = f.read()
                            # Look for error indicators
                            if "process has died" in content or "exit code" in content:
                                # Extract the relevant error line
                                for line in content.split('\n'):
                                    if "process has died" in line or "Traceback" in line:
                                        errors.append({
                                            "source": os.path.basename(log_dir),
                                            "message": line.strip()[:200]  # Truncate long messages
                                        })
                    except Exception as e:
                        self.get_logger().debug(f"Error reading log {launch_log}: {e}")
            
            # Also check for Python tracebacks in node-specific logs
            for log_dir in log_dirs[:3]:
                for log_file in glob.glob(os.path.join(log_dir, "*.log")):
                    if "brain_client" in log_file:
                        try:
                            with open(log_file, 'r') as f:
                                content = f.read()
                                if "Traceback" in content or "Error" in content:
                                    # Find the last traceback
                                    lines = content.split('\n')
                                    for i, line in enumerate(lines):
                                        if "AttributeError" in line or "TypeError" in line or "Exception" in line:
                                            errors.append({
                                                "source": "brain_client",
                                                "message": line.strip()[:200]
                                            })
                        except Exception:
                            pass
                            
        except Exception as e:
            self.get_logger().debug(f"Error scanning logs: {e}")
        
        return errors[:5]  # Return at most 5 errors

    def get_health_status(self) -> dict:
        """
        Get comprehensive health status of the system.
        """
        status = {
            "healthy": True,
            "timestamp": datetime.now().isoformat(),
            "services": {},
            "nodes": {},
            "errors": [],
            "summary": ""
        }
        
        # Check critical services
        missing_services = []
        for service in self.CRITICAL_SERVICES:
            available = self.check_service_available(service)
            status["services"][service] = available
            if not available:
                missing_services.append(service)
                status["healthy"] = False
        
        # Check critical nodes
        missing_nodes = []
        for node in self.CRITICAL_NODES:
            running = self.check_node_running(node)
            status["nodes"][node] = running
            if not running:
                missing_nodes.append(node)
                status["healthy"] = False
        
        # Get recent errors if system is unhealthy
        if not status["healthy"]:
            status["errors"] = self.get_recent_crash_logs()
        
        # Generate summary message
        if status["healthy"]:
            status["summary"] = "All systems operational"
        else:
            issues = []
            if missing_services:
                issues.append(f"Missing services: {', '.join(missing_services)}")
            if missing_nodes:
                issues.append(f"Missing nodes: {', '.join(missing_nodes)}")
            if status["errors"]:
                # Get the most relevant error message
                for err in status["errors"]:
                    if "AttributeError" in err.get("message", "") or "TypeError" in err.get("message", ""):
                        issues.append(f"Crash detected: {err['message']}")
                        break
            status["summary"] = "; ".join(issues) if issues else "System unhealthy - check logs"
        
        return status

    def publish_health_status(self):
        """Publish periodic health status."""
        try:
            status = self.get_health_status()
            msg = String()
            msg.data = json.dumps(status)
            self.health_pub.publish(msg)
            
            if not status["healthy"]:
                self.get_logger().warn(f"System unhealthy: {status['summary']}")
        except Exception as e:
            self.get_logger().error(f"Error publishing health status: {e}")

    def handle_health_request(self, request, response):
        """Handle health check service request."""
        try:
            status = self.get_health_status()
            response.success = status["healthy"]
            response.message = json.dumps(status)
        except Exception as e:
            response.success = False
            response.message = json.dumps({
                "healthy": False,
                "error": str(e),
                "summary": f"Health check failed: {e}"
            })
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

