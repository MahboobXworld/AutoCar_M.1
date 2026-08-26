#!/usr/bin/env python3
# ==============================================================================
# Mahboob alam 
# Robotics and AI engineer 
# ma.mahboob2002@gmail.com
# india
# Copyright (c) 2026 Mahboob alam. All rights reserved.
# ==============================================================================

from flask import Flask, request, jsonify, render_template, send_file
import rospy
from std_msgs.msg import Float32MultiArray, Bool
from threading import Thread, Lock
import subprocess
import os
import time
import datetime
import uuid
import math
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist


class TFRosSubscriber:
    def __init__(self, max_transforms=10):
        self.latest_tf_data = []
        self.max_transforms = max_transforms
        self.lock = Lock()
        rospy.Subscriber("/tf", TFMessage, self.callback)
        rospy.loginfo("TFRosSubscriber initialized")

    def callback(self, msg):
        # Extremely fast callback: just store the raw message reference
        with self.lock:
            self.latest_tf_data.append(msg)
            if len(self.latest_tf_data) > self.max_transforms:
                self.latest_tf_data.pop(0)

    def get_tf(self):
        # Parse dynamically only when requested
        transforms_list = []
        with self.lock:
            messages = list(self.latest_tf_data)

        for msg in messages:
            for t in msg.transforms:
                transform_dict = {
                    "header": {
                        "seq": t.header.seq,
                        "stamp": {
                            "secs": t.header.stamp.secs,
                            "nsecs": t.header.stamp.nsecs
                        },
                        "frame_id": t.header.frame_id
                    },
                    "child_frame_id": t.child_frame_id,
                    "transform": {
                        "translation": {
                            "x": t.transform.translation.x,
                            "y": t.transform.translation.y,
                            "z": t.transform.translation.z
                        },
                        "rotation": {
                            "x": t.transform.rotation.x,
                            "y": t.transform.rotation.y,
                            "z": t.transform.rotation.z,
                            "w": t.transform.rotation.w
                        }
                    }
                }
                transforms_list.append(transform_dict)
        return jsonify({"transforms": transforms_list[-self.max_transforms:]})


class TeleopControl:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('bopt_teleop', anonymous=True)

        # Set up Flask web receiver
        current_dir = os.path.dirname(os.path.abspath(__file__))
        template_dir = os.path.join(current_dir, 'templates')
        self.app = Flask(__name__, template_folder=template_dir)

        # Disable console request spamming by Flask/Werkzeug
        import logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        self.app.logger.setLevel(logging.ERROR)

        # Concurrency safety lock
        import threading
        self.lock = threading.Lock()

        # State storage for incoming web telemetry
        self.received_data = None

        # Configurable watchdog timeout (default: 5.0 seconds for robust Wi-Fi telemetry)
        self.watchdog_timeout = rospy.get_param("~watchdog_timeout", 5.0)

        # Slew rate limiting for smooth command acceleration/deceleration
        self.current_throttle = 0.0
        self.current_steering = 0.0
        self.max_throttle_slew = rospy.get_param("~max_throttle_slew", 150.0)  # max throttle change per second
        self.max_steering_slew = rospy.get_param("~max_steering_slew", 720.0)  # max steering degrees change per second

        # Exclusive session lock variables
        self.active_session_id = None
        self.last_active_time = 0.0
        self.active_operator_name = "N/A"
        self.active_device_name = "N/A"
        self.active_device_uuid = "N/A"

        self.tf_subscriber = TFRosSubscriber()

        # Setup ROS publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.wheelbase = rospy.get_param("~wheelbase", 0.70)

        # Bind HTTP API routes
        self.app.route('/', methods=['GET'])(self.serve_index)
        self.app.route('/manifest.json', methods=['GET'])(self.serve_manifest)
        self.app.route('/bot/login', methods=['POST'])(self.handle_login)
        self.app.route(
            '/bot/updateoperation/CL-00001/LOC-00000001/BOT-000001',
            methods=['POST']
        )(self.update_operation)

        self.app.route('/bot/tf', methods=['GET'])(self.tf_subscriber.get_tf)
        self.app.route('/autocar.png', methods=['GET'])(self.serve_autocar_icon)

        # Run Flask endpoint in a separate thread
        self.flask_thread = Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()

        # Start ROS publisher loop
        self.publish_data()

    # HTTP server implementation

    def run_flask(self):
        try:
            from waitress import serve
            rospy.loginfo("Starting production WSGI server (waitress) on port 3000")
            serve(self.app, host='0.0.0.0', port=3000, threads=6)
        except ImportError:
            rospy.logwarn("Waitress server not installed. Falling back to Flask development server.")
            self.app.run(host='0.0.0.0', port=3000, debug=False, threaded=True)

    def serve_index(self):
        return render_template('index.html')

    def serve_manifest(self):
        return jsonify({
            "name": "AGV Remote Control Hub",
            "short_name": "AGV Control",
            "start_url": "/",
            "display": "standalone",
            "background_color": "#0b0f19",
            "theme_color": "#00bfff",
            "orientation": "portrait",
            "icons": [
                {
                    "src": "/autocar.png",
                    "sizes": "512x512",
                    "type": "image/png"
                }
            ]
        })

    def serve_autocar_icon(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        icon_path = os.path.abspath(os.path.join(current_dir, "..", "..", "png", "autocar.png"))
        return send_file(icon_path)

    def log_connection(self, event_type, username, token, ip=None, user_agent=None, operator_name=None, device_name=None, device_uuid=None):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_dir = os.path.dirname(os.path.abspath(__file__))
        log_file = os.path.join(log_dir, "teleop_connections.log")

        client_ip = "N/A"
        client_ua = "N/A"
        try:
            if ip:
                client_ip = ip
            elif request:
                client_ip = request.remote_addr
        except RuntimeError:
            pass

        try:
            if user_agent:
                client_ua = user_agent
            elif request:
                client_ua = request.headers.get("User-Agent", "N/A")
        except RuntimeError:
            pass

        dev_uuid = device_uuid or "N/A"

        log_line = (
            f"[{timestamp}] EVENT: {event_type} | "
            f"USER: {username} | "
            f"DEVICE_UUID: {dev_uuid} | "
            f"TOKEN: {token} | "
            f"IP: {client_ip} | "
            f"UA: {client_ua}\n"
        )

        try:
            with open(log_file, "a") as f:
                f.write(log_line)
        except Exception as e:
            rospy.logerr(f"Failed to write connection audit log: {e}")

    def handle_login(self):
        data = request.json or {}
        username = str(data.get("username", "")).strip().lower()
        password = str(data.get("password", "")).strip()
        operator_name = str(data.get("operator_name", "")).strip()
        device_name = str(data.get("device_name", "")).strip()
        device_uuid = str(data.get("device_uuid", "")).strip()

        if username == "admin" and password == "password123":
            now = time.time()
            with self.lock:
                # If a session is active and has sent commands within the last 2.0s
                if self.active_session_id is not None and (now - self.last_active_time) < 2.0:
                    self.log_connection("LOGIN_BUSY", username, "N/A", request.remote_addr, request.headers.get("User-Agent"), operator_name, device_name, device_uuid)
                    return jsonify({
                        "status": "busy",
                        "message": "AGV is currently controlled by another operator."
                    }), 403

                session_token = str(uuid.uuid4())
                self.active_session_id = session_token
                self.last_active_time = now
                self.active_operator_name = operator_name
                self.active_device_name = device_name
                self.active_device_uuid = device_uuid
            self.log_connection("LOGIN_SUCCESS", username, session_token, request.remote_addr, request.headers.get("User-Agent"), operator_name, device_name, device_uuid)
            return jsonify({
                "status": "authenticated",
                "session_token": session_token
            }), 200
        else:
            self.log_connection("LOGIN_FAILED", username, "N/A", request.remote_addr, request.headers.get("User-Agent"), operator_name, device_name, device_uuid)
            return jsonify({
                "status": "failed",
                "message": "Invalid username or password."
            }), 401

    def update_operation(self):
        now = time.time()
        
        # Access request JSON safely
        payload = request.json or {}
        session_token = payload.get("session_token")

        with self.lock:
            self.received_data = payload

            # Validate session token
            is_active = self.active_session_id is not None and (now - self.last_active_time) < 2.0

            if is_active:
                if session_token != self.active_session_id:
                    self.log_connection(
                        "COMMAND_REJECTED_BUSY", 
                        "N/A", 
                        session_token or "N/A", 
                        request.remote_addr, 
                        request.headers.get("User-Agent")
                    )
                    return jsonify({
                        "status": "busy",
                        "message": "AGV is currently controlled by another operator."
                    }), 403
                self.last_active_time = now
            else:
                if session_token:
                    self.active_session_id = session_token
                    self.last_active_time = now
                    # Update active metadata if received in this payload
                    self.active_operator_name = payload.get("operator_name", self.active_operator_name)
                    self.active_device_name = payload.get("device_name", self.active_device_name)
                    self.active_device_uuid = payload.get("device_uuid", self.active_device_uuid)
                else:
                    self.log_connection(
                        "COMMAND_REJECTED_UNAUTHORIZED", 
                        "N/A", 
                        "N/A", 
                        request.remote_addr, 
                        request.headers.get("User-Agent")
                    )
                    return jsonify({
                        "status": "unauthorized",
                        "message": "Authentication required."
                    }), 401

        return jsonify({
            "status": "received",
            "manual_switch": False,
            "mast_virtual_pos": 0.0
        }), 200

    # Main operational loop

    def publish_data(self):
        rate = rospy.Rate(30)
        cmd_msg = Twist()

        while not rospy.is_shutdown():
            # Watchdog timeout check
            now_time = time.time()
            
            with self.lock:
                # Use configurable watchdog timeout threshold
                if self.active_session_id is not None and (now_time - self.last_active_time) > self.watchdog_timeout:
                    rospy.logwarn(f"⚠ Watchdog timeout (threshold: {self.watchdog_timeout}s): Active operator disconnected! Releasing lock and engaging safe stop.")
                    self.log_connection(
                        "WATCHDOG_DISCONNECT", 
                        "N/A", 
                        self.active_session_id, 
                        "N/A", 
                        "N/A",
                        operator_name=self.active_operator_name,
                        device_name=self.active_device_name,
                        device_uuid=self.active_device_uuid
                    )
                    self.active_session_id = None
                    self.active_operator_name = "N/A"
                    self.active_device_name = "N/A"
                    self.active_device_uuid = "N/A"
                    self.received_data = None

                if not self.received_data:
                    # If not receiving telemetry commands, publish zero velocity so it stops safely
                    cmd_msg.linear.x = 0.0
                    cmd_msg.angular.z = 0.0
                    self.cmd_pub.publish(cmd_msg)
                    
                    rate.sleep()
                    continue

                # Safely copy values to local variables inside the lock
                received_data_copy = dict(self.received_data)

            # Calculate dt for rate limiting based on elapsed time
            now = rospy.get_time()
            dt_control = min(0.1, now - self.last_loop_time) if hasattr(self, 'last_loop_time') else 0.033
            self.last_loop_time = now

            # Extract control velocities and commands safely
            op = received_data_copy.get("operation_details", {})
            try:
                target_throttle = float(op.get("throttle", 0.0))
                target_steering = float(op.get("steering", 0.0))
            except (TypeError, ValueError, AttributeError) as e:
                rospy.logwarn(f"Malformed payload field types: {e}")
                target_throttle = 0.0
                target_steering = 0.0

            # Apply slew rate limit to throttle
            throttle_step = self.max_throttle_slew * dt_control
            throttle_diff = target_throttle - self.current_throttle
            self.current_throttle += max(-throttle_step, min(throttle_step, throttle_diff))

            # Apply slew rate limit to steering
            steering_step = self.max_steering_slew * dt_control
            steering_diff = target_steering - self.current_steering
            self.current_steering += max(-steering_step, min(steering_step, steering_diff))

            # Scale self.current_throttle (range [-50.0, 50.0]) to [-10.0, 10.0] m/s
            linear_v = self.current_throttle * 0.2

            # Calculate relative steering angle in radians (left is positive, right is negative)
            steering_rel_deg = self.current_steering
            steering_rel_rad = -steering_rel_deg * (math.pi / 180.0)

            # Calculate angular velocity (yaw rate)
            if abs(linear_v) > 0.001:
                # w = v * tan(delta) / L
                angular_w = (linear_v * math.tan(steering_rel_rad)) / self.wheelbase
            else:
                # Standstill yaw command (for standstill steering to work)
                angular_w = steering_rel_rad

            # Log control inputs and vehicle status
            rospy.logdebug(f"CMD vel={linear_v:.2f}, yaw_rate={angular_w:.2f}")

            cmd_msg.linear.x = linear_v
            cmd_msg.angular.z = angular_w
            self.cmd_pub.publish(cmd_msg)

            rate.sleep()


if __name__ == "__main__":
    TeleopControl()
