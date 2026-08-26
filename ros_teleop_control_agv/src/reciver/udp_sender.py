#!/usr/bin/env python3
# ==============================================================================
# Mahboob alam 
# Robotics and AI engineer 
# ma.mahboob2002@gmail.com
# india
# Copyright (c) 2026 Mahboob alam. All rights reserved.
# ==============================================================================

import socket
import json
import rospy
from sensor_msgs.msg import LaserScan

UDP_IP = "192.168.0.83"  # Target IP address of the receiving machine
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

class UdpLaserSender:
    def __init__(self):
        rospy.Subscriber("/scan_merge/baselink", LaserScan, self.callback)
        rospy.loginfo("UDP laser sender initialized")

    def callback(self, msg):
        scan_dict = {
            'header': {
                'seq': msg.header.seq,
                'stamp': {
                    'secs': msg.header.stamp.secs,
                    'nsecs': msg.header.stamp.nsecs
                },
                'frame_id': msg.header.frame_id
            },
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'time_increment': msg.time_increment,
            'scan_time': msg.scan_time,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'ranges': msg.ranges,
            'intensities': msg.intensities
        }

        data = json.dumps(scan_dict).encode('utf-8')
        sock.sendto(data, (UDP_IP, UDP_PORT))

def main():
    rospy.init_node("udp_laser_sender", anonymous=True)
    UdpLaserSender()
    rospy.spin()

if __name__ == "__main__":
    main()
