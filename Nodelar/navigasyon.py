import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry

import tf2_ros
import tf2_geometry_msgs
from tf_transformations import euler_from_quaternion

from cv_bridge import CvBridge
import numpy as np
import math
import time

import serial
import struct

class RoverNavigation(Node):
    def __init__(self):
        super().__init__("Rover_navigation_node")

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
            self.get_logger().info("Seri port baglantisi basarili.")
        except Exception as e:
            self.get_logger().error(f"Seri port hatasi: {e}")
            self.ser = None

        self.START_FRAME = 0xABCD

        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.target_x = None
        self.target_y = None

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(0.05, self.control_loop)
        
    def read_serial_feedback(self):
        if self.ser is None or not self.ser.is_open:
            return None
        if self.ser.in_waiting >= 18:
            try:
                header = self.ser.read(2)
                if len(header) < 2: return None

                header_val = struct.unpack('<H', header)[0]

                if header_val == self.START_FRAME:
                    data = self.ser.read(16)
                    if len(data) == 16:
                        unpacked_data = struct.unpack('<hhhhhhHH', data)

                        data_checksum = unpacked_data[7]
                        calc_checksum = (self.START_FRAME ^ unpacked_data[0] ^ unpacked_data[1] 
                                         ^ unpacked_data[2] ^ unpacked_data[3] ^ unpacked_data[4] 
                                         ^ unpacked_data[5] ^ unpacked_data[6]) & 0xFFFF

                        if calc_checksum == data_checksum:    
                            feedback = {
                                        'cmd1': unpacked_data[0],
                                        'cmd2': unpacked_data[1],
                                        'speedR_meas': unpacked_data[2],
                                        'speedL_meas': unpacked_data[3],
                                        'batVoltage': unpacked_data[4],
                                        'boardTemp': unpacked_data[5],
                                        'cmdLed': unpacked_data[6],
                                        'checksum': unpacked_data[7]
                                    }
                            return feedback
                        else:
                            self.get_logger().warn(f"Checksum Error! Calculated checksum: {calc_checksum}, Checksum from data: {data_checksum}")
                            return None
            except Exception as e:
                self.get_logger().warn(f"Serial Read Error: {e}")
        return None

    def control_loop(self):
        feedback = self.read_serial_feedback()

        if feedback:
            feedback_to_meter_per_sec = 0.001

            vel_right = feedback['speedR_meas'] * feedback_to_meter_per_sec
            vel_left = feedback['speedL_meas'] * feedback_to_meter_per_sec

            dt = 0.05 # Time spend / Timer period
            rover_width = 0.5 # Meter

            vel_linear = (vel_right + vel_left) / 2.0
            vel_angular = (vel_right - vel_left) / rover_width

            if self.current_x == None: # Set start point as 0,0
                self.current_x = 0.0
                self.current_y = 0.0
                self.current_yaw = 0.0

            self.current_yaw += vel_angular * dt
            if self.current_yaw > math.pi: self.current_yaw -= 2*math.pi
            elif self.current_yaw < -math.pi: self.current_yaw += 2*math.pi

            self.current_x += vel_linear * math.cos(self.current_yaw) * dt
            self.current_y += vel_linear * math.sin(self.current_yaw) * dt

        if self.current_x == None:
            self.get_logger().warn("Odometry data is waiting.")
            return

        self.target_x = 5.0
        self.target_y = 0.0

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y

        self.target_yaw = math.atan2(dy, dx)

        self.distance_error = math.sqrt((dx)**2 + (dy)**2)
        self.heading_error = self.target_yaw - self.current_yaw

        while self.heading_error > math.pi:
            self.heading_error -= 2 * math.pi
        while self.heading_error < -math.pi:
            self.heading_error += 2 * math.pi

        Kp_linear = 0.5
        Kp_angular = 1.5

        self.linear_speed = Kp_linear * self.distance_error
        self.angular_speed = Kp_angular * self.heading_error

        self.max_linear_speed = 1.0
        self.max_angular_speed = 1.0
        if self.linear_speed > self.max_linear_speed:
            self.linear_speed = self.max_linear_speed
        if self.linear_speed < -self.max_linear_speed:
            self.linear_speed = -self.max_linear_speed
        if self.angular_speed > self.max_angular_speed:
            self.angular_speed = self.max_angular_speed
        if self.angular_speed < -self.max_angular_speed:
            self.angular_speed = -self.max_angular_speed

        if self.distance_error < 0.5:
            self.linear_speed = 0
            self.angular_speed = 0
            self.get_logger().info("Target reached")

        self.send_to_hoverboard(self.linear_speed, self.angular_speed)

        self.cmd = Twist()
        self.cmd.linear.x = self.linear_speed
        self.cmd.angular.z = self.angular_speed
        self.vel_pub.publish(self.cmd)

    def send_to_hoverboard(self, lin_speed, ang_speed):
        """
        Float hız verilerini Arduino'nun beklediği int16 bayt dizisine çevirir ve gönderir.
        """
        if self.ser is None or not self.ser.is_open:
            return

        LINEAR_SCALE = 300.0  
        ANGULAR_SCALE = 150.0 

        uSpeed = int(lin_speed * LINEAR_SCALE)
        uSteer = int(ang_speed * ANGULAR_SCALE)

        uSpeed = max(min(uSpeed, 1000), -1000)
        uSteer = max(min(uSteer, 1000), -1000)

        checksum = (self.START_FRAME ^ (uSteer & 0xFFFF) ^ (uSpeed & 0xFFFF)) & 0xFFFF

        # Format: '<HhhH'
        # < : Little Endian (Arduino standart)
        # H : uint16 (Start Frame)
        # h : int16 (Steer)
        # h : int16 (Speed)
        # H : uint16 (Checksum)
        packet = struct.pack('<HhhH', self.START_FRAME, uSteer, uSpeed, checksum)

        self.ser.write(packet)

def main(args=None):
    rclpy.init(args=args)
    node = RoverNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()