import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry

import tf2_ros
import tf2_geometry_msgs
from tf_transformations import euler_from_quaternion

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import time

import serial
import struct

import cv2

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

        self.cv_bridge = CvBridge()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.current_target_x = None
        self.current_target_y = None
        self.current_target_id = None

        self.first_target_x, self.first_target_y = 7, 10
        self.second_target_x, self.second_target_y = 8, 17
        self.third_target_x, self.third_target_y = 0, 25
        self.fourth_target_x, self.fourth_target_y = -7, 15

        self.is_first_target_reached = False
        self.is_second_target_reached = False
        self.is_third_target_reached = False
        self.is_fourth_target_reached = False

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.align_depth_sub = self.create_subscription(Image, '/realsense/depth/color_aligned', self.align_depth_callback, 10)
        self.rgb_sub = self.create_subscription(Image, '/realsense/rgb/image_raw', self.rgb_callback, 10)

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

    def align_depth_callback(self, msg):
        return
    
    def rgb_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, 
            self.aruco_dict, 
            parameters=self.aruco_params
        )

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            for i, marker_id in enumerate(ids):
                mid = marker_id[0]
                if 51 <= mid <= 64:
                    c = corners[i][0]
                    center_x = int(np.mean(c[:, 0]))
                    center_y = int(np.mean(c[:, 1]))
                    
                    error_x = center_x - (w / 2)

                        
    def control_loop(self):
        feedback = self.read_serial_feedback()

        if feedback:
            feedback_to_meter_per_sec = 0.001 # Hoverboarddan gelen encoder verisini m/s cinsine çevirme katsayısı

            vel_right = feedback['speedR_meas'] * feedback_to_meter_per_sec # Hoverboarddan gelen sağ tekerlek encoder verisinin m/s cinsinden hızı
            vel_left = feedback['speedL_meas'] * feedback_to_meter_per_sec # Hoverboarddan gelen sol tekerlek encoder verisinin m/s cinsinden hızı

            dt = 0.05 # Time spend / Timer period
            rover_width = 0.5 # Meter

            vel_linear = (vel_right + vel_left) / 2.0 # Aracın doğrusal hızı
            vel_angular = (vel_right - vel_left) / rover_width # Aracın açısal hızı

            if self.current_x == None: # Set start point as 0,0
                self.current_x = 0.0
                self.current_y = 0.0
                self.current_yaw = 0.0

            self.current_yaw += vel_angular * dt # Aracın anlık dönme derecesi
            if self.current_yaw > math.pi: self.current_yaw -= 2*math.pi
            elif self.current_yaw < -math.pi: self.current_yaw += 2*math.pi

            self.current_x += vel_linear * math.cos(self.current_yaw) * dt # Aracın anlık x konumu
            self.current_y += vel_linear * math.sin(self.current_yaw) * dt # Aracın anlık y konumu

        if self.current_x == None:
            self.get_logger().warn("Odometry data is waiting.")
            return

        if self.is_first_target_reached == False:
            self.current_target_x = self.first_target_x
            self.current_target_y = self.first_target_y
            self.current_target_id = 1
        elif self.is_second_target_reached == False:
            self.current_target_x = self.second_target_x
            self.current_target_y = self.second_target_y
            self.current_target_id = 2
        elif self.is_third_target_reached == False:
            self.current_target_x = self.third_target_x
            self.current_target_y = self.third_target_y
            self.current_target_id = 3
        elif self.is_fourth_target_reached == False:
            self.current_target_x = self.fourth_target_x
            self.current_target_y = self.fourth_target_y
            self.current_target_id = 4
        else:
            self.current_target_x = 0.0
            self.current_target_y = 0.0
            self.current_target_id = 5

        dx = self.current_target_x - self.current_x # Hedefe olan x mesafesi
        dy = self.current_target_y - self.current_y # Hedefe olan y mesafesi

        self.target_yaw = math.atan2(dy, dx) # Hedefin x ekseni ile arasındaki açı

        self.distance_error = math.sqrt((dx)**2 + (dy)**2) # Hedefe olan uzaklık
        self.heading_error = self.target_yaw - self.current_yaw # Araç ile hedef arasındaki açı değeri

        while self.heading_error > math.pi:
            self.heading_error -= 2 * math.pi
        while self.heading_error < -math.pi:
            self.heading_error += 2 * math.pi

        Kp_linear = 0.5  # Doğrusal hareket için PID kontrolün P kısmı
        Kp_angular = 1.5 # Açısal hareket için PID kontrolün P kısmı

        self.linear_speed = Kp_linear * self.distance_error # Aracın ilerleyeceği doğrusal hız (m/s)
        self.angular_speed = Kp_angular * self.heading_error # Aracın ilerleyeceği açısal hız (m/s)

        self.max_linear_speed = 1.0 # Aracın sahip olabileceği max doğrusal hız (m/s)
        self.max_angular_speed = 1.0 # Aracın sahip olabileceği max açısal hız (m/s)
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
            match self.current_target_id:
                case 1:
                    self.is_first_target_reached = True
                case 2:
                    self.is_second_target_reached = True
                case 3:
                    self.is_third_target_reached = True
                case 4:
                    self.is_fourth_target_reached = True

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

        LINEAR_SCALE = 300.0 # Tekerleklere gidecek hız verisinin (m/s) formatından float veri tipine çevirilme katsayısı 
        ANGULAR_SCALE = 150.0 

        uSpeed = int(lin_speed * LINEAR_SCALE)
        uSteer = int(ang_speed * ANGULAR_SCALE)

        uSpeed = max(min(uSpeed, 1000), -1000) # Tekerleklere gidecek hızın max alabileceği float değeri
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