import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, Imu, NavSatFix
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
            self.ser_read = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
            self.get_logger().info("Serial port connection is sucsessful(ser_read).")
        except Exception as e:
            self.get_logger().error(f"Serial port Error(ser_read): {e}")
            self.ser_read = None
        try:
            self.ser_write_front = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
            self.get_logger().info("Serial port connection is sucsessful(ser_write_front).")
        except Exception as e:
            self.get_logger().error(f"Serial port Error(ser_write_front): {e}")
            self.ser_write_front = None
        try:
            self.ser_write_back = serial.Serial('/dev/ttyUSB2', 115200, timeout=0.1)
            self.get_logger().info("Serial port connection is sucsessful(ser_write_back).")
        except Exception as e:
            self.get_logger().error(f"Serial port Error(ser_write_back): {e}")
            self.ser_write_back = None

        self.START_FRAME = 0xABCD # Veri paketlerinin başlangıç değeri
        self.buffer = bytearray() # Veri paketlerinin saklanacağı liste

        self.cv_bridge = CvBridge() # CV bridge

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250) # ArUco Marker kütüphanesi
        self.aruco_params = cv2.aruco.DetectorParameters() # ArUco tespiti için konfigürasyon parametreleri

        self.warmup_count = 0 # GNSS kalibrasyon için sayaç
        self.warmup_limit = 20 # GNSS kalibrasyon sayacı için max değer
        self.lat_buffer = [] # GNSS kalibrasyon için enlem değerlerinin saklanacağı liste
        self.lon_buffer = [] # GNSS kalibrasyon için boylam değerlerinin saklanacağı liste
        
        self.heading_deadband = 0.05 # Aracın tepki vereceği min dönme açı değeri
        self.min_angular_speed = 0.1 # Aracın sahip olabileceği min dönme hızı
        self.max_angular_speed = 1.0 # Aracın sahip olabileceği max dönme hızı (m/s)
        self.max_linear_speed = 1.0 # Aracın sahip olabileceği max doğrusal hız (m/s)

        self.current_yaw = 0 # Aracın bakış açısı
        self.current_x = None # Aracın anlık bulunduğu x konumu
        self.current_y = None # Aracın anlık bulunduğu x konumu
        self.current_target_x = None # Hedefin anlık bulunduğu x konumu
        self.current_target_y = None # Hedefin anlık bulunduğu y konumu
        self.current_target_id = None # Hedefin numarası
        self.gnss_origin_lat = None # GNSS başlangıç konumu (enlem)
        self.gnss_origin_lon = None # GNSS başlangıç konumu (Boylam)

        self.first_target_x, self.first_target_y = 7, 10  # İlk hedefin konumları
        self.second_target_x, self.second_target_y = 8, 17 # İkinci hedefon konumları
        self.third_target_x, self.third_target_y = 0, 25 # Üçüncü hedefin konumları
        self.fourth_target_x, self.fourth_target_y = -7, 15 # Dördüncü hedefin konumları

        self.is_first_target_reached = False # İlk hedefe ulaşıldı mı?
        self.is_second_target_reached = False # İkinci hedefe ulaşıldı mı?
        self.is_third_target_reached = False # Üçüncü Hedefe Ulaşıldı mı?
        self.is_fourth_target_reached = False # Dördüncü hedefe ulaşıldı mı?
        self.is_gnss_available = False # GNSS aktif mi?

        self.timer = self.create_timer(0.05, self.control_loop) # Zamanlayıcı
        self.last_time = self.get_clock().now().nanoseconds / 1e9 # Anlık zaman değeri

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10) # Hız yayıncısı

        self.align_depth_sub = self.create_subscription(Image, '/realsense/depth/color_aligned', self.align_depth_callback, 10) # Derinlik algı abone
        self.rgb_sub = self.create_subscription(Image, '/realsense/rgb/image_raw', self.rgb_callback, 10) # RGB abone
        self.gnss_sub = self.create_subscription(NavSatFix, '/gnss/fix', self.gnss_callback, 10) # GNSS abone
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10) # İMU abone

    def read_serial_feedback(self):
        if self.ser_read is None or not self.ser_read.is_open:
            return None
        
        waiting = self.ser_read.in_waiting
        if waiting > 0:
            new_data = self.ser_read.read(waiting)
            self.buffer.extend(new_data)

        if len(self.buffer) >= 18:
            header_bytes = b'\xCD\xAB'
            last_packet_idx = self.buffer.rfind(header_bytes)
            if last_packet_idx != -1:
                if len(self.buffer) >= last_packet_idx + 18:
                    unpacked_data = struct.unpack('<hhhhhhHH', self.buffer[last_packet_idx + 2: last_packet_idx + 18])
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
                        del(self.buffer[:last_packet_idx+18])
                        return feedback
                    else:
                        self.get_logger().warn(f"Checksum Error! Calculated checksum: {calc_checksum}, Checksum from data: {data_checksum}")
                        del(self.buffer[:last_packet_idx + 1])
                        return None
                else:
                    return
            else:
                self.buffer.clear()
                return

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

    def gnss_callback(self, msg):
        R = 6378137.0 # Dünyanın yarıçapı

        if not self.is_gnss_available:
            if self.warmup_count < self.warmup_limit:
                self.lat_buffer.append(msg.latitude)
                self.lon_buffer.append(msg.longitude)
                self.warmup_count += 1
                self.get_logger().info("Calibrating the GNSS data...")
                return
            else:
                self.gnss_origin_lat = (sum(self.lat_buffer) / len(self.lat_buffer))
                self.gnss_origin_lon = (sum(self.lon_buffer) / len(self.lon_buffer))
                self.is_gnss_available = True
                self.get_logger().info("GNSS data is calibrated")

        try:
            d_lat = math.radians(msg.latitude - self.gnss_origin_lat) # Başlangıç konumuna olan uzaklık ölçülür
            d_lon = math.radians(msg.longitude - self.gnss_origin_lon)
            
            ref_lat_rad = math.radians(self.gnss_origin_lat) # Başlangıç konumunun radyana çevirilmiş hali

            self.current_x = d_lon * R * math.cos(ref_lat_rad)
            self.current_y = d_lat * R
            
        except Exception as e:
            self.get_logger().warn(f"GNSS Read Error: {e}")
    
    def imu_callback(self, msg):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        quaternions_list = [x, y, z, w]
        self.current_yaw = euler_from_quaternion(quaternions_list)[2]

    def control_loop(self):
        feedback = self.read_serial_feedback()

        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time # Geçen zaman
        self.last_time = now

        if self.is_gnss_available == False:
            self.get_logger().warn("GNSS data is waiting.")
            return
        else:
            
            if feedback:
                feedback_to_meter_per_sec = 0.001 # Hoverboarddan gelen encoder verisini m/s cinsine çevirme katsayısı

                vel_right = feedback['speedR_meas'] * feedback_to_meter_per_sec # Hoverboarddan gelen sağ tekerlek encoder verisinin m/s cinsinden hızı
                vel_left = feedback['speedL_meas'] * feedback_to_meter_per_sec # Hoverboarddan gelen sol tekerlek encoder verisinin m/s cinsinden hızı

                rover_width = 0.5

                vel_linear = (vel_right + vel_left) / 2.0 # Aracın doğrusal hızı
            else:
                self.get_logger().warn("Serial Port Read Error")
                self.send_to_hoverboard(0, 0)
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
        self.distance_error = math.sqrt((dx)**2 + (dy)**2) # Hedefe olan uzaklık

        self.target_yaw = math.atan2(dy, dx) # Hedefin x ekseni ile arasındaki açı
        self.heading_error = self.target_yaw - self.current_yaw # Araç ile hedef arasındaki açı değeri

        while self.heading_error > math.pi:
            self.heading_error -= 2 * math.pi
        while self.heading_error < -math.pi:
            self.heading_error += 2 * math.pi

        Kp_linear = 0.5  # Doğrusal hareket için PID kontrolün P kısmı
        Kp_angular = 1.5 # Açısal hareket için PID kontrolün P kısmı

        if abs(self.heading_error) < self.heading_deadband:
            self.angular_speed = 0
            self.get_logger().info("Target is in the Rover's sight, anggular speed is zeroed.")
        else:
            raw_angular = Kp_angular * self.heading_error
        
            if raw_angular > 0:
                self.angular_speed = max(self.min_angular_speed, min(raw_angular, self.max_angular_speed)) # Aracın ilerleyeceği açısal hız (m/s)
            else:
                self.angular_speed = min(-self.min_angular_speed, max(raw_angular, -self.max_angular_speed))

        self.linear_speed = Kp_linear * self.distance_error # Aracın ilerleyeceği doğrusal hız (m/s)
        self.linear_speed = np.clip(self.linear_speed, -self.max_linear_speed, self.max_linear_speed)

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
                case 5:
                    self.get_logger().info("MİSSON COMPLETED")
                    self.send_to_hoverboard(0, 0)
                    return

        self.send_to_hoverboard(self.linear_speed, self.angular_speed)

        self.cmd = Twist()
        self.cmd.linear.x = self.linear_speed
        self.cmd.angular.z = self.angular_speed
        self.vel_pub.publish(self.cmd)

    def send_to_hoverboard(self, lin_speed, ang_speed):
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
        
        if self.ser_write_front is None or not self.ser_write_front.is_open:
            self.get_logger().warn("ser_write_front port is not open.")
        else:
            self.ser_write_front.write(packet)

        if self.ser_write_back is None or not self.ser_write_back.is_open:
            self.get_logger().warn("ser_write_back port is not open.")
        else:
            self.ser_write_back.write(packet)
        
def main(args=None):
    rclpy.init(args=args)
    node = RoverNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()  