#!/usr/bin/env python3
"""
Intel RealSense D435i Publisher Node
Tüm kamera stream'lerini ROS topic'leri olarak yayınlar
Intel RealSense D435i kamerasından alınan görüntüleri ROS sistem mimarimize yayınlar
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu, Temperature
from std_msgs.msg import Header
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import time


class RealSensePublisher(Node):
    """
    Intel RealSense D435i kamerasından tüm stream'leri ROS 2'ye yayınlar
    
    Yayınlanan Topic'ler:
    - /realsense/rgb/image_raw          : RGB görüntü (848x480 @ 30 FPS)
    - /realsense/rgb/camera_info        : RGB kamera kalibrasyon bilgisi
    - /realsense/depth/image_rect       : Depth görüntü (848x480 @ 30 FPS)
    - /realsense/depth/camera_info      : Depth kamera kalibrasyon bilgisi
    - /realsense/depth/color_aligned    : RGB'ye hizalanmış depth
    - /realsense/infra1/image_rect      : Sol kızılötesi kamera
    - /realsense/infra2/image_rect      : Sağ kızılötesi kamera
    - /realsense/imu/accel              : İvmeölçer verisi (D435i'de var)
    - /realsense/imu/gyro               : Jiroskop verisi (D435i'de var)
    """
    
    def __init__(self):
        super().__init__('realsense_publisher_node')
        
        # Parametreler
        self.declare_parameter('rgb_width', 848)
        self.declare_parameter('rgb_height', 480)
        self.declare_parameter('depth_width', 848)
        self.declare_parameter('depth_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('enable_imu', True)
        self.declare_parameter('enable_infrared', True)
        self.declare_parameter('align_depth_to_color', True)
        
        self.rgb_width = self.get_parameter('rgb_width').value
        self.rgb_height = self.get_parameter('rgb_height').value
        self.depth_width = self.get_parameter('depth_width').value
        self.depth_height = self.get_parameter('depth_height').value
        self.fps = self.get_parameter('fps').value
        self.enable_imu = self.get_parameter('enable_imu').value
        self.enable_infrared = self.get_parameter('enable_infrared').value
        self.align_depth = self.get_parameter('align_depth_to_color').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # RealSense pipeline ve config
        self.pipeline = None
        self.config = None
        self.align = None
        
        # Publishers oluştur
        self.create_publishers()
        
        # RealSense'i başlat
        self.initialize_realsense()
        
        # Timer ile frame'leri publish et
        self.timer = self.create_timer(1.0 / self.fps, self.publish_frames)
        
        # İstatistikler
        self.frame_count = {
            'rgb': 0,
            'depth': 0,
            'infra1': 0,
            'infra2': 0,
            'aligned_depth': 0,
            'imu_accel': 0,
            'imu_gyro': 0
        }
        
        self.get_logger().info('🎥 RealSense Publisher Node başlatıldı')
        self.get_logger().info(f'📸 RGB: {self.rgb_width}x{self.rgb_height} @ {self.fps} FPS')
        self.get_logger().info(f'📏 Depth: {self.depth_width}x{self.depth_height} @ {self.fps} FPS')
        self.get_logger().info(f'🔧 IMU: {"Aktif" if self.enable_imu else "Pasif"}')
        self.get_logger().info(f'🔧 Infrared: {"Aktif" if self.enable_infrared else "Pasif"}')
        self.get_logger().info(f'🔧 Aligned Depth: {"Aktif" if self.align_depth else "Pasif"}')
    
    def create_publishers(self):
        """Tüm ROS publisher'ları oluştur"""
        
        # RGB görüntü ve camera info
        self.pub_rgb = self.create_publisher(
            Image, '/realsense/rgb/image_raw', 10)
        self.pub_rgb_info = self.create_publisher(
            CameraInfo, '/realsense/rgb/camera_info', 10)
        
        # Depth görüntü ve camera info
        self.pub_depth = self.create_publisher(
            Image, '/realsense/depth/image_rect', 10)
        self.pub_depth_info = self.create_publisher(
            CameraInfo, '/realsense/depth/camera_info', 10)
        
        # RGB'ye hizalanmış depth
        if self.align_depth:
            self.pub_aligned_depth = self.create_publisher(
                Image, '/realsense/depth/color_aligned', 10)
        
        # Infrared kameralar
        if self.enable_infrared:
            self.pub_infra1 = self.create_publisher(
                Image, '/realsense/infra1/image_rect', 10)
            self.pub_infra2 = self.create_publisher(
                Image, '/realsense/infra2/image_rect', 10)
        
        # IMU verileri (D435i'de mevcut)
        if self.enable_imu:
            self.pub_imu_accel = self.create_publisher(
                Imu, '/realsense/imu/accel', 10)
            self.pub_imu_gyro = self.create_publisher(
                Imu, '/realsense/imu/gyro', 10)
        
        self.get_logger().info('✅ Tüm publisher\'lar oluşturuldu')
    
    def initialize_realsense(self):
        """RealSense kamerasını başlat ve yapılandır"""
        try:
            # Pipeline ve config oluştur
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Cihazı bul
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                self.get_logger().error('❌ RealSense cihazı bulunamadı!')
                raise RuntimeError('No RealSense device found')
            
            device = devices[0]
            device_name = device.get_info(rs.camera_info.name)
            serial = device.get_info(rs.camera_info.serial_number)
            self.get_logger().info(f'📷 Cihaz bulundu: {device_name} (S/N: {serial})')
            
            # Serial number ile cihazı seç
            self.config.enable_device(serial)
            
            # RGB stream
            self.config.enable_stream(
                rs.stream.color,
                self.rgb_width,
                self.rgb_height,
                rs.format.bgr8,
                self.fps
            )
            
            # Depth stream
            self.config.enable_stream(
                rs.stream.depth,
                self.depth_width,
                self.depth_height,
                rs.format.z16,
                self.fps
            )
            
            # Infrared streams
            if self.enable_infrared:
                self.config.enable_stream(
                    rs.stream.infrared, 1,
                    self.depth_width, self.depth_height,
                    rs.format.y8, self.fps
                )
                self.config.enable_stream(
                    rs.stream.infrared, 2,
                    self.depth_width, self.depth_height,
                    rs.format.y8, self.fps
                )
            
            # IMU streams (D435i için)
            if self.enable_imu:
                try:
                    self.config.enable_stream(rs.stream.accel)
                    self.config.enable_stream(rs.stream.gyro)
                    self.get_logger().info('✅ IMU stream\'leri aktifleştirildi')
                except Exception as e:
                    self.get_logger().warn(f'⚠️ IMU başlatılamadı (D435 modelinde IMU yok): {e}')
                    self.enable_imu = False
            
            # Pipeline'ı başlat
            profile = self.pipeline.start(self.config)
            
            # Align objesi oluştur (depth'i RGB'ye hizalamak için)
            if self.align_depth:
                align_to = rs.stream.color
                self.align = rs.align(align_to)
            
            # Birkaç frame atla (otomatik exposure için)
            for _ in range(30):
                self.pipeline.wait_for_frames()
            
            self.get_logger().info('✅ RealSense başarıyla başlatıldı')
            
        except Exception as e:
            self.get_logger().error(f'❌ RealSense başlatma hatası: {e}')
            raise
    
    def publish_frames(self):
        """Her timer tick'inde frame'leri al ve publish et"""
        try:
            # Frame set al
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
            
            # Timestamp oluştur
            timestamp = self.get_clock().now().to_msg()
            
            # RGB frame
            color_frame = frames.get_color_frame()
            if color_frame:
                self.publish_rgb_frame(color_frame, timestamp)
            
            # Depth frame
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                self.publish_depth_frame(depth_frame, timestamp)
            
            # Aligned depth
            if self.align_depth and color_frame and depth_frame:
                aligned_frames = self.align.process(frames)
                aligned_depth = aligned_frames.get_depth_frame()
                if aligned_depth:
                    self.publish_aligned_depth_frame(aligned_depth, timestamp)
            
            # Infrared frames
            if self.enable_infrared:
                infra1 = frames.get_infrared_frame(1)
                infra2 = frames.get_infrared_frame(2)
                if infra1:
                    self.publish_infrared_frame(infra1, timestamp, 1)
                if infra2:
                    self.publish_infrared_frame(infra2, timestamp, 2)
            
            # IMU verileri
            if self.enable_imu:
                # Accel
                accel_frame = frames.first_or_default(rs.stream.accel)
                if accel_frame:
                    self.publish_imu_accel(accel_frame, timestamp)
                
                # Gyro
                gyro_frame = frames.first_or_default(rs.stream.gyro)
                if gyro_frame:
                    self.publish_imu_gyro(gyro_frame, timestamp)
        
        except Exception as e:
            self.get_logger().error(f'❌ Frame publish hatası: {e}')
    
    def publish_rgb_frame(self, frame, timestamp):
        """RGB frame'i ROS topic'e publish et"""
        # Frame'i numpy array'e çevir
        rgb_image = np.asanyarray(frame.get_data())
        
        # ROS Image mesajı oluştur
        msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='bgr8')
        msg.header.stamp = timestamp
        msg.header.frame_id = 'realsense_color_optical_frame'
        
        # Publish et
        self.pub_rgb.publish(msg)
        
        # Camera info publish et
        rgb_info = self.create_camera_info(frame, timestamp, 'realsense_color_optical_frame')
        self.pub_rgb_info.publish(rgb_info)
        
        self.frame_count['rgb'] += 1
    
    def publish_depth_frame(self, frame, timestamp):
        """Depth frame'i ROS topic'e publish et"""
        # Frame'i numpy array'e çevir (16-bit)
        depth_image = np.asanyarray(frame.get_data())
        
        # ROS Image mesajı oluştur
        msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
        msg.header.stamp = timestamp
        msg.header.frame_id = 'realsense_depth_optical_frame'
        
        # Publish et
        self.pub_depth.publish(msg)
        
        # Camera info publish et
        depth_info = self.create_camera_info(frame, timestamp, 'realsense_depth_optical_frame')
        self.pub_depth_info.publish(depth_info)
        
        self.frame_count['depth'] += 1
    
    def publish_aligned_depth_frame(self, frame, timestamp):
        """RGB'ye hizalanmış depth frame'i publish et"""
        depth_image = np.asanyarray(frame.get_data())
        
        msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
        msg.header.stamp = timestamp
        msg.header.frame_id = 'realsense_color_optical_frame'
        
        self.pub_aligned_depth.publish(msg)
        self.frame_count['aligned_depth'] += 1
    
    def publish_infrared_frame(self, frame, timestamp, camera_num):
        """Infrared frame'i publish et"""
        ir_image = np.asanyarray(frame.get_data())
        
        msg = self.bridge.cv2_to_imgmsg(ir_image, encoding='mono8')
        msg.header.stamp = timestamp
        msg.header.frame_id = f'realsense_infra{camera_num}_optical_frame'
        
        if camera_num == 1:
            self.pub_infra1.publish(msg)
            self.frame_count['infra1'] += 1
        else:
            self.pub_infra2.publish(msg)
            self.frame_count['infra2'] += 1
    
    def publish_imu_accel(self, frame, timestamp):
        """IMU ivmeölçer verisini publish et"""
        accel_data = frame.as_motion_frame().get_motion_data()
        
        msg = Imu()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'realsense_imu_optical_frame'
        
        msg.linear_acceleration.x = float(accel_data.x)
        msg.linear_acceleration.y = float(accel_data.y)
        msg.linear_acceleration.z = float(accel_data.z)
        
        self.pub_imu_accel.publish(msg)
        self.frame_count['imu_accel'] += 1
    
    def publish_imu_gyro(self, frame, timestamp):
        """IMU jiroskop verisini publish et"""
        gyro_data = frame.as_motion_frame().get_motion_data()
        
        msg = Imu()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'realsense_imu_optical_frame'
        
        msg.angular_velocity.x = float(gyro_data.x)
        msg.angular_velocity.y = float(gyro_data.y)
        msg.angular_velocity.z = float(gyro_data.z)
        
        self.pub_imu_gyro.publish(msg)
        self.frame_count['imu_gyro'] += 1
    
    def create_camera_info(self, frame, timestamp, frame_id):
        """Camera calibration bilgisini oluştur"""
        intrinsics = frame.profile.as_video_stream_profile().intrinsics
        
        msg = CameraInfo()
        msg.header.stamp = timestamp
        msg.header.frame_id = frame_id
        
        msg.width = intrinsics.width
        msg.height = intrinsics.height
        msg.distortion_model = 'plumb_bob'
        
        # Distortion coefficients [k1, k2, t1, t2, k3]
        msg.d = [intrinsics.coeffs[i] for i in range(5)]
        
        # Intrinsic camera matrix (K)
        msg.k = [
            intrinsics.fx, 0.0, intrinsics.ppx,
            0.0, intrinsics.fy, intrinsics.ppy,
            0.0, 0.0, 1.0
        ]
        
        # Rectification matrix (R) - identity için
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Projection matrix (P)
        msg.p = [
            intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
            0.0, intrinsics.fy, intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        return msg
    
    def print_stats(self):
        """İstatistikleri yazdır"""
        self.get_logger().info('📊 Frame İstatistikleri:')
        for stream, count in self.frame_count.items():
            self.get_logger().info(f'  {stream:15s}: {count:6d} frames')
    
    def destroy_node(self):
        """Node kapatılırken temizlik yap"""
        self.get_logger().info('🛑 RealSense Publisher Node kapatılıyor...')
        self.print_stats()
        
        if self.pipeline:
            self.pipeline.stop()
            self.get_logger().info('✅ RealSense pipeline durduruldu')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = RealSensePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
