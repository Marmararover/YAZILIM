#!/usr/bin/env python3
# GÜVENLİK: sudo usermod -a -G dialout $USER

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import serial
import struct
import math
import time
import os
from threading import Lock

class RoverMaster(Node):
    def __init__(self):
        super().__init__('rover_ornekleme_node')
        
        self.get_logger().info(" ROVER ORNEKLEME BAŞLATILIYOR...")
        
        # --- KİLİTLER (THREAD SAFETY) ---
        self.data_lock = Lock() # Veri koruma için
        self.yolo_lock = Lock() # Yolo yükünü koruma için

        # --- AYARLAR ---
        self.DEBUG_VISUAL = False # İhtiyaç halinde açılır
        self.TIMEOUT_LIMIT = 0.7  # Kayıp görsel için sınır süre
        
        # --- MOTOR AYARLARI ---
        self.serial_port = '/dev/ttyUSB0' 
        self.baud_rate = 115200
        self.START_FRAME = 0xABCD
        
        self.SPEED_COEFF = 600.0   
        self.STEER_COEFF = 250.0   
        
        self.MAX_SPEED = 0.25      # m/s
        self.MAX_TURN = 0.6        # rad/s

        # --- YOLO & DOSYA AYARLARI ---
        self.TARGET_CLASS_ID = None # Belki başka şeyler de algılanması gerekirse diye ekledim
        self.CONFIDENCE_THRESHOLD = 0.7 # Emin olma yüzdesi
        
        user_home = os.path.expanduser("~")
        self.desktop_path = user_home # Mevcut bilgisayarın home yolunu bulur
        if os.path.exists(os.path.join(user_home, "Masaüstü")):
            self.desktop_path = os.path.join(user_home, "Masaüstü")
        elif os.path.exists(os.path.join(user_home, "Desktop")):
            self.desktop_path = os.path.join(user_home, "Desktop")
        # Fotoğrafların kaydedileceği klasör oluşturulur.    
        self.save_dir = os.path.join(self.desktop_path, "rover_fotograflar")
        os.makedirs(self.save_dir, exist_ok=True)

        # Model Yükleme
        model_name = "best.pt" 
        model_path = os.path.join(self.desktop_path, model_name) # best.pt masaüstünde bulundurulmalı!
        
        try:
            if os.path.exists(model_path):
                # Model varsa yükle
                self.get_logger().info(f' Özel Model Yüklendi: {model_path}')
                self.model = YOLO(model_path)
            else:
                error_msg = f" Kritik Hata: '{model_name}' dosyası Masaüstünde bulunamadı!"
                self.get_logger().error(error_msg)
                raise FileNotFoundError(error_msg)
                
        except Exception as e:
            self.get_logger().error(f' Model Başlatılamadı: {e}')
            raise e

        # --- Navigasyon ---
        self.global_goal_x = 61.0
        self.global_goal_y = 34.0
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        # -Hız
        self.prev_v = 0.0     
        self.prev_w = 0.0
        
        self.state = "GLOBAL_APPROACH"
        
        # Hedef Değişkenleri 
        self.visual_valid = False
        self.target_dist = 0.0  # Hedefe olan mesafe (metre)
        self.target_angle = 0.0 # Hedefin açısı (radyan)
        self.last_visual_time = 0.0
        
        # Zamanlayıcılar
        self.last_loop_time = self.get_clock().now()
        self.search_start_time = None
        self.arrival_time = None
        self.last_photo_time = 0.0
        self.photo_count = 0
        self.TARGET_PHOTO_LIMIT = 5
        self.last_yolo_time = 0
        self.YOLO_INTERVAL = 0.15 # ~7 fps(yaklaşık)

        self.fx = 600.0; self.cx = 320.0

        # --- BAĞLANTILAR ---
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(" Seri Port Aktif.")
        except Exception as e:
            self.get_logger().error(f" SERİ PORT YOK: {e}")
            self.ser = None

        self.bridge = CvBridge()
        self.rgb_frame = None
        self.depth_frame = None
        
        self.create_subscription(Image, '/realsense/rgb/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/realsense/depth/image_rect', self.depth_callback, 10)
        self.create_subscription(Point, '/mission/goal', self.mission_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

    # --- CALLBACK'LER ---
    def rgb_callback(self, msg):
        with self.data_lock:
            try: self.rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except: pass
        
        now = time.time()
        if now - self.last_yolo_time > self.YOLO_INTERVAL:
            self.process_yolo() 
            self.last_yolo_time = now

    def depth_callback(self, msg):
        with self.data_lock:
            try: self.depth_frame = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            except: pass

    def mission_callback(self, msg):
        self.global_goal_x = msg.x
        self.global_goal_y = msg.y
        self.state = "GLOBAL_APPROACH"
        self.photo_count = 0
        self.arrival_time = None
        self.search_start_time = None
        self.get_logger().info(f" Hedef: {msg.x:.1f}, {msg.y:.1f}")

    # --- GÖRÜNTÜ İŞLEME (GÜVENLİ VE OPTİMİZE) ---
    def process_yolo(self):
        current_rgb = None
        current_depth = None
        
        # Veri kopyalama (Okuma kilidi)
        with self.data_lock:
            if self.rgb_frame is None: return
            current_rgb = self.rgb_frame.copy()
            if self.depth_frame is not None: current_depth = self.depth_frame.copy()

        has_display = os.environ.get("DISPLAY") is not None
        results = None

        # Birden fazla ID olursa diye kondu,sadece kaya için eğitilecekse burası silinecek
        # Bu kilit model çalışırken başka nesnenin müdahalesini engeller
        with self.yolo_lock:
            results = self.model(current_rgb, verbose=False)

        target_found = False
        best_box = None
        max_conf = 0

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                
                if self.TARGET_CLASS_ID is not None and cls_id != self.TARGET_CLASS_ID:
                    continue 

                if conf > self.CONFIDENCE_THRESHOLD:
                    if conf > max_conf:
                        max_conf = conf
                        best_box = box.xyxy[0].cpu().numpy()
                        target_found = True
        
        if target_found and current_depth is not None:
            if current_depth.shape == current_rgb.shape[:2]:
                x1, y1, x2, y2 = map(int, best_box)
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                try:
                    dist_mm = current_depth[center_y, center_x]
                    if dist_mm > 0:
                        dist_m = dist_mm / 1000.0
                        
                        # HEDEF HESAPLAMA
                        # angle = (pixel_farkı) / odak_uzaklığı
                        angle_rad = -1 * (center_x - self.cx) / self.fx 
                        
                        with self.data_lock:
                            self.target_dist = float(dist_m)
                            self.target_angle = float(angle_rad)
                            self.visual_valid = True
                            self.last_visual_time = time.time()
                        
                        if has_display and self.DEBUG_VISUAL: 
                            cv2.rectangle(current_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(current_rgb, f"{dist_m:.2f}m", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                except: pass

        # DEBUG GÖRÜNTÜSÜ 
        if has_display and self.DEBUG_VISUAL:
            cv2.imshow("Rover View", current_rgb)
            cv2.waitKey(1)

    # --- KONTROL DÖNGÜSÜ ---
    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_loop_time).nanoseconds / 1e9
        self.last_loop_time = now
        if dt > 1.0: dt = 0.1

        # Odometri
        self.pose_yaw += self.prev_w * dt
        self.pose_yaw = math.atan2(math.sin(self.pose_yaw), math.cos(self.pose_yaw))
        self.pose_x += self.prev_v * math.cos(self.pose_yaw) * dt
        self.pose_y += self.prev_v * math.sin(self.pose_yaw) * dt

        # Görsel Timeout Kontrolü 
        current_sys_time = time.time()
        with self.data_lock:
            if current_sys_time - self.last_visual_time > self.TIMEOUT_LIMIT:
                self.visual_valid = False

        target_v = 0.0
        target_w = 0.0

        if self.state == "STOPPED":
            if self.arrival_time is None:
                self.arrival_time = current_sys_time
                self.last_photo_time = current_sys_time
                self.get_logger().info(" Çekim Başlıyor...")
            elif current_sys_time - self.arrival_time > 3.0: 
                if self.state != "MISSION_COMPLETE":
                    if current_sys_time - self.last_photo_time > 1.0:
                        self.take_snapshot()
                        self.photo_count += 1
                        self.last_photo_time = current_sys_time
                        if self.photo_count >= self.TARGET_PHOTO_LIMIT:
                            self.state = "MISSION_COMPLETE"
                            self.get_logger().info(" BİTTİ!")

        elif self.state == "MISSION_COMPLETE":
            target_v = 0.0; target_w = 0.0

        elif self.visual_valid:
            self.state = "VISUAL_SERVOING"
            dist_val, angle_val = 0.0, 0.0
            
            # Kilitli(lock) okuma
            with self.data_lock: 
                dist_val = self.target_dist
                angle_val = self.target_angle
            
            if dist_val < 0.5:
                self.state = "STOPPED"
            else:
                # --- TANK GÖRSEL SÜRÜŞÜ ---
                # Açı hatası çok büyükse (> 20 derece / 0.35 rad) -> Dur ve Dön
                if abs(angle_val) > 0.35:
                    target_v = 0.0
                    target_w = 0.5 if angle_val > 0 else -0.5
                
                # Açı hatası azsa -> Hem git hem düzelt
                else:
                    target_v = self.MAX_SPEED
                    # Basit Oransal Kontrol (P-Controller)
                    # angle_val * Kp (0.8)
                    target_w = angle_val * 0.8 

        else:
            # Kör Sürüş
            if self.state == "VISUAL_SERVOING":
                self.state = "SEARCHING"
                self.search_start_time = current_sys_time
                self.get_logger().warn(" Görüntü gitti!")

            dx = self.global_goal_x - self.pose_x
            dy = self.global_goal_y - self.pose_y
            dist = math.sqrt(dx**2 + dy**2)

            if dist < 0.5:
                self.state = "SEARCHING"
                if self.search_start_time is None:
                    self.search_start_time = current_sys_time
                    self.get_logger().info(" Bölge Araması...")
                
                elapsed = current_sys_time - self.search_start_time
                if elapsed < 5.0:
                    target_w = 0.4 
                elif elapsed < 25.0:
                    boost = (elapsed - 5.0) * 0.01
                    target_v = min(0.1 + boost, 0.2)
                    target_w = 0.4
                else:
                    self.state = "MISSION_COMPLETE"
                    self.get_logger().error(" Bulunamadı.")
            else:
                self.state = "GLOBAL_APPROACH"
                self.search_start_time = None
                t_angle = math.atan2(dy, dx)
                err = math.atan2(math.sin(t_angle - self.pose_yaw), math.cos(t_angle - self.pose_yaw))
                
                if abs(err) > 0.5: 
                    target_v = 0.0
                    target_w = 0.5 if err > 0 else -0.5
                elif abs(err) > 0.2:
                    target_v = self.MAX_SPEED * 0.5
                    target_w = err * 1.0
                else: 
                    target_v = self.MAX_SPEED
                    target_w = err * 0.5

        # Limitler
        target_v = max(min(target_v, self.MAX_SPEED), -self.MAX_SPEED)
        target_w = max(min(target_w, self.MAX_TURN), -self.MAX_TURN)
        self.prev_v = target_v
        self.prev_w = target_w

        self.send_to_motor(target_v, target_w)

    def send_to_motor(self, v_x, v_z):
        if self.ser is None: return

        speed = int(v_x * self.SPEED_COEFF)
        steer = int(v_z * self.STEER_COEFF)
        
        speed = max(min(speed, 1000), -1000)
        steer = max(min(steer, 1000), -1000)

        try:
            checksum = (self.START_FRAME ^ steer ^ speed) & 0xFFFF
            packet = struct.pack('<HhhH', self.START_FRAME, steer, speed, checksum)
            self.ser.write(packet)
        except Exception as e:
            self.get_logger().error(f"Motor Hatası: {e}")

    def take_snapshot(self):
        frame_to_save = None
        with self.data_lock:
            if self.rgb_frame is not None:
                frame_to_save = self.rgb_frame.copy()
        
        if frame_to_save is not None:
            ts = time.strftime("%Y%m%d_%H%M%S")
            fname = f"sample_{self.photo_count + 1}_{ts}.jpg"
            path = os.path.join(self.save_dir, fname)
            cv2.imwrite(path, frame_to_save)
            self.get_logger().info(f" Kayıt: {fname}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = RoverMaster()
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    except Exception as e: print(f"KRİTİK: {e}")
    finally:
        if 'node' in locals(): node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
