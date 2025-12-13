#!/usr/bin/env python3
"""
Marmara Rover - Advanced 3-Camera Control Station
3 Kamera + Telemetri + Harita + Grafikler + Video Kayıt
- Ön Kamera: Intel RealSense (RGB + Depth)
- Arka Kamera: Logitech C270
"""
import sys
import json
import time
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, 
    QGridLayout, QVBoxLayout, QHBoxLayout, QFrame, QTabWidget,
    QTextEdit, QProgressBar, QGroupBox, QSplitter, QComboBox,
    QSlider, QCheckBox
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QPropertyAnimation, QEasingCurve
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor, QFont, QPen, QBrush, QLinearGradient

import zmq
import cv2
import numpy as np

# Grafik kütüphaneleri
try:
    import pyqtgraph as pg
    PYQTGRAPH_AVAILABLE = True
except ImportError:
    PYQTGRAPH_AVAILABLE = False
    print("[WARNING] pyqtgraph bulunamadı. Grafik özellikleri devre dışı.")

# ----------------- KONFİG -----------------

PC_CMD_PORT = 5555
JETSON_IP = "192.168.1.131"

# ZMQ Portları (zmq_bridge_node.py ile uyumlu)
JETSON_LOGITECH_PORT = 6000      # Arka kamera (Logitech)
JETSON_TELEMETRY_PORT = 6001     # Telemetri JSON
JETSON_REALSENSE_RGB_PORT = 6002 # Ön kamera RGB (RealSense)
JETSON_REALSENSE_DEPTH_PORT = 6003 # Ön kamera Depth (RealSense)
JETSON_HEALTH_PORT = 6004        # Health status

# Telemetri veri buffer boyutu
TELEMETRY_BUFFER_SIZE = 100

# Renkler ve Tema
THEME = {
    'bg_dark': '#1a1a2e',
    'bg_medium': '#16213e',
    'bg_light': '#0f3460',
    'accent': '#e94560',
    'accent_green': '#00ff88',
    'accent_blue': '#00d9ff',
    'accent_yellow': '#ffd700',
    'accent_purple': '#b084cc',
    'text_primary': '#ffffff',
    'text_secondary': '#a0a0a0',
    'border': '#2a2a3e',
    'success': '#00ff88',
    'warning': '#ffd700',
    'error': '#e94560',
}


# ----------------- KAMERA ALICI THREAD'LERİ -----------------

class CameraReceiver(QThread):
    """Genel kamera receiver thread - RGB görüntüleri için"""
    frame_received = pyqtSignal(np.ndarray)
    fps_updated = pyqtSignal(float)
    
    def __init__(self, jetson_ip: str, port: int, camera_name: str, parent=None):
        super().__init__(parent)
        self.jetson_ip = jetson_ip
        self.port = port
        self.camera_name = camera_name
        self._running = True
        self.frame_count = 0
        self.last_fps_time = time.time()

    def run(self):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        connect_addr = f"tcp://{self.jetson_ip}:{self.port}"
        print(f"[INFO] {self.camera_name} ZMQ SUB bağlanıyor -> {connect_addr}")
        socket.connect(connect_addr)
        socket.setsockopt_string(zmq.SUBSCRIBE, "")
        socket.RCVTIMEO = 1000

        while self._running:
            try:
                parts = socket.recv_multipart()
            except zmq.Again:
                continue
            except Exception as e:
                print(f"[ERROR] {self.camera_name} ZMQ recv hata: {e}")
                break

            if not parts or len(parts) < 2:
                continue

            jpg_bytes = parts[1]
            np_arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                self.frame_received.emit(frame)
                
                # FPS hesapla
                self.frame_count += 1
                current_time = time.time()
                elapsed = current_time - self.last_fps_time
                if elapsed >= 1.0:
                    fps = self.frame_count / elapsed
                    self.fps_updated.emit(fps)
                    self.frame_count = 0
                    self.last_fps_time = current_time

        socket.close(0)
        context.term()
        print(f"[INFO] {self.camera_name} thread kapandı.")

    def stop(self):
        self._running = False


# ----------------- TELEMETRİ ALICI THREAD -----------------

class TelemetryReceiver(QThread):
    telemetry_received = pyqtSignal(dict)
    
    def __init__(self, jetson_ip: str, port: int, parent=None):
        super().__init__(parent)
        self.jetson_ip = jetson_ip
        self.port = port
        self._running = True

    def run(self):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        connect_addr = f"tcp://{self.jetson_ip}:{self.port}"
        print(f"[INFO] Telemetri ZMQ SUB bağlanıyor -> {connect_addr}")
        socket.connect(connect_addr)
        socket.setsockopt_string(zmq.SUBSCRIBE, "")
        socket.RCVTIMEO = 1000

        while self._running:
            try:
                msg = socket.recv_string()
            except zmq.Again:
                continue
            except Exception as e:
                print(f"[ERROR] Telemetri ZMQ recv hata: {e}")
                break

            try:
                data = json.loads(msg)
                self.telemetry_received.emit(data)
            except json.JSONDecodeError:
                continue

        socket.close(0)
        context.term()
        print("[INFO] Telemetri thread kapandı.")

    def stop(self):
        self._running = False


# ----------------- HEALTH STATUS ALICI THREAD -----------------

class HealthReceiver(QThread):
    health_received = pyqtSignal(dict)
    
    def __init__(self, jetson_ip: str, port: int, parent=None):
        super().__init__(parent)
        self.jetson_ip = jetson_ip
        self.port = port
        self._running = True

    def run(self):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        connect_addr = f"tcp://{self.jetson_ip}:{self.port}"
        print(f"[INFO] Health Status ZMQ SUB bağlanıyor -> {connect_addr}")
        socket.connect(connect_addr)
        socket.setsockopt_string(zmq.SUBSCRIBE, "")
        socket.RCVTIMEO = 1000

        while self._running:
            try:
                msg = socket.recv_string()
            except zmq.Again:
                continue
            except Exception as e:
                print(f"[ERROR] Health Status ZMQ recv hata: {e}")
                break

            try:
                data = json.loads(msg)
                self.health_received.emit(data)
            except json.JSONDecodeError:
                continue

        socket.close(0)
        context.term()
        print("[INFO] Health Status thread kapandı.")

    def stop(self):
        self._running = False


# ----------------- ÖZEL WİDGET'LAR -----------------

class ModernCard(QFrame):
    """Modern kart container widget"""
    def __init__(self, title="", parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {THEME['bg_medium']};
                border: 1px solid {THEME['border']};
                border-radius: 10px;
                padding: 10px;
            }}
        """)
        
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        if title:
            title_label = QLabel(title)
            title_label.setStyleSheet(f"""
                font-size: 14px;
                font-weight: bold;
                color: {THEME['accent_blue']};
                padding: 5px;
            """)
            layout.addWidget(title_label)
        
        self.content_layout = QVBoxLayout()
        layout.addLayout(self.content_layout)


class CameraWidget(QFrame):
    """Gelişmiş kamera widget - Overlay bilgileri ile"""
    def __init__(self, camera_name: str, camera_type: str, parent=None):
        super().__init__(parent)
        self.camera_name = camera_name
        self.camera_type = camera_type  # "rgb" or "depth"
        self.current_fps = 0
        self.is_recording = False
        
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {THEME['bg_dark']};
                border: 2px solid {THEME['border']};
                border-radius: 8px;
            }}
        """)
        
        layout = QVBoxLayout()
        self.setLayout(layout)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # Başlık
        header = QHBoxLayout()
        self.title_label = QLabel(f"📹 {camera_name}")
        self.title_label.setStyleSheet(f"""
            font-size: 12px;
            font-weight: bold;
            color: {THEME['accent_blue']};
            padding: 3px;
        """)
        header.addWidget(self.title_label)
        
        header.addStretch()
        
        # FPS göstergesi
        self.fps_label = QLabel("FPS: 0")
        self.fps_label.setStyleSheet(f"color: {THEME['accent_green']}; font-size: 10px;")
        header.addWidget(self.fps_label)
        
        # Kayıt göstergesi
        self.rec_label = QLabel("")
        self.rec_label.setStyleSheet(f"color: {THEME['error']}; font-size: 10px; font-weight: bold;")
        header.addWidget(self.rec_label)
        
        layout.addLayout(header)
        
        # Kamera görüntüsü
        self.camera_label = QLabel()
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setStyleSheet(f"background-color: {THEME['bg_dark']}; border: none;")
        self.camera_label.setMinimumSize(400, 300)
        layout.addWidget(self.camera_label)
        
    def update_frame(self, frame: np.ndarray):
        """Frame'i güncelle"""
        if frame is None:
            return
        
        # FPS bilgisini frame üzerine yaz
        cv2.putText(frame, f"{self.camera_name} | FPS: {self.current_fps:.1f}", 
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        if self.is_recording:
            # Kayıt göstergesi
            cv2.circle(frame, (frame.shape[1] - 30, 25), 8, (0, 0, 255), -1)
            cv2.putText(frame, "REC", (frame.shape[1] - 70, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # BGR -> RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        h, w, ch = frame_rgb.shape
        bytes_per_line = ch * w
        qimg = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        
        self.camera_label.setPixmap(
            pixmap.scaled(self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )
    
    def update_fps(self, fps: float):
        """FPS'i güncelle"""
        self.current_fps = fps
        self.fps_label.setText(f"FPS: {fps:.1f}")
    
    def set_recording(self, recording: bool):
        """Kayıt durumunu ayarla"""
        self.is_recording = recording
        if recording:
            self.rec_label.setText("⏺️ REC")
        else:
            self.rec_label.setText("")


class GaugeWidget(QWidget):
    """Gauge gösterge widget"""
    def __init__(self, min_val=0, max_val=100, unit="", parent=None):
        super().__init__(parent)
        self.min_val = min_val
        self.max_val = max_val
        self.unit = unit
        self.value = 0
        self.setMinimumSize(120, 120)
        
    def setValue(self, value):
        self.value = max(self.min_val, min(self.max_val, value))
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        radius = min(w, h) // 2 - 20
        
        # Arka plan çemberi
        painter.setPen(QPen(QColor(THEME['border']), 6))
        painter.drawArc(cx - radius, cy - radius, radius * 2, radius * 2, 0, 360 * 16)
        
        # Değer yayı
        span_angle = int(((self.value - self.min_val) / (self.max_val - self.min_val)) * 360 * 16)
        
        gradient = QLinearGradient(0, 0, w, h)
        gradient.setColorAt(0, QColor(THEME['accent_green']))
        gradient.setColorAt(0.5, QColor(THEME['accent_yellow']))
        gradient.setColorAt(1, QColor(THEME['accent']))
        
        painter.setPen(QPen(QBrush(gradient), 6))
        painter.drawArc(cx - radius, cy - radius, radius * 2, radius * 2, 90 * 16, -span_angle)
        
        # Değer metni
        painter.setPen(QColor(THEME['text_primary']))
        font = QFont("Arial", 16, QFont.Bold)
        painter.setFont(font)
        text = f"{self.value:.1f}{self.unit}"
        painter.drawText(self.rect(), Qt.AlignCenter, text)


class StatusIndicator(QWidget):
    """Durum gösterge LED widget"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.is_active = False
        self.setFixedSize(16, 16)
        
    def setActive(self, active):
        self.is_active = active
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        color = QColor(THEME['success']) if self.is_active else QColor(THEME['text_secondary'])
        painter.setBrush(QBrush(color))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(2, 2, 12, 12)


# ----------------- ANA ARAYÜZ -----------------

class RoverControlStation(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle("🚀 Marmara Rover - 3-Camera Control Station")
        self.ros_node = ros_node
        
        # ROS publishers
        self.pub_connection_test = self.ros_node.create_publisher(String, "/pc/connection_test", 10)
        self.pub_teleop_key = self.ros_node.create_publisher(String, "/pc/teleop_key", 10)
        
        # ZMQ Komut Socket
        self.zmq_cmd_context = zmq.Context()
        self.zmq_cmd_socket = self.zmq_cmd_context.socket(zmq.PUB)
        self.zmq_cmd_socket.bind(f"tcp://*:{PC_CMD_PORT}")
        
        # Durum değişkenleri
        self.movement_enabled = False
        self.connected = False
        self.recording_cameras = {
            'logitech': False,
            'realsense_rgb': False,
            'realsense_depth': False
        }
        
        # Video writers
        self.video_writers = {
            'logitech': None,
            'realsense_rgb': None,
            'realsense_depth': None
        }
        
        # Current frames (video kayıt için)
        self.current_frames = {
            'logitech': None,
            'realsense_rgb': None,
            'realsense_depth': None
        }
        
        # Telemetri data buffers
        self.telemetry_buffer = {
            'temp': deque(maxlen=TELEMETRY_BUFFER_SIZE),
            'humidity': deque(maxlen=TELEMETRY_BUFFER_SIZE),
            'pressure': deque(maxlen=TELEMETRY_BUFFER_SIZE),
            'roll': deque(maxlen=TELEMETRY_BUFFER_SIZE),
            'pitch': deque(maxlen=TELEMETRY_BUFFER_SIZE),
            'yaw': deque(maxlen=TELEMETRY_BUFFER_SIZE),
            'timestamps': deque(maxlen=TELEMETRY_BUFFER_SIZE),
        }
        
        # Health status
        self.health_status = {}
        
        # Threads - 3 Kamera + Telemetri + Health
        self.logitech_thread = CameraReceiver(JETSON_IP, JETSON_LOGITECH_PORT, "Logitech (Arka)")
        self.logitech_thread.frame_received.connect(lambda f: self.on_camera_frame(f, 'logitech'))
        self.logitech_thread.fps_updated.connect(lambda fps: self.on_fps_update(fps, 'logitech'))
        self.logitech_thread.start()
        
        self.realsense_rgb_thread = CameraReceiver(JETSON_IP, JETSON_REALSENSE_RGB_PORT, "RealSense RGB (Ön)")
        self.realsense_rgb_thread.frame_received.connect(lambda f: self.on_camera_frame(f, 'realsense_rgb'))
        self.realsense_rgb_thread.fps_updated.connect(lambda fps: self.on_fps_update(fps, 'realsense_rgb'))
        self.realsense_rgb_thread.start()
        
        self.realsense_depth_thread = CameraReceiver(JETSON_IP, JETSON_REALSENSE_DEPTH_PORT, "RealSense Depth (Ön)")
        self.realsense_depth_thread.frame_received.connect(lambda f: self.on_camera_frame(f, 'realsense_depth'))
        self.realsense_depth_thread.fps_updated.connect(lambda fps: self.on_fps_update(fps, 'realsense_depth'))
        self.realsense_depth_thread.start()
        
        self.telemetry_thread = TelemetryReceiver(JETSON_IP, JETSON_TELEMETRY_PORT)
        self.telemetry_thread.telemetry_received.connect(self.on_telemetry)
        self.telemetry_thread.start()
        
        self.health_thread = HealthReceiver(JETSON_IP, JETSON_HEALTH_PORT)
        self.health_thread.health_received.connect(self.on_health_status)
        self.health_thread.start()
        
        # UI oluştur
        self._build_ui()
        
        # Stil uygula
        self._apply_theme()
        
        # Bağlantı durumu güncelleyici
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._update_connection_status)
        self.status_timer.start(1000)
        
        self.add_log("🚀 Sistem başlatıldı. 3 Kamera + Telemetri aktif...")

    def _build_ui(self):
        """Ana UI yapısını oluştur"""
        central = QWidget()
        self.setCentralWidget(central)
        
        main_layout = QVBoxLayout()
        central.setLayout(main_layout)
        
        # === ÜST BAR: Başlık ve Durum ===
        header = self._create_header()
        main_layout.addWidget(header)
        
        # === ANA İÇERİK: Sekme Widget ===
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet(f"""
            QTabWidget::pane {{
                border: 1px solid {THEME['border']};
                background: {THEME['bg_dark']};
                border-radius: 5px;
            }}
            QTabBar::tab {{
                background: {THEME['bg_medium']};
                color: {THEME['text_secondary']};
                padding: 10px 20px;
                margin: 2px;
                border-radius: 5px;
            }}
            QTabBar::tab:selected {{
                background: {THEME['accent']};
                color: {THEME['text_primary']};
            }}
        """)
        
        # Sekmeler
        self.tabs.addTab(self._create_cameras_tab(), "📹 Kamera Görüntüleri")
        self.tabs.addTab(self._create_main_tab(), "🎮 Kontrol & Telemetri")
        self.tabs.addTab(self._create_telemetry_tab(), "📊 Grafikler & Sensörler")
        self.tabs.addTab(self._create_logs_tab(), "📝 Sistem Logları")
        
        main_layout.addWidget(self.tabs)
        
        # Pencere boyutu
        self.resize(1600, 1000)

    def _create_header(self):
        """Üst başlık barı"""
        header = QFrame()
        header.setFixedHeight(80)
        header.setStyleSheet(f"""
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                stop:0 {THEME['bg_medium']}, stop:1 {THEME['bg_light']});
            border-bottom: 2px solid {THEME['accent']};
            border-radius: 5px;
        """)
        
        layout = QHBoxLayout()
        header.setLayout(layout)
        
        # Logo ve başlık
        title = QLabel("🚀 MARMARA ROVER - 3 CAMERA CONTROL STATION")
        title.setStyleSheet(f"""
            font-size: 22px;
            font-weight: bold;
            color: {THEME['accent']};
            padding: 10px;
        """)
        layout.addWidget(title)
        
        layout.addStretch()
        
        # Health status widget
        health_widget = QWidget()
        health_layout = QHBoxLayout()
        health_widget.setLayout(health_layout)
        
        # Kamera durumları
        self.camera_indicators = {}
        for cam_name, display_name in [
            ('logitech', '📷 Arka'),
            ('realsense_rgb', '📷 Ön RGB'),
            ('realsense_depth', '📷 Ön Depth')
        ]:
            indicator = StatusIndicator()
            label = QLabel(display_name)
            label.setStyleSheet(f"color: {THEME['text_secondary']}; font-size: 10px;")
            
            cam_layout = QVBoxLayout()
            cam_layout.addWidget(indicator, alignment=Qt.AlignCenter)
            cam_layout.addWidget(label)
            cam_layout.setSpacing(2)
            
            cam_widget = QWidget()
            cam_widget.setLayout(cam_layout)
            health_layout.addWidget(cam_widget)
            
            self.camera_indicators[cam_name] = indicator
        
        layout.addWidget(health_widget)
        
        # Bağlantı durumu
        status_widget = QWidget()
        status_layout = QHBoxLayout()
        status_widget.setLayout(status_layout)
        
        self.status_indicator = StatusIndicator()
        status_layout.addWidget(self.status_indicator)
        
        self.status_label = QLabel("Bağlantı Bekleniyor...")
        self.status_label.setStyleSheet(f"color: {THEME['text_secondary']}; font-size: 11px;")
        status_layout.addWidget(self.status_label)
        
        layout.addWidget(status_widget)
        
        return header

    def _create_cameras_tab(self):
        """Kamera görüntüleri sekmesi - 3 kamera grid"""
        tab = QWidget()
        layout = QGridLayout()
        tab.setLayout(layout)
        
        # Kamera widget'ları oluştur
        self.camera_widgets = {}
        
        # Üst satır: Ön kameralar (RealSense RGB + Depth)
        self.camera_widgets['realsense_rgb'] = CameraWidget("RealSense RGB (Ön Kamera)", "rgb")
        self.camera_widgets['realsense_depth'] = CameraWidget("RealSense Depth (Ön Kamera)", "depth")
        
        layout.addWidget(self.camera_widgets['realsense_rgb'], 0, 0)
        layout.addWidget(self.camera_widgets['realsense_depth'], 0, 1)
        
        # Alt satır: Arka kamera (Logitech) - tam genişlik
        self.camera_widgets['logitech'] = CameraWidget("Logitech C270 (Arka Kamera)", "rgb")
        layout.addWidget(self.camera_widgets['logitech'], 1, 0, 1, 2)
        
        # Kayıt kontrolleri
        record_card = ModernCard("🎥 Video Kayıt Kontrolü")
        record_layout = QHBoxLayout()
        
        self.record_buttons = {}
        for cam_name, display_name in [
            ('logitech', 'Arka Kamera'),
            ('realsense_rgb', 'Ön RGB'),
            ('realsense_depth', 'Ön Depth')
        ]:
            btn = QPushButton(f"🎬 {display_name}")
            btn.setCheckable(True)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {THEME['bg_light']};
                    color: {THEME['text_primary']};
                    border: 2px solid {THEME['border']};
                    border-radius: 6px;
                    padding: 10px;
                    font-size: 11px;
                }}
                QPushButton:checked {{
                    background-color: {THEME['error']};
                    color: white;
                    border-color: {THEME['error']};
                }}
            """)
            btn.clicked.connect(lambda checked, c=cam_name: self.on_record_camera_clicked(c, checked))
            record_layout.addWidget(btn)
            self.record_buttons[cam_name] = btn
        
        # Hepsini kaydet butonu
        btn_record_all = QPushButton("⏺️ HEPSINI KAYDET")
        btn_record_all.setCheckable(True)
        btn_record_all.setStyleSheet(f"""
            QPushButton {{
                background-color: {THEME['accent']};
                color: white;
                border: none;
                border-radius: 6px;
                padding: 12px;
                font-size: 12px;
                font-weight: bold;
            }}
            QPushButton:checked {{
                background-color: {THEME['error']};
            }}
        """)
        btn_record_all.clicked.connect(self.on_record_all_clicked)
        record_layout.addWidget(btn_record_all)
        self.btn_record_all = btn_record_all
        
        record_card.content_layout.addLayout(record_layout)
        layout.addWidget(record_card, 2, 0, 1, 2)
        
        return tab

    def _create_main_tab(self):
        """Kontrol ve hızlı telemetri sekmesi"""
        tab = QWidget()
        layout = QHBoxLayout()
        tab.setLayout(layout)
        
        # SOL PANEL: Kontroller
        left_panel = QWidget()
        left_layout = QVBoxLayout()
        left_panel.setLayout(left_layout)
        left_panel.setMaximumWidth(350)
        
        # Bağlantı kartı
        connection_card = ModernCard("🔌 Bağlantı")
        self.btn_connect = QPushButton("ROVERA BAĞLAN")
        self.btn_connect.setStyleSheet(f"""
            QPushButton {{
                background-color: {THEME['accent']};
                color: white;
                border: none;
                border-radius: 8px;
                padding: 15px;
                font-size: 14px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: {THEME['accent_blue']};
            }}
        """)
        self.btn_connect.clicked.connect(self.on_connect_clicked)
        connection_card.content_layout.addWidget(self.btn_connect)
        left_layout.addWidget(connection_card)
        
        # Hareket kontrol kartı
        movement_card = ModernCard("🎮 Hareket Kontrolü")
        self.btn_move = QPushButton("HAREKETİ AKTİF ET")
        self.btn_move.setCheckable(True)
        self.btn_move.setStyleSheet(f"""
            QPushButton {{
                background-color: {THEME['bg_light']};
                color: {THEME['text_secondary']};
                border: 2px solid {THEME['border']};
                border-radius: 8px;
                padding: 15px;
                font-size: 14px;
                font-weight: bold;
            }}
            QPushButton:checked {{
                background-color: {THEME['success']};
                color: white;
                border-color: {THEME['success']};
            }}
        """)
        self.btn_move.clicked.connect(self.on_move_clicked)
        movement_card.content_layout.addWidget(self.btn_move)
        
        keyboard_info = QLabel("⌨️ WASD ile kontrol")
        keyboard_info.setStyleSheet(f"color: {THEME['text_secondary']}; font-size: 11px; padding: 5px;")
        keyboard_info.setAlignment(Qt.AlignCenter)
        movement_card.content_layout.addWidget(keyboard_info)
        
        left_layout.addWidget(movement_card)
        
        # Hızlı telemetri kartı
        quick_telemetry_card = ModernCard("📡 Hızlı Bakış Telemetri")
        self.quick_gps = QLabel("GPS: -")
        self.quick_temp = QLabel("Sıcaklık: -")
        self.quick_humidity = QLabel("Nem: -")
        self.quick_imu = QLabel("IMU: -")
        
        for lbl in [self.quick_gps, self.quick_temp, self.quick_humidity, self.quick_imu]:
            lbl.setStyleSheet(f"color: {THEME['text_primary']}; font-size: 11px; padding: 3px;")
            quick_telemetry_card.content_layout.addWidget(lbl)
        
        left_layout.addWidget(quick_telemetry_card)
        
        # System health
        health_card = ModernCard("💚 Sistem Sağlığı")
        self.health_text = QTextEdit()
        self.health_text.setReadOnly(True)
        self.health_text.setMaximumHeight(150)
        self.health_text.setStyleSheet(f"""
            QTextEdit {{
                background-color: {THEME['bg_dark']};
                color: {THEME['accent_green']};
                border: 1px solid {THEME['border']};
                border-radius: 5px;
                padding: 5px;
                font-family: 'Courier New', monospace;
                font-size: 10px;
            }}
        """)
        health_card.content_layout.addWidget(self.health_text)
        left_layout.addWidget(health_card)
        
        left_layout.addStretch()
        
        layout.addWidget(left_panel)
        
        # SAĞ PANEL: Kamera önizlemeleri
        right_panel = ModernCard("📹 Kamera Önizleme (Kontrol Sekmesi)")
        right_layout = QVBoxLayout()
        
        # Mini kamera görüntüleri
        mini_cam_layout = QHBoxLayout()
        self.mini_camera_labels = {}
        
        for cam_name, display_name in [
            ('logitech', 'Arka'),
            ('realsense_rgb', 'Ön RGB'),
            ('realsense_depth', 'Ön Depth')
        ]:
            cam_container = QWidget()
            cam_vlayout = QVBoxLayout()
            cam_container.setLayout(cam_vlayout)
            
            title = QLabel(display_name)
            title.setAlignment(Qt.AlignCenter)
            title.setStyleSheet(f"color: {THEME['accent_blue']}; font-size: 10px;")
            cam_vlayout.addWidget(title)
            
            label = QLabel()
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet(f"background-color: {THEME['bg_dark']}; border: 1px solid {THEME['border']};")
            label.setMinimumSize(250, 180)
            cam_vlayout.addWidget(label)
            
            mini_cam_layout.addWidget(cam_container)
            self.mini_camera_labels[cam_name] = label
        
        right_layout.addLayout(mini_cam_layout)
        right_panel.content_layout.addLayout(right_layout)
        
        layout.addWidget(right_panel, 1)
        
        return tab

    def _create_telemetry_tab(self):
        """Telemetri ve grafikler sekmesi"""
        tab = QWidget()
        layout = QGridLayout()
        tab.setLayout(layout)
        
        # GPS Kartı
        gps_card = ModernCard("🌍 GPS Konum")
        self.lbl_gps_lat = QLabel("Enlem: -")
        self.lbl_gps_lon = QLabel("Boylam: -")
        self.lbl_gps_alt = QLabel("Rakım: -")
        for lbl in [self.lbl_gps_lat, self.lbl_gps_lon, self.lbl_gps_alt]:
            lbl.setStyleSheet(f"color: {THEME['text_primary']}; font-size: 13px; padding: 5px;")
            gps_card.content_layout.addWidget(lbl)
        
        self.map_label = QLabel("🗺️ Harita Görünümü\n(GPS verisi geldiğinde aktif olacak)")
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setStyleSheet(f"""
            background-color: {THEME['bg_dark']};
            color: {THEME['text_secondary']};
            border: 1px dashed {THEME['border']};
            padding: 30px;
            border-radius: 5px;
        """)
        self.map_label.setMinimumHeight(180)
        gps_card.content_layout.addWidget(self.map_label)
        
        layout.addWidget(gps_card, 0, 0)
        
        # IMU Gauge'ler
        imu_card = ModernCard("🧭 IMU Sensörü")
        imu_layout = QHBoxLayout()
        
        self.gauge_roll = GaugeWidget(-180, 180, "°")
        self.gauge_pitch = GaugeWidget(-180, 180, "°")
        self.gauge_yaw = GaugeWidget(0, 360, "°")
        
        for gauge, name in [(self.gauge_roll, "Roll"), (self.gauge_pitch, "Pitch"), (self.gauge_yaw, "Yaw")]:
            container = QWidget()
            vlayout = QVBoxLayout()
            container.setLayout(vlayout)
            lbl = QLabel(name)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(f"color: {THEME['accent_blue']}; font-size: 11px;")
            vlayout.addWidget(lbl)
            vlayout.addWidget(gauge)
            imu_layout.addWidget(container)
        
        imu_card.content_layout.addLayout(imu_layout)
        layout.addWidget(imu_card, 0, 1)
        
        # Çevre Sensörleri
        env_card = ModernCard("🌡️ Çevre Sensörleri")
        env_layout = QHBoxLayout()
        
        self.gauge_temp = GaugeWidget(-20, 60, "°C")
        self.gauge_humidity = GaugeWidget(0, 100, "%")
        self.gauge_pressure = GaugeWidget(900, 1100, "hPa")
        
        for gauge, name in [(self.gauge_temp, "Sıcaklık"), (self.gauge_humidity, "Nem"), (self.gauge_pressure, "Basınç")]:
            container = QWidget()
            vlayout = QVBoxLayout()
            container.setLayout(vlayout)
            lbl = QLabel(name)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(f"color: {THEME['accent_green']}; font-size: 11px;")
            vlayout.addWidget(lbl)
            vlayout.addWidget(gauge)
            env_layout.addWidget(container)
        
        env_card.content_layout.addLayout(env_layout)
        layout.addWidget(env_card, 1, 0, 1, 2)
        
        # Grafikler
        if PYQTGRAPH_AVAILABLE:
            graph_card = ModernCard("📈 Telemetri Grafikleri")
            
            self.graph_widget = pg.PlotWidget()
            self.graph_widget.setBackground(THEME['bg_dark'])
            self.graph_widget.showGrid(x=True, y=True, alpha=0.3)
            self.graph_widget.setLabel('left', 'Değer')
            self.graph_widget.setLabel('bottom', 'Zaman (s)')
            
            self.plot_temp = self.graph_widget.plot(pen=pg.mkPen(color=THEME['accent'], width=2), name='Sıcaklık')
            self.plot_humidity = self.graph_widget.plot(pen=pg.mkPen(color=THEME['accent_blue'], width=2), name='Nem')
            
            graph_card.content_layout.addWidget(self.graph_widget)
            layout.addWidget(graph_card, 2, 0, 1, 2)
        
        return tab

    def _create_logs_tab(self):
        """Sistem logları sekmesi"""
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)
        
        log_card = ModernCard("📝 Sistem Logları")
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet(f"""
            QTextEdit {{
                background-color: {THEME['bg_dark']};
                color: {THEME['accent_green']};
                border: 1px solid {THEME['border']};
                border-radius: 5px;
                padding: 10px;
                font-family: 'Courier New', monospace;
                font-size: 11px;
            }}
        """)
        
        log_card.content_layout.addWidget(self.log_text)
        
        btn_clear = QPushButton("Logları Temizle")
        btn_clear.clicked.connect(lambda: self.log_text.clear())
        btn_clear.setStyleSheet(f"""
            QPushButton {{
                background-color: {THEME['bg_light']};
                color: {THEME['text_primary']};
                border: none;
                border-radius: 5px;
                padding: 8px;
            }}
            QPushButton:hover {{
                background-color: {THEME['accent']};
            }}
        """)
        log_card.content_layout.addWidget(btn_clear)
        
        layout.addWidget(log_card)
        
        return tab

    def _apply_theme(self):
        """Global tema stilleri"""
        self.setStyleSheet(f"""
            QMainWindow {{
                background-color: {THEME['bg_dark']};
            }}
            QWidget {{
                color: {THEME['text_primary']};
                font-family: 'Segoe UI', Arial, sans-serif;
            }}
        """)

    # ========== EVENT HANDLERS ==========

    def on_connect_clicked(self):
        """Bağlantı butonuna tıklandı"""
        msg = String()
        msg.data = "ping_from_pc_ui"
        self.pub_connection_test.publish(msg)
        
        payload = {
            "type": "ping",
            "source": "pc_ui_3cam",
            "timestamp": time.time(),
        }
        try:
            self.zmq_cmd_socket.send_string(json.dumps(payload))
            self.connected = True
            self.status_indicator.setActive(True)
            self.status_label.setText("✅ Bağlandı")
            self.status_label.setStyleSheet(f"color: {THEME['success']}; font-size: 11px;")
            self.add_log("[SUCCESS] Rover ile bağlantı kuruldu (3 kamera aktif)")
        except Exception as e:
            self.connected = False
            self.status_indicator.setActive(False)
            self.status_label.setText(f"❌ Bağlantı Hatası")
            self.status_label.setStyleSheet(f"color: {THEME['error']}; font-size: 11px;")
            self.add_log(f"[ERROR] Bağlantı hatası: {e}")

    def on_move_clicked(self, checked):
        """Hareket butonuna tıklandı"""
        self.movement_enabled = checked
        if checked:
            self.btn_move.setText("✅ HAREKET AKTİF (WASD)")
            self.add_log("[INFO] Hareket kontrolü aktif edildi")
        else:
            self.btn_move.setText("HAREKETİ AKTİF ET")
            self.add_log("[INFO] Hareket kontrolü devre dışı")

    def on_record_camera_clicked(self, camera_name: str, checked: bool):
        """Tek kamera kayıt butonuna tıklandı"""
        self.recording_cameras[camera_name] = checked
        
        if checked:
            # Video kayıt başlat
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"/mnt/user-data/outputs/rover_{camera_name}_{timestamp}.avi"
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            
            # Frame boyutunu ayarla
            if camera_name == 'logitech':
                size = (640, 480)
            else:  # RealSense
                size = (848, 480)
            
            self.video_writers[camera_name] = cv2.VideoWriter(filename, fourcc, 20.0, size)
            self.camera_widgets[camera_name].set_recording(True)
            self.add_log(f"[INFO] {camera_name} video kaydı başlatıldı: {filename}")
        else:
            # Video kayıt durdur
            if self.video_writers[camera_name]:
                self.video_writers[camera_name].release()
                self.video_writers[camera_name] = None
            self.camera_widgets[camera_name].set_recording(False)
            self.add_log(f"[INFO] {camera_name} video kaydı durduruldu")

    def on_record_all_clicked(self, checked: bool):
        """Tüm kameraları kaydet butonuna tıklandı"""
        for camera_name, btn in self.record_buttons.items():
            btn.setChecked(checked)
            self.on_record_camera_clicked(camera_name, checked)

    def keyPressEvent(self, event):
        """Klavye tuşlarına basıldı"""
        if not self.movement_enabled:
            return
        
        key = event.key()
        key_map = {
            Qt.Key_W: "w",
            Qt.Key_A: "a",
            Qt.Key_S: "s",
            Qt.Key_D: "d",
        }
        
        if key in key_map:
            key_char = key_map[key]
            self.send_teleop_key(key_char)

    def send_teleop_key(self, key_char: str):
        """Hareket komutu gönder"""
        msg = String()
        msg.data = key_char
        self.pub_teleop_key.publish(msg)
        
        payload = {
            "type": "teleop_key",
            "key": key_char,
            "timestamp": time.time(),
        }
        try:
            self.zmq_cmd_socket.send_string(json.dumps(payload))
            self.add_log(f"[CMD] Hareket: {key_char.upper()}")
        except Exception as e:
            self.add_log(f"[ERROR] Komut gönderilemedi: {e}")

    def on_camera_frame(self, frame_bgr: np.ndarray, camera_name: str):
        """Kamera frame'i geldi"""
        if frame_bgr is None:
            return
        
        # Current frame'i sakla (video kayıt için)
        self.current_frames[camera_name] = frame_bgr.copy()
        
        # Video kayıt
        if self.recording_cameras[camera_name] and self.video_writers[camera_name]:
            self.video_writers[camera_name].write(frame_bgr)
        
        # Ana kamera widget'ına göster
        if camera_name in self.camera_widgets:
            self.camera_widgets[camera_name].update_frame(frame_bgr)
        
        # Mini önizlemeye de göster (kontrol sekmesinde)
        if camera_name in self.mini_camera_labels:
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            h, w, ch = frame_rgb.shape
            bytes_per_line = ch * w
            qimg = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)
            self.mini_camera_labels[camera_name].setPixmap(
                pixmap.scaled(self.mini_camera_labels[camera_name].size(), 
                             Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )

    def on_fps_update(self, fps: float, camera_name: str):
        """FPS güncellemesi"""
        if camera_name in self.camera_widgets:
            self.camera_widgets[camera_name].update_fps(fps)

    def on_telemetry(self, data: dict):
        """Telemetri verisi geldi"""
        timestamp = time.time()
        
        gps = data.get("gps", {})
        imu = data.get("imu", {})
        env = data.get("env", {})
        
        # GPS
        lat = gps.get("lat")
        lon = gps.get("lon")
        alt = gps.get("alt")
        
        if lat is not None and lon is not None:
            self.lbl_gps_lat.setText(f"Enlem: {lat:.6f}°")
            self.lbl_gps_lon.setText(f"Boylam: {lon:.6f}°")
            self.lbl_gps_alt.setText(f"Rakım: {alt if alt else 0:.1f} m")
            self.quick_gps.setText(f"GPS: {lat:.4f}, {lon:.4f}")
        
        # IMU
        roll = imu.get("roll")
        pitch = imu.get("pitch")
        yaw = imu.get("yaw")
        
        if roll is not None:
            self.gauge_roll.setValue(roll)
        if pitch is not None:
            self.gauge_pitch.setValue(pitch)
        if yaw is not None:
            self.gauge_yaw.setValue(yaw)
        if roll is not None and pitch is not None and yaw is not None:
            self.quick_imu.setText(f"IMU: R{roll:.0f}° P{pitch:.0f}° Y{yaw:.0f}°")
        
        # ENV
        temp_c = env.get("temp_c")
        humidity = env.get("humidity")
        pressure = env.get("pressure_hpa")
        
        if temp_c is not None:
            self.gauge_temp.setValue(temp_c)
            self.telemetry_buffer['temp'].append(temp_c)
            self.quick_temp.setText(f"Sıcaklık: {temp_c:.1f}°C")
        
        if humidity is not None:
            self.gauge_humidity.setValue(humidity)
            self.telemetry_buffer['humidity'].append(humidity)
            self.quick_humidity.setText(f"Nem: {humidity:.1f}%")
        
        if pressure is not None:
            self.gauge_pressure.setValue(pressure)
            self.telemetry_buffer['pressure'].append(pressure)
        
        self.telemetry_buffer['timestamps'].append(timestamp)
        
        # Grafikleri güncelle
        if PYQTGRAPH_AVAILABLE and len(self.telemetry_buffer['timestamps']) > 0:
            times = list(self.telemetry_buffer['timestamps'])
            times = [t - times[0] for t in times]
            
            if len(self.telemetry_buffer['temp']) > 0:
                self.plot_temp.setData(times, list(self.telemetry_buffer['temp']))
            
            if len(self.telemetry_buffer['humidity']) > 0:
                self.plot_humidity.setData(times, list(self.telemetry_buffer['humidity']))

    def on_health_status(self, data: dict):
        """Health status verisi geldi"""
        self.health_status = data
        
        streams = data.get('streams', {})
        
        # Kamera göstergelerini güncelle
        for cam_name in ['logitech', 'realsense_rgb', 'realsense_depth']:
            stream_key = cam_name
            if stream_key in streams:
                is_alive = streams[stream_key].get('alive', False)
                if cam_name in self.camera_indicators:
                    self.camera_indicators[cam_name].setActive(is_alive)
        
        # Health text güncelle
        health_lines = [
            f"[{datetime.now().strftime('%H:%M:%S')}] Sistem Sağlığı",
            f"Status: {data.get('status', 'unknown')}",
            "",
            "Stream Durumları:"
        ]
        
        for stream_name, stream_data in streams.items():
            alive = "🟢 AKTIF" if stream_data.get('alive') else "🔴 PASIF"
            count = stream_data.get('count', 0)
            health_lines.append(f"  {stream_name}: {alive} ({count} frames)")
        
        self.health_text.setText("\n".join(health_lines))

    def _update_connection_status(self):
        """Bağlantı durumunu periyodik güncelle"""
        pass

    def add_log(self, message: str):
        """Log mesajı ekle"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_line = f"[{timestamp}] {message}"
        self.log_text.append(log_line)
        
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def closeEvent(self, event):
        """Pencere kapatılıyor"""
        self.add_log("[INFO] Sistem kapatılıyor...")
        
        # Threads durdur
        for thread in [self.logitech_thread, self.realsense_rgb_thread, 
                      self.realsense_depth_thread, self.telemetry_thread, 
                      self.health_thread]:
            if thread:
                thread.stop()
                thread.wait(1000)
        
        # Video writers kapat
        for camera_name, writer in self.video_writers.items():
            if writer:
                writer.release()
                self.add_log(f"[INFO] {camera_name} video writer kapatıldı")
        
        # ZMQ kapat
        try:
            self.zmq_cmd_socket.close(0)
            self.zmq_cmd_context.term()
        except:
            pass
        
        super().closeEvent(event)


# ----------------- MAIN -----------------

def main():
    rclpy.init()
    ros_node = rclpy.create_node("rover_control_station_3cam")
    
    app = QApplication(sys.argv)
    window = RoverControlStation(ros_node)
    window.show()
    
    try:
        ret = app.exec_()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
    
    sys.exit(ret)


if __name__ == "__main__":
    main()
