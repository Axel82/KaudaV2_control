"""
KAuda 5-Axes Control - Full application (procedural 3D model)
Features:
 - COM scan + Connect
 - Sliders with min/max labels
 - Jog +/- for each axis
 - Send position (XYZ + grip + tool angle) -> inverse kinematics -> angles
 - Teach Origin (home)
 - Console logs
 - 3D procedural model (no STL) using cylinders/cube
 - Smooth animation between poses
 - Serial read thread (shows Arduino responses in console)
"""

import sys
import math
import time
import numpy as np
import serial
import serial.tools.list_ports
import ctypes
import sys
from PyQt5 import QtGui
from PyQt5.QtGui import QPalette, QColor
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QTextEdit, QComboBox, QGroupBox, QGridLayout, QCheckBox, QTabWidget
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import MeshData, GLMeshItem, GLGridItem, GLScatterPlotItem, GLTextItem

# ---------------------------
# USER-MODIFIABLE PARAMETERS
# ---------------------------
# Segment lengths (mm) -- modify these to match your physical KAuda
L1 = 120.0   # shoulder -> elbow
L2 = 100.0   # elbow -> wrist
L3 = 80.0    # wrist -> tool (effector)
LBASE = 20.0  # base height
RADIUS = 10.0  # base radius for cylinders

# Slider limits (adjustable)
X_MIN, X_MAX = -250, 250
Y_MIN, Y_MAX = -250, 250
Z_MIN, Z_MAX = 0, 350
GRIP_MIN, GRIP_MAX = -90, 90
TOOL_MIN, TOOL_MAX = -180, 180

# Serial default
BAUDRATE = 115200

# ---------------------------
# Inverse kinematics
# ---------------------------
def inverse_kinematics(x, y, z, tool_angle_deg=0.0, gripper_angle_deg=0.0):
    """
    Return angles [theta1, theta2, theta3, theta4, theta5] in degrees.
    theta1: base azimuth (J1)
    theta2: shoulder elevation (J2)
    theta3: elbow (J3)
    theta4: wrist rotation (J4)
    theta5: gripper (J5)
    """
    # base
    theta1 = math.degrees(math.atan2(y, x)) if (x != 0 or y != 0) else 0.0
    # planar distance and vertical
    r = math.hypot(x, y)
    z_eff = z - LBASE  # consider base offset
    # law of cosines for elbow
    denom = 2 * L1 * L2
    num = r*r + z_eff*z_eff - L1*L1 - L2*L2
    cos_theta3 = num / denom
    cos_theta3 = max(-1.0, min(1.0, cos_theta3))  # Clamp to avoid numerical errors
    theta3_rad = math.acos(cos_theta3)
    # choose elbow-down by default; to invert use -theta3_rad
    k1 = L1 + L2 * math.cos(theta3_rad)
    k2 = L2 * math.sin(theta3_rad)
    theta2_rad = math.atan2(z_eff, r) - math.atan2(k2, k1)
    theta_tool_rad = math.radians(tool_angle_deg)
    theta4_rad = theta_tool_rad - theta2_rad - theta3_rad
    return [
        math.degrees(theta1),
        math.degrees(theta2_rad),
        math.degrees(theta3_rad),
        math.degrees(theta4_rad),
        float(gripper_angle_deg)
    ]

# ---------------------------
# Serial reader thread
# ---------------------------
class SerialReader(QThread):
    new_line = pyqtSignal(str)
    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self.running = True

    def run(self):
        while self.running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').rstrip('\r\n')
                    if line:
                        self.new_line.emit(line)
                else:
                    self.msleep(20)
            except Exception as e:
                self.new_line.emit(f"[SerialReadError] {e}")
                self.running = False

    def stop(self):
        self.running = False
        self.wait(200)

# ---------------------------
# Procedural mesh generation utilities
# ---------------------------
def make_cylinder(radius, length, slices=32):
    """
    Create a cylindrical MeshData aligned along local +X (height = length along X).
    We create two rings of vertices (start and end) and faces for the sides.
    """
    theta = np.linspace(0, 2*np.pi, slices, endpoint=False)
    y = radius * np.cos(theta)
    z = radius * np.sin(theta)
    x0 = np.zeros(slices, dtype=np.float32)
    x1 = np.ones(slices, dtype=np.float32) * float(length)

    verts0 = np.column_stack((x0, y, z))
    verts1 = np.column_stack((x1, y, z))
    verts = np.vstack((verts0, verts1)).astype(np.float32)

    faces = []
    n = slices
    for i in range(n):
        ni = (i + 1) % n
        # quad -> two triangles
        faces.append([i, ni, n + ni])
        faces.append([i, n + ni, n + i])
    faces = np.array(faces, dtype=np.int32)
    md = MeshData(vertexes=verts, faces=faces)
    return md

def make_box(dx, dy, dz):
    md = MeshData.cube()
    # cube vertices are [-1,1] unit cube -> we scale
    md = MeshData(vertexes=(md.vertexes() * np.array([dx/2.0, dy/2.0, dz/2.0])))
    return md

class ToggleSwitch(QCheckBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setChecked(False)
        self.setStyleSheet("""
            QCheckBox::indicator {
                width: 40px;
                height: 20px;
                border-radius: 10px;
                background-color: #888;
            }
            QCheckBox::indicator:checked {
                background-color: #4CAF50;
            }
            QCheckBox::indicator:unchecked {
                background-color: #888;
            }
        """)

# ---------------------------
# Main Application
# ---------------------------
class KAudaApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("KAuda 5 axes Control")
        self.serial = None
        self.reader = None

        # animation angles
        self.current_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.target_angles = list(self.current_angles)

        # store raw mesh data (vertex arrays & faces) for transforms
        self.raw_mesh = {}    # key -> dict { 'md': MeshData, 'verts': np.array, 'faces': np.array }
        self.local_frames = []  # Initialisation de la liste pour stocker les lignes des axes

        self._build_ui()
        self._build_3d()
        self._apply_theme()
        self._build_timers()
        self._set_icon()

    def _set_icon(self):
        """Set the application window icon."""
        import os
        icon_path = os.path.join(os.path.dirname(__file__), "kauda_icon.png")
        if os.path.exists(icon_path):
            self.setWindowIcon(QtGui.QIcon(icon_path))

    def _apply_theme(self):
        # Dark Palette
        dark_palette = QPalette()
        
        # Colors
        color_bg = QColor(45, 45, 45)
        color_fg = QColor(220, 220, 220)
        color_base = QColor(30, 30, 30)
        color_alt_base = QColor(45, 45, 45)
        color_btn = QColor(60, 60, 60)
        color_highlight = QColor(42, 130, 218) # Blue highlight
        color_highlight_text = QColor(255, 255, 255)

        dark_palette.setColor(QPalette.Window, color_bg)
        dark_palette.setColor(QPalette.WindowText, color_fg)
        dark_palette.setColor(QPalette.Base, color_base)
        dark_palette.setColor(QPalette.AlternateBase, color_alt_base)
        dark_palette.setColor(QPalette.ToolTipBase, color_highlight)
        dark_palette.setColor(QPalette.ToolTipText, color_highlight_text)
        dark_palette.setColor(QPalette.Text, color_fg)
        dark_palette.setColor(QPalette.Button, color_btn)
        dark_palette.setColor(QPalette.ButtonText, color_fg)
        dark_palette.setColor(QPalette.BrightText, Qt.red)
        dark_palette.setColor(QPalette.Link, color_highlight)
        dark_palette.setColor(QPalette.Highlight, color_highlight)
        dark_palette.setColor(QPalette.HighlightedText, color_highlight_text)

        self.setPalette(dark_palette)

        # Stylesheet for specific tweaks
        self.setStyleSheet("""
            QWidget {
                color: #ffffff;
            }
            QToolTip { 
                color: #ffffff; 
                background-color: #2a82da; 
                border: 1px solid white; 
            }
            QGroupBox {
                border: 1px solid #555;
                border-radius: 5px;
                margin-top: 10px;
                font-weight: bold;
                color: #ffffff;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 3px;
                left: 10px;
            }
            QTabWidget::pane {
                border: 1px solid #555;
                background: #2d2d2d;
            }
            QTabBar::tab {
                background: #3c3f41;
                color: #ffffff;
                padding: 6px 8px;
                border: 1px solid #555;
                border-bottom-color: #555;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                min-width: 60px;
            }
            QTabBar::tab:selected, QTabBar::tab:hover {
                background: #505355;
                color: #ffffff;
            }
            QTabBar::tab:selected {
                border-color: #555;
                border-bottom-color: #2d2d2d; /* Blend with pane */
            }
            QPushButton {
                background-color: #3c3f41;
                border: 1px solid #555;
                border-radius: 4px;
                padding: 5px;
                color: #ffffff;
            }
            QPushButton:hover {
                background-color: #505355;
            }
            QPushButton:pressed {
                background-color: #2a82da;
                color: #ffffff;
            }
            QSlider::groove:horizontal {
                border: 1px solid #555;
                height: 8px;
                background: #2d2d2d;
                margin: 2px 0;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #2a82da;
                border: 1px solid #2a82da;
                width: 18px;
                height: 18px;
                margin: -7px 0;
                border-radius: 9px;
            }
            QComboBox {
                background: #3c3f41;
                border: 1px solid #555;
                border-radius: 4px;
                padding: 4px;
                color: #ffffff;
            }
            QComboBox::drop-down {
                subcontrol-origin: padding;
                subcontrol-position: top right;
                width: 20px;
                border-left-width: 1px;
                border-left-color: #555;
                border-left-style: solid;
            }
            QTextEdit {
                background-color: #1e1e1e;
                color: #ffffff;
                border: 1px solid #555;
            }
            QLabel {
                color: #ffffff;
            }
        """)

    # -----------------------
    # UI construction
    # -----------------------
    def _build_ui(self):
        # Main layout is vertical: Top (Tabs + 3D) / Bottom (Console)
        main_layout = QVBoxLayout(self)

        # Top section: Horizontal split between Tabs (Left) and 3D View (Right)
        top_layout = QHBoxLayout()
        
        # Left: Vertical Tabs
        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.West)  # Vertical tabs on the left
        #self.tabs.setFixedWidth(400)

        # Create Tab Widgets
        tab_conn = QWidget()
        tab_cart = QWidget()
        tab_joint = QWidget()
        tab_tools = QWidget()

        self.tabs.addTab(tab_conn, "Connexion")
        self.tabs.addTab(tab_cart, "Cartésien")
        self.tabs.addTab(tab_joint, "Articulaire")
        self.tabs.addTab(tab_tools, "Outils")

        # --- Tab 1: Connexion ---
        layout_conn = QVBoxLayout()
        
        gb_ports = QGroupBox("Ports / Connexion")
        gbp_layout = QHBoxLayout()
        self.combo_com = QComboBox()
        btn_refresh = QPushButton("Refresh")
        btn_refresh.clicked.connect(self.refresh_ports)
        btn_connect = QPushButton("Connect")
        btn_connect.clicked.connect(self.connect_serial)
        gbp_layout.addWidget(self.combo_com)
        gbp_layout.addWidget(btn_refresh)
        gbp_layout.addWidget(btn_connect)
        gb_ports.setLayout(gbp_layout)
        layout_conn.addWidget(gb_ports)

        btn_teach = QPushButton("Teach Origin")
        btn_teach.clicked.connect(self.on_teach_origin)
        layout_conn.addWidget(btn_teach)
        
        layout_conn.addStretch() # Push items to top
        tab_conn.setLayout(layout_conn)

        # --- Tab 2: Cartésien ---
        layout_cart = QVBoxLayout()

        # sliders
        gb_sliders = QGroupBox("Positions et limites")
        grid = QGridLayout()

        self.slider_x, self.label_x = self._add_slider("X (mm)", X_MIN, X_MAX, 0)
        grid.addWidget(self.label_x, 0, 0)
        grid.addWidget(self.slider_x, 0, 1)

        self.slider_y, self.label_y = self._add_slider("Y (mm)", Y_MIN, Y_MAX, 0)
        grid.addWidget(self.label_y, 1, 0)
        grid.addWidget(self.slider_y, 1, 1)

        self.slider_z, self.label_z = self._add_slider("Z (mm)", Z_MIN, Z_MAX, int((Z_MIN+Z_MAX)/2))
        grid.addWidget(self.label_z, 2, 0)
        grid.addWidget(self.slider_z, 2, 1)

        self.slider_grip, self.label_grip = self._add_slider("Angle pince (deg)", GRIP_MIN, GRIP_MAX, 0)
        grid.addWidget(self.label_grip, 3, 0)
        grid.addWidget(self.slider_grip, 3, 1)

        self.slider_tool, self.label_tool = self._add_slider("Angle outil (deg)", TOOL_MIN, TOOL_MAX, 0)
        grid.addWidget(self.label_tool, 4, 0)
        grid.addWidget(self.slider_tool, 4, 1)

        gb_sliders.setLayout(grid)
        layout_cart.addWidget(gb_sliders)

        # jog buttons
        gb_jog = QGroupBox("Move Jog (1 unité)")
        jog_layout = QGridLayout()
        axes = [("X", self.slider_x), ("Y", self.slider_y), ("Z", self.slider_z),
                ("Grip", self.slider_grip), ("Tool", self.slider_tool)]
        for i, (name, slider) in enumerate(axes):
            btn_minus = QPushButton(f"{name} -")
            btn_plus = QPushButton(f"{name} +")
            btn_minus.clicked.connect(lambda _, s=slider: self.jog(s, -1))
            btn_plus.clicked.connect(lambda _, s=slider: self.jog(s, +1))
            jog_layout.addWidget(btn_minus, i, 0)
            jog_layout.addWidget(btn_plus, i, 1)
        gb_jog.setLayout(jog_layout)
        layout_cart.addWidget(gb_jog)

        btn_send = QPushButton("Envoyer Position")
        btn_send.clicked.connect(self.on_send_position)
        layout_cart.addWidget(btn_send)

        layout_cart.addStretch()
        tab_cart.setLayout(layout_cart)

        # --- Tab 3: Articulaire ---
        layout_joint = QVBoxLayout()
        
        # Constants for joint slider ranges (deg) -- Mise à jour selon les specs KAuda
        J1_MIN, J1_MAX = -165, 165   # base
        J2_MIN, J2_MAX = -100, 120   # shoulder
        J3_MIN, J3_MAX = -60, 150    # elbow
        J4_MIN, J4_MAX = -175, 175   # wrist
        J5_MIN, J5_MAX = -30, 130    # gripper rotation

        gb_joints = QGroupBox("Joint Angles (deg) - pour GOTO")
        grid_j = QGridLayout()

        # Joint 1
        self.slider_j1 = QSlider(Qt.Horizontal)
        self.slider_j1.setRange(J1_MIN, J1_MAX)
        self.slider_j1.setValue(0)
        lbl_j1 = QLabel(f"J1 Base ({J1_MIN} → {J1_MAX})")
        grid_j.addWidget(lbl_j1, 0, 0)
        grid_j.addWidget(self.slider_j1, 0, 1)

        # Joint 2
        self.slider_j2 = QSlider(Qt.Horizontal)
        self.slider_j2.setRange(J2_MIN, J2_MAX)
        self.slider_j2.setValue(0)
        lbl_j2 = QLabel(f"J2 Shoulder ({J2_MIN} → {J2_MAX})")
        grid_j.addWidget(lbl_j2, 1, 0)
        grid_j.addWidget(self.slider_j2, 1, 1)

        # Joint 3
        self.slider_j3 = QSlider(Qt.Horizontal)
        self.slider_j3.setRange(J3_MIN, J3_MAX)
        self.slider_j3.setValue(0)
        lbl_j3 = QLabel(f"J3 Elbow ({J3_MIN} → {J3_MAX})")
        grid_j.addWidget(lbl_j3, 2, 0)
        grid_j.addWidget(self.slider_j3, 2, 1)

        # Joint 4
        self.slider_j4 = QSlider(Qt.Horizontal)
        self.slider_j4.setRange(J4_MIN, J4_MAX)
        self.slider_j4.setValue(0)
        lbl_j4 = QLabel(f"J4 Wrist ({J4_MIN} → {J4_MAX})")
        grid_j.addWidget(lbl_j4, 3, 0)
        grid_j.addWidget(self.slider_j4, 3, 1)

        # Joint 5
        self.slider_j5 = QSlider(Qt.Horizontal)
        self.slider_j5.setRange(J5_MIN, J5_MAX)
        self.slider_j5.setValue(0)
        lbl_j5 = QLabel(f"J5 Gripper ({J5_MIN} → {J5_MAX})")
        grid_j.addWidget(lbl_j5, 4, 0)
        grid_j.addWidget(self.slider_j5, 4, 1)

        gb_joints.setLayout(grid_j)
        layout_joint.addWidget(gb_joints)

        # Connect joint sliders to preview update (so moving them previews the angular pose)
        for s in [self.slider_j1, self.slider_j2, self.slider_j3, self.slider_j4, self.slider_j5]:
            s.valueChanged.connect(self.on_angular_joint_slider_changed)

        btn_goto = QPushButton("GOTO")
        btn_goto.clicked.connect(self.on_goto_clicked)
        btn_gohome = QPushButton("GOHOME")
        btn_gohome.clicked.connect(self.on_gohome_clicked)

        layout_joint.addWidget(btn_goto)
        layout_joint.addWidget(btn_gohome)
        layout_joint.addStretch()
        tab_joint.setLayout(layout_joint)

        # --- Tab 4: Outils ---
        layout_tools = QVBoxLayout()
        
        # Gripper commands
        gb_gripper = QGroupBox("Gripper")
        grip_layout = QHBoxLayout()

        label = QLabel("Pince")
        self.gripper_switch = ToggleSwitch()
        self.gripper_switch.toggled.connect(self._on_gripper_toggled)

        grip_layout.addWidget(label)
        grip_layout.addWidget(self.gripper_switch)
        gb_gripper.setLayout(grip_layout)
        layout_tools.addWidget(gb_gripper)
        
        layout_tools.addStretch()
        tab_tools.setLayout(layout_tools)

        # Add Tabs to Top Layout
        top_layout.addWidget(self.tabs)

        # 3D view on right
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=800)
        top_layout.addWidget(self.view, stretch=1) # 3D view takes remaining space

        main_layout.addLayout(top_layout, stretch=4)

        # Bottom: Console
        main_layout.addWidget(QLabel("Console Logs:"))
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumHeight(150) # Limit console height
        main_layout.addWidget(self.console)

        self.setLayout(main_layout)

        # connect sliders to preview update
        for s in [self.slider_x, self.slider_y, self.slider_z, self.slider_grip, self.slider_tool]:
            s.valueChanged.connect(self.on_slider_changed)
        
        # --- Runner ---
        self.refresh_ports()

    def _add_slider(self, label_text, minv, maxv, default):
        label = QLabel(f"{label_text} ({minv} → {maxv})")
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(int(minv))
        slider.setMaximum(int(maxv))
        slider.setValue(int(default))
        return slider, label

    def refresh_ports(self):
        self.combo_com.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_com.addItem(p.device)
        self.log(f"Ports scannés: {[p.device for p in ports]}")

    def connect_serial(self):
        port = self.combo_com.currentText()
        if not port:
            self.log("Aucun port sélectionné.")
            return
        try:
            self.serial = serial.Serial(port, BAUDRATE, timeout=0.1)
            self.log(f"Connecté à {port} @ {BAUDRATE}bps")
            if self.reader and self.reader.isRunning():
                self.reader.stop()
            self.reader = SerialReader(self.serial)
            self.reader.new_line.connect(self.on_serial_line)
            self.reader.start()
        except Exception as e:
            self.log(f"Erreur connexion: {e}")

    def on_serial_line(self, line):
        self.log(f"[Arduino] {line}")

    def log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.console.append(f"[{ts}] {msg}")

    # -----------------------
    # Jog & send & teach
    # -----------------------
    def jog(self, slider, direction):
        new_val = slider.value() + direction
        new_val = max(slider.minimum(), min(slider.maximum(), new_val))
        slider.setValue(new_val)
        self.log(f"Jog -> {new_val}")

    def on_slider_changed(self):
        x = self.slider_x.value()
        y = self.slider_y.value()
        z = self.slider_z.value()
        grip = self.slider_grip.value()
        tool = self.slider_tool.value()
        angles = inverse_kinematics(x, y, z, tool, grip)
        self.target_angles = angles  # preview target
        self.update_3d_immediate(angles)

    def on_send_position(self):
        if self.serial is None or not (hasattr(self.serial, 'is_open') and self.serial.is_open):
            self.log("Arduino non connecté. Utilisez Connect.")
            return
        x = self.slider_x.value()
        y = self.slider_y.value()
        z = self.slider_z.value()
        grip = self.slider_grip.value()
        tool = self.slider_tool.value()
        angles = inverse_kinematics(x, y, z, tool, grip)
        self.log(f"Calcul IK -> angles: {['{:.2f}'.format(a) for a in angles]}")
        self.target_angles = angles
        self.send_angles_to_arduino(angles)

    def on_teach_origin(self):
        if self.serial is None or not (hasattr(self.serial, 'is_open') and self.serial.is_open):
            self.log("Arduino non connecté. Usage: Connect.")
            return
        home_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.target_angles = home_angles
        self.send_angles_to_arduino(home_angles)
        self.log("Teach Origin envoyé (home)")

    def on_angular_joint_slider_changed(self):
        """
        Preview the robot pose using the joint-slider angles (no IK).
        This updates the 3D display by converting the joint sliders directly into the model frames.
        """
        a1 = float(self.slider_j1.value())
        a2 = float(self.slider_j2.value())
        a3 = float(self.slider_j3.value())
        a4 = float(self.slider_j4.value())
        a5 = float(self.slider_j5.value())
        # Set target angles for animation preview (so the animation interpolates toward this pose)
        self.target_angles = [a1, a2, a3, a4, a5]
        # Update 3D immediately for instant feedback (preview)
        self.update_3d_immediate(self.target_angles)

    def send_angles_to_arduino(self, angles):
        if self.serial is None or not (hasattr(self.serial, 'is_open') and self.serial.is_open):
            self.log("Impossible d'envoyer: Arduino non connecté.")
            return
        try:
            payload = "A," + ",".join([f"{a:.2f}" for a in angles]) + "\n"
            self.serial.write(payload.encode('utf-8'))
            self.log(f"[TX] {payload.strip()}")
        except Exception as e:
            self.log(f"Erreur d'envoi: {e}")

    def on_goto_clicked(self):
        if not self.serial or not self.serial.is_open:
            self.log("⚠️ Aucun port COM connecté.")
            return

        angles = [
            self.slider_j1.value(),
            self.slider_j2.value(),
            self.slider_j3.value(),
            self.slider_j4.value(),
            self.slider_j5.value(),
        ]

        msg = f"GOTO;A1={angles[0]};A2={angles[1]};A3={angles[2]};A4={angles[3]};A5={angles[4]}\n"
        try:
            self.serial.write(msg.encode("utf-8"))
            self.log(f"➡️ Sent: {msg.strip()}")
        except Exception as e:
            self.log(f"❌ Erreur d'envoi GOTO : {e}")

    def on_gohome_clicked(self):
        home = [0, 0, 0, 0, 0]

        # 1) Mise à jour UI (sliders)
        self.slider_j1.setValue(home[0])
        self.slider_j2.setValue(home[1])
        self.slider_j3.setValue(home[2])
        self.slider_j4.setValue(home[3])
        self.slider_j5.setValue(home[4])

        # 2) Mise à jour de la 3D
        self.update_3d_immediate(home)

        # 3) Envoi série
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(b"GOHOME\n")
                self.log("➡️ Sent: GOHOME")
            except Exception as e:
                self.log(f"❌ Erreur GOHOME : {e}")
        else:
            self.log("⚠️ Aucun port COM connecté.")

    def _on_gripper_toggled(self, checked):
        if not self.serial or not self.serial.isOpen():
            self.log("⚠️ Aucun port COM connecté")
            return

        if checked:
            cmd = "GRIPPER_OPEN\n"
            self.log("🟢 Ouverture du gripper")
        else:
            cmd = "GRIPPER_CLOSE\n"
            self.log("🔴 Fermeture du gripper")

        try:
            self.serial.write(cmd.encode("ascii"))
        except Exception as e:
            self.log(f"❌ Erreur gripper: {str(e)}")

    # -----------------------
    # 3D scene (procedural)
    # -----------------------
    def _build_3d(self):
        # grid
        g = GLGridItem()
        g.setSize(800, 800)
        g.setSpacing(50, 50)
        self.view.addItem(g)

        # create procedural mesh primitives (raw)
        # Base (short cylinder)
        base_md = make_cylinder(RADIUS * 1.5, LBASE, slices=40)
        shoulder_md = make_cylinder(RADIUS, L1, slices=32)
        forearm_md = make_cylinder(RADIUS * 0.85, L2, slices=28)
        wrist_md = make_cylinder(RADIUS * 0.7, L3, slices=20)
        # --- cube (gripper) explicit vertices & faces (fallback without MeshData.cube()) ---
        cube_verts = np.array([
            [-1.0, -1.0, -1.0],
            [ 1.0, -1.0, -1.0],
            [ 1.0,  1.0, -1.0],
            [-1.0,  1.0, -1.0],
            [-1.0, -1.0,  1.0],
            [ 1.0, -1.0,  1.0],
            [ 1.0,  1.0,  1.0],
            [-1.0,  1.0,  1.0],
        ], dtype=np.float32)

        cube_faces = np.array([
            [0,1,2], [0,2,3],   # bottom
            [4,6,5], [4,7,6],   # top
            [0,4,5], [0,5,1],   # front
            [1,5,6], [1,6,2],   # right
            [2,6,7], [2,7,3],   # back
            [3,7,4], [3,4,0],   # left
        ], dtype=np.int32)

        # store raw mesh arrays for transformations
        self.raw_mesh['base'] = {
            'verts': base_md.vertexes().copy(),
            'faces': base_md.faces().copy()
        }
        self.raw_mesh['shoulder'] = {
            'verts': shoulder_md.vertexes().copy(),
            'faces': shoulder_md.faces().copy()
        }
        self.raw_mesh['forearm'] = {
            'verts': forearm_md.vertexes().copy(),
            'faces': forearm_md.faces().copy()
        }
        self.raw_mesh['wrist'] = {
            'verts': wrist_md.vertexes().copy(),
            'faces': wrist_md.faces().copy()
        }
        # cube vertices for gripper
        self.raw_mesh['gripper'] = {
            'verts': cube_verts.copy(),
            'faces': cube_faces.copy()
        }

        # create GLMeshItem placeholders (we'll update meshdata per frame)
        self.mesh_items = {}
        for key in ['base', 'shoulder', 'forearm', 'wrist', 'gripper']:
            md = MeshData(vertexes=self.raw_mesh[key]['verts'], faces=self.raw_mesh[key]['faces'])
            item = GLMeshItem(meshdata=md, smooth=True, shader='shaded', drawEdges=False)
            self.view.addItem(item)
            self.mesh_items[key] = item

        # joint markers
        self.joints = GLScatterPlotItem(size=8, color=(1,1,0,1))
        self.view.addItem(self.joints)

        # initial update
        self.update_3d_immediate(self.current_angles)

    def update_3d_immediate(self, angles_deg):
        theta1, theta2, theta3, theta4, theta5 = [math.radians(a) for a in angles_deg]

        # Helper transforms (4x4 homogeneous matrices)
        def RotX(a):
            c, s = math.cos(a), math.sin(a)
            return np.array([
                [1, 0, 0, 0],
                [0, c, -s, 0],
                [0, s, c, 0],
                [0, 0, 0, 1]
            ], dtype=np.float64)

        def RotY(a):
            c, s = math.cos(a), math.sin(a)
            return np.array([
                [c, 0, s, 0],
                [0, 1, 0, 0],
                [-s, 0, c, 0],
                [0, 0, 0, 1]
            ], dtype=np.float64)

        def RotZ(a):
            c, s = math.cos(a), math.sin(a)
            return np.array([
                [c, -s, 0, 0],
                [s, c, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ], dtype=np.float64)

        def Tx(t):
            return np.array([
                [1, 0, 0, t],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ], dtype=np.float64)

        def Tz(t):
            return np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, t],
                [0, 0, 0, 1]
            ], dtype=np.float64)

        # Frame chain
        F0 = np.eye(4)  # World frame

        # J1: Rotation around Z (base)
        F1 = F0 @ RotZ(theta1) @ Tz(LBASE)  # F1 est à la sortie de la base

        # J2: Rotation around Y of F1's end (shoulder)
        # F2 est placé à l'extrémité de L1 (épaule)
        F2 = F1 @ RotY(theta2) @ Tx(L1)

        # J3: Rotation around Y of F2's end (elbow)
        # F3 est placé à l'extrémité de L2 (coude)
        F3 = F2 @ RotY(theta3) @ Tx(L2)

        # J4: Rotation around Y of F3's end (wrist)
        # F4 est placé à l'extrémité de L3 (poignet)
        F4 = F3 @ RotY(theta4) @ Tx(L3)

        # Gripper frame
        F5 = F4

        # Mapping keys -> frames to use for transforming mesh vertices
        # Les maillages sont placés correctement, sans décalage de L/2
        frames = {
            'base': F0 @ RotY(-math.pi/2) @ Tx(LBASE / 2.0),  # Base mesh (vertical cylinder)
            'shoulder': F1,  # Shoulder mesh (début à F1, fin à F2)
            'forearm': F2,   # Forearm mesh (début à F2, fin à F3)
            'wrist': F3,     # Wrist mesh (début à F3, fin à F4)
            'gripper': F4    # Gripper mesh (attaché à F4)
        }

        # Apply transforms to each mesh (inchangé)
        for key, item in self.mesh_items.items():
            raw = self.raw_mesh.get(key)
            if raw is None:
                continue
            verts = raw['verts']  # Nx3
            faces = raw['faces']
            n = verts.shape[0]
            hom = np.ones((n, 4), dtype=np.float64)
            hom[:, :3] = verts
            frame = frames.get(key, np.eye(4))

            # Special handling for gripper
            if key == 'gripper':
                verts_scaled = verts * np.array([10.0, 6.0, 6.0])  # Scale gripper
                hom2 = np.ones((verts_scaled.shape[0], 4), dtype=np.float64)
                hom2[:, :3] = verts_scaled
                T_offset = np.eye(4)
                T_offset[0, 3] = 10.0  # Offset gripper along local X
                final_frame = frame @ T_offset
                trans = (final_frame @ hom2.T).T
                verts_t = trans[:, :3].astype(np.float32)
            else:
                trans = (frame @ hom.T).T
                verts_t = trans[:, :3].astype(np.float32)

            md = MeshData(vertexes=verts_t, faces=faces)
            item.setMeshData(meshdata=md)

        # Joint markers: show origins of F0..F4
        joint_pts = np.array([
            F0[:3, 3],  # Base origin
            F1[:3, 3],  # J1's end (sortie de la base)
            F2[:3, 3],  # J2's end (extrémité de L1)
            F3[:3, 3],  # J3's end (extrémité de L2)
            F4[:3, 3]   # J4's end (extrémité de L3)
        ], dtype=np.float32)
        self.joints.setData(pos=joint_pts, size=8, color=(1, 1, 0, 1))

        # Effacer les anciens repères et étiquettes
        for frame_items in self.local_frames:
            for item in frame_items:
                if item in self.view.items:
                    self.view.removeItem(item)
        self.local_frames = []

        # Dessiner les nouveaux repères pour chaque articulation avec leurs noms
        frames_list = [
            (F1, "J1"),
            (F2, "J2"),
            (F3, "J3"),
            (F4, "J4")
        ]
        for frame, name in frames_list:
            frame_items = self.draw_local_frame(frame, name=name)
            self.local_frames.append(frame_items)

    # -----------------------
    # Draw local frame axes
    # -----------------------
    def draw_local_frame(self, frame, name="", length=50, color_x=(1, 0, 0, 1), color_y=(0, 1, 0, 1), color_z=(0, 0, 1, 1)):
        """
        Dessine les axes X, Y, Z d'un repère local et ajoute une étiquette visible.
        :param frame: Matrice 4x4 du repère local.
        :param name: Nom du repère (ex: "J1").
        :param length: Longueur des axes.
        :param color_x: Couleur de l'axe X (rouge).
        :param color_y: Couleur de l'axe Y (vert).
        :param color_z: Couleur de l'axe Z (bleu).
        """
        origin = frame[:3, 3]
        items = []

        # Axe X (rouge)
        x_axis_end = origin + frame[:3, :3] @ np.array([length, 0, 0])
        x_axis = np.array([origin, x_axis_end])
        x_line = gl.GLLinePlotItem(pos=x_axis, color=color_x, width=2, antialias=True)
        self.view.addItem(x_line)
        items.append(x_line)

        # Axe Y (vert)
        y_axis_end = origin + frame[:3, :3] @ np.array([0, length, 0])
        y_axis = np.array([origin, y_axis_end])
        y_line = gl.GLLinePlotItem(pos=y_axis, color=color_y, width=2, antialias=True)
        self.view.addItem(y_line)
        items.append(y_line)

        # Axe Z (bleu)
        z_axis_end = origin + frame[:3, :3] @ np.array([0, 0, length])
        z_axis = np.array([origin, z_axis_end])
        z_line = gl.GLLinePlotItem(pos=z_axis, color=color_z, width=2, antialias=True)
        self.view.addItem(z_line)
        items.append(z_line)

        # Ajouter une étiquette pour le repère
        if name:
            # Décalage pour éviter la superposition avec les axes
            text_offset = frame[:3, :3] @ np.array([0, 0, length * 1.2])  # Décalage le long de Z local
            text_pos = origin + text_offset

            # Création du texte
            text = GLTextItem(pos=text_pos, text=name, color=(1, 1, 1, 1))

            # Ajout à la vue
            self.view.addItem(text)
            items.append(text)

        return items

    # -----------------------
    # Animation timer
    # -----------------------
    def _build_timers(self):
        self.anim_timer = QTimer()
        self.anim_timer.setInterval(30)
        self.anim_timer.timeout.connect(self._animate_step)
        self.anim_timer.start()

    def _animate_step(self):
        stepped = False
        for i in range(len(self.current_angles)):
            cur = self.current_angles[i]
            tgt = self.target_angles[i]
            delta = tgt - cur
            if abs(delta) > 0.03:
                step = np.sign(delta) * max(0.2, abs(delta) * 0.18)
                if abs(step) > abs(delta):
                    step = delta
                self.current_angles[i] = cur + step
                stepped = True
        if stepped:
            self.update_3d_immediate(self.current_angles)

    # -----------------------
    # Clean up
    # -----------------------
    def closeEvent(self, event):
        if self.reader and self.reader.isRunning():
            self.reader.stop()
        if self.serial and hasattr(self.serial, 'is_open') and self.serial.is_open:
            try:
                self.serial.close()
            except:
                pass
        event.accept()

# ---------------------------
# Runner
# ---------------------------
def main():
    # Set Windows AppUserModelID for taskbar icon support
    if sys.platform == 'win32':
        myappid = 'axelhabeillon.kauda.control.1.0'  # arbitrary string
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)
    
    app = QApplication(sys.argv)
    
    # Set application icon globally
    import os
    icon_path = os.path.join(os.path.dirname(__file__), "kauda_icon.png")
    if os.path.exists(icon_path):
        app.setWindowIcon(QtGui.QIcon(icon_path))
    
    win = KAudaApp()
    win.resize(1300, 800)
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
