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
import os
import numpy as np
import serial
import ctypes
from PyQt5 import QtGui
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QTextEdit, QComboBox, QGroupBox, QGridLayout, QTabWidget
)
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import MeshData, GLMeshItem, GLGridItem, GLScatterPlotItem

# Import custom modules
from config import *
from serial_comm import SerialReader, scan_ports, format_angles_command, format_goto_command
from ui_components import ToggleSwitch, create_slider, create_joint_slider
from theme import get_dark_palette, get_stylesheet
from visualization_3d import make_cylinder, make_box, draw_local_frame
from version import Version
from kinematics_5dof import inverse_kinematics, forward_kinematics, deg2rad
from stl_mesh import load_stl_mesh

STL_DIR = os.path.join(os.path.dirname(__file__), "STL")

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
        
        # Gripper state
        self.gripper_open = False  # False = closed, True = open

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
        """Apply dark theme to the application"""
        self.setPalette(get_dark_palette())
        self.setStyleSheet(get_stylesheet())

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
        self.tabs.setMinimumWidth(450)  # Increased to accommodate wider sliders
        
        # Connect signal to adjust width based on selected tab
        self.tabs.currentChanged.connect(self._on_tab_changed)

        # Create Tab Widgets
        tab_conn = QWidget()
        tab_cart = QWidget()
        tab_joint = QWidget()
        tab_tools = QWidget()
        tab_about = QWidget()

        self.tabs.addTab(tab_conn, "Connexion")
        self.tabs.addTab(tab_cart, "Cartésien")
        self.tabs.addTab(tab_joint, "Articulaire")
        self.tabs.addTab(tab_tools, "Outils")
        self.tabs.addTab(tab_about, "About")

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

        self.slider_x, self.label_x, self.value_x = create_slider("X (mm)", X_MIN, X_MAX, 0)
        grid.addWidget(self.label_x, 0, 0)
        grid.addWidget(self.slider_x, 0, 1)
        grid.addWidget(self.value_x, 0, 2)

        self.slider_y, self.label_y, self.value_y = create_slider("Y (mm)", Y_MIN, Y_MAX, 0)
        grid.addWidget(self.label_y, 1, 0)
        grid.addWidget(self.slider_y, 1, 1)
        grid.addWidget(self.value_y, 1, 2)

        self.slider_z, self.label_z, self.value_z = create_slider("Z (mm)", Z_MIN, Z_MAX, int((Z_MIN+Z_MAX)/2))
        grid.addWidget(self.label_z, 2, 0)
        grid.addWidget(self.slider_z, 2, 1)
        grid.addWidget(self.value_z, 2, 2)

        self.slider_grip, self.label_grip, self.value_grip = create_slider("Angle pince (deg)", GRIP_MIN, GRIP_MAX, 0)
        grid.addWidget(self.label_grip, 3, 0)
        grid.addWidget(self.slider_grip, 3, 1)
        grid.addWidget(self.value_grip, 3, 2)

        self.slider_tool, self.label_tool, self.value_tool = create_slider("Angle outil (deg)", TOOL_MIN, TOOL_MAX, 0)
        grid.addWidget(self.label_tool, 4, 0)
        grid.addWidget(self.slider_tool, 4, 1)
        grid.addWidget(self.value_tool, 4, 2)

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

        gb_joints = QGroupBox("Joint Angles (deg) - pour GOTO")
        grid_j = QGridLayout()

        # Joint 1
        self.slider_j1, lbl_j1, val_j1 = create_joint_slider("Base", 1, J1_MIN, J1_MAX)
        grid_j.addWidget(lbl_j1, 0, 0)
        grid_j.addWidget(self.slider_j1, 0, 1)
        grid_j.addWidget(val_j1, 0, 2)

        # Joint 2
        self.slider_j2, lbl_j2, val_j2 = create_joint_slider("Shoulder", 2, J2_MIN, J2_MAX)
        grid_j.addWidget(lbl_j2, 1, 0)
        grid_j.addWidget(self.slider_j2, 1, 1)
        grid_j.addWidget(val_j2, 1, 2)

        # Joint 3
        self.slider_j3, lbl_j3, val_j3 = create_joint_slider("Elbow", 3, J3_MIN, J3_MAX)
        grid_j.addWidget(lbl_j3, 2, 0)
        grid_j.addWidget(self.slider_j3, 2, 1)
        grid_j.addWidget(val_j3, 2, 2)

        # Joint 4
        self.slider_j4, lbl_j4, val_j4 = create_joint_slider("Wrist", 4, J4_MIN, J4_MAX)
        grid_j.addWidget(lbl_j4, 3, 0)
        grid_j.addWidget(self.slider_j4, 3, 1)
        grid_j.addWidget(val_j4, 3, 2)

        # Joint 5
        self.slider_j5, lbl_j5, val_j5 = create_joint_slider("Gripper", 5, J5_MIN, J5_MAX)
        grid_j.addWidget(lbl_j5, 4, 0)
        grid_j.addWidget(self.slider_j5, 4, 1)
        grid_j.addWidget(val_j5, 4, 2)

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

        # --- Tab 5: About ---
        layout_about = QVBoxLayout()
        
        # Logo
        logo_label = QLabel()
        icon_path = os.path.join(os.path.dirname(__file__), "kauda_icon.png")
        if os.path.exists(icon_path):
            pixmap = QtGui.QPixmap(icon_path)
            scaled_pixmap = pixmap.scaled(128, 128, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            logo_label.setPixmap(scaled_pixmap)
            logo_label.setAlignment(Qt.AlignCenter)
        layout_about.addWidget(logo_label)
        
        # Version info
        version_info = Version.get_full_info()
        info_text = f"""
        <h2 style='text-align: center;'>{APP_NAME}</h2>
        <p style='text-align: center;'>
            <b>Version:</b> {version_info['version']}<br>
            <b>Author:</b> {version_info['author']}<br>
            <b>License:</b> {version_info['license']}<br>
            <b>Repository:</b> <a href="{version_info['repository']}">{version_info['repository']}</a>
        </p>
        <p style='text-align: center;'>{version_info['description']}</p>
        """
        info_label = QLabel(info_text)
        info_label.setWordWrap(True)
        info_label.setAlignment(Qt.AlignCenter)
        info_label.setOpenExternalLinks(True)  # Enable clickable links
        layout_about.addWidget(info_label)
        
        # README viewer
        readme_label = QLabel("<b>README:</b>")
        layout_about.addWidget(readme_label)
        
        readme_viewer = QTextEdit()
        readme_viewer.setReadOnly(True)
        readme_path = os.path.join(os.path.dirname(__file__), "README.md")
        if os.path.exists(readme_path):
            with open(readme_path, 'r', encoding='utf-8') as f:
                readme_content = f.read()
                readme_viewer.setMarkdown(readme_content)  # Use Markdown rendering
        else:
            readme_viewer.setPlainText("README.md not found")
        layout_about.addWidget(readme_viewer)
        
        tab_about.setLayout(layout_about)

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

    def _on_tab_changed(self, index):
        """Adjust tab widget width based on selected tab"""
        # About tab (index 4) should be wider
        if index == 4:  # About tab
            self.tabs.setMinimumWidth(600)
            self.tabs.setMaximumWidth(600)
        else:
            self.tabs.setMinimumWidth(450)
            self.tabs.setMaximumWidth(450)


    def refresh_ports(self):
        self.combo_com.clear()
        ports = scan_ports()
        for p in ports:
            self.combo_com.addItem(p)
        self.log(f"Ports scannés: {ports}")

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
        # Lire angles articulaires
        angles_deg = [
            self.slider_j1.value(),
            self.slider_j2.value(),
            self.slider_j3.value(),
            self.slider_j4.value(),
            self.slider_j5.value()
        ]
        angles_rad = deg2rad(angles_deg)  # Convertir en radians pour FK

        # Mettre à jour la 3D
        self.target_angles = angles_deg
        self.update_3d_immediate(self.target_angles)

        # FK pour récupérer la position finale du gripper
        links = [LBASE, L1, L2, L3, 0.0]  # Rappel: L5 pour la pince, ici zéro si tu ne l'utilises pas
        T, pts = forward_kinematics(angles_rad, links)
        x, y, z = pts[-1]  # Position finale du gripper

        # Optionnel: prendre aussi les angles tool et grip
        tool = angles_deg[3]  # J4
        grip = angles_deg[4]  # J5

        # Bloquer signaux pour éviter boucle infinie
        for s in [self.slider_x, self.slider_y, self.slider_z, self.slider_tool, self.slider_grip]:
            s.blockSignals(True)

        self.slider_x.setValue(int(round(x)))
        self.slider_y.setValue(int(round(y)))
        self.slider_z.setValue(int(round(z)))
        self.slider_tool.setValue(int(round(tool)))
        self.slider_grip.setValue(int(round(grip)))

        for s in [self.slider_x, self.slider_y, self.slider_z, self.slider_tool, self.slider_grip]:
            s.blockSignals(False)

    def send_angles_to_arduino(self, angles):
        if self.serial is None or not (hasattr(self.serial, 'is_open') and self.serial.is_open):
            self.log("Impossible d'envoyer: Arduino non connecté.")
            return
        try:
            payload = format_angles_command(angles)
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

        msg = format_goto_command(angles)
        try:
            self.serial.write(msg.encode("utf-8"))
            self.log(f"➡️ Sent: {msg.strip()}")
        except Exception as e:
            self.log(f"❌ Erreur d'envoi GOTO : {e}")

    def on_gohome_clicked(self):
        # Use HOME_POSITION from robot_config
        home = HOME_ANGULAR_POSITION

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
        # Update gripper state
        self.gripper_open = checked
        
        # Refresh 3D visualization to show jaw movement
        self.update_3d_immediate(self.current_angles)
        
        # Send serial command
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
        # Enhanced grid with better visibility
        g = GLGridItem()
        g.setSize(800, 800)
        g.setSpacing(50, 50)
        g.setColor((100, 100, 120, 150))  # Subtle blue-gray color
        self.view.addItem(g)
        
        # Set background color for better contrast
        self.view.setBackgroundColor((25, 25, 30))

        # Joints markers
        self.joints = GLScatterPlotItem(size=8, color=(1,1,0,1))
        self.view.addItem(self.joints)
        self.local_frames = []

        # Charger STL segments
        self.mesh_items = {}
        self.raw_mesh = {}

        # Exemple fichiers STL : base, shoulder, forearm, wrist, jaw_left, jaw_right
        stl_files = {
            'base': "Base.stl",
            'shoulder': "Segment_1.stl",
            'forearm': "Segment_2.stl",
            'wrist': "Segment_3.stl",
            'jaw_left': "Segment_4.stl",
            'jaw_right': "Gripper_I.stl"
        }

        colors = {
            'base': (0.95, 0.95, 0.95, 1.0),    # Blanc légèrement grisâtre
            'shoulder': (0.98, 0.98, 0.98, 1.0), # Blanc très clair
            'forearm': (0.97, 0.97, 0.97, 1.0),  # Blanc clair
            'wrist': (0.93, 0.93, 0.93, 1.0),    # Blanc grisâtre
            'jaw_left': (0.99, 0.99, 0.99, 1.0), # Blanc pur
            'jaw_right': (0.99, 0.99, 0.99, 1.0) # Blanc pur
        }

        for key, fname in stl_files.items():
            path = os.path.join(STL_DIR, fname)
            md = load_stl_mesh(path)
            self.raw_mesh[key] = {
                'verts': md.vertexes().copy(),
                'faces': md.faces().copy()
            }
            item = GLMeshItem(meshdata=md, smooth=True, shader='shaded',
                            drawEdges=False, color=colors[key], glOptions='opaque')
            self.view.addItem(item)
            self.mesh_items[key] = item

        # Update initial position
        self.update_3d_immediate(self.current_angles)

    def update_3d_immediate(self, angles_deg):
        """
        Transform STL vertices selon les angles actuels des joints.
        """
        # Conversion en radians
        theta1, theta2, theta3, theta4, theta5 = [np.radians(a) for a in angles_deg]

        # Transformations homogènes
        def rotz(t): c,s=np.cos(t),np.sin(t); return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]],dtype=float)
        def roty(t): c,s=np.cos(t),np.sin(t); return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]],dtype=float)
        def rotx(t): c,s=np.cos(t),np.sin(t); return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]],dtype=float)
        def tx(a): return np.array([[1,0,0,a],[0,1,0,0],[0,0,1,0],[0,0,0,1]],dtype=float)
        def tz(a): return np.array([[1,0,0,0],[0,1,0,0],[0,0,1,a],[0,0,0,1]],dtype=float)

        # Chain
        F0 = np.eye(4)
        F1 = F0 @ rotz(theta1) @ tz(LBASE)
        F2 = F1 @ roty(theta2) @ tx(L1)
        F3 = F2 @ roty(theta3) @ tx(L2)
        F4 = F3 @ roty(theta4) @ tx(L3)
        F5 = F4  # gripper

        frames = {
            'base': F0,
            'shoulder': F1,
            'forearm': F2,
            'wrist': F3,
            'jaw_left': F4,
            'jaw_right': F4
        }

        # Déplacer jaws selon gripper_open
        jaw_offset = 7.5 if self.gripper_open else 2.0
        jaw_offsets = {'jaw_left': -jaw_offset, 'jaw_right': jaw_offset}

        for key, item in self.mesh_items.items():
            raw = self.raw_mesh[key]
            verts = raw['verts']
            faces = raw['faces']
            n = verts.shape[0]
            hom = np.ones((n,4), dtype=np.float64)
            hom[:,:3] = verts
            frame = frames.get(key, np.eye(4))

            if key in ['jaw_left','jaw_right']:
                T_off = np.eye(4)
                T_off[0,3] = 10.0
                T_off[1,3] = jaw_offsets[key]
                final = frame @ T_off
            else:
                final = frame

            transformed = (final @ hom.T).T
            verts_t = transformed[:,:3].astype(np.float32)
            md = MeshData(vertexes=verts_t, faces=faces)
            item.setMeshData(meshdata=md)

        # Joints markers
        joint_pts = np.array([F0[:3,3], F1[:3,3], F2[:3,3], F3[:3,3], F4[:3,3]], dtype=np.float32)
        self.joints.setData(pos=joint_pts, size=8, color=(1,1,0,1))
    
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
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(APP_ID)
    
    app = QApplication(sys.argv)
    
    # Set application icon globally
    icon_path = os.path.join(os.path.dirname(__file__), "kauda_icon.png")
    if os.path.exists(icon_path):
        app.setWindowIcon(QtGui.QIcon(icon_path))
    
    win = KAudaApp()
    win.resize(1300, 800)
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
