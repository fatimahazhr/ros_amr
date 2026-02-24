#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AMR Robot Navigation System
Combines PyQt5 UI with ROS navigation logic
"""

import sys
import threading
import subprocess
import numpy as np
import rospy
import yaml
import time
import os
import signal

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (
    PolygonStamped, PoseWithCovarianceStamped, Twist
)

import tf2_ros
import tf.transformations as tft
from actionlib_msgs.msg import GoalID
from nav_msgs.msg import Path

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton,
    QVBoxLayout, QHBoxLayout, QInputDialog, QMessageBox
)
from PyQt5.QtCore import QTimer, QRectF, Qt

import pyqtgraph as pg

# =========================================================
# CONFIGURATION
# =========================================================

MAP_DIR = "/home/delivery/catkin_ws/src/stark/maps"

# TELEOP KEY MAP
MOVE_BINDINGS = {
    Qt.Key_W: (1, 0, 0, 0),   # Forward
    Qt.Key_S: (-1, 0, 0, 0),  # Backward
    Qt.Key_A: (0, 0, 0, 1),   # Turn left
    Qt.Key_D: (0, 0, 0, -1),  # Turn right
}

SPEED_BINDINGS = {
    Qt.Key_Q: (1.1, 1.1),
    Qt.Key_Z: (0.9, 0.9),
    Qt.Key_E: (1.0, 1.1),
    Qt.Key_C: (1.0, 0.9)
}

# =========================================================
# WORLD MODEL
# =========================================================

class WorldModel:
    """Thread-safe storage for robot world state."""
    
    def __init__(self):
        self.lock = threading.Lock()
        self.map_img = None
        self.map_extent = None
        self.scan_pts = None
        self.footprint_pts = None
        self.robot_pose = None  # (x, y, yaw)

    def snapshot(self):
        """Get thread-safe snapshot of current state."""
        with self.lock:
            return (
                self.map_img,
                self.map_extent,
                self.scan_pts,
                self.footprint_pts,
                self.robot_pose
            )

# =========================================================
# ROS INGEST
# =========================================================

class ROSIngest:
    """Handles ROS topic subscriptions and TF lookups."""
    
    def __init__(self, model):
        self.model = model
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=1)
        rospy.Subscriber(
            "/move_base/local_costmap/footprint",
            PolygonStamped, self.footprint_cb, queue_size=1
        )

    def map_cb(self, msg):
        """Process map updates."""
        h, w = msg.info.height, msg.info.width
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y

        data = np.array(msg.data).reshape(w, h).T
        img = np.zeros_like(data, dtype=np.uint8)
        img[data == -1] = 100  # Unknown
        img[data == 0] = 255   # Free
        img[data > 0] = 0      # Occupied

        with self.model.lock:
            self.model.map_img = img
            self.model.map_extent = (ox, ox + w * res, oy, oy + h * res)

    def update_robot_pose(self):
        """Update robot pose from TF."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link",
                rospy.Time(0), rospy.Duration(0.05)
            )
        except Exception:
            return

        q = tf.transform.rotation
        yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        with self.model.lock:
            self.model.robot_pose = (
                tf.transform.translation.x,
                tf.transform.translation.y,
                yaw
            )

    def scan_cb(self, msg):
        """Process laser scan data."""
        try:
            target = "map" if self.model.map_img is not None else "odom"
            tf = self.tf_buffer.lookup_transform(
                target, msg.header.frame_id,
                rospy.Time(0), rospy.Duration(0.05)
            )
        except Exception:
            return

        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid = np.isfinite(ranges)

        xs = ranges[valid] * np.cos(angles[valid])
        ys = ranges[valid] * np.sin(angles[valid])

        T = tft.quaternion_matrix([
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w
        ])
        T[0, 3] = tf.transform.translation.x
        T[1, 3] = tf.transform.translation.y

        pts = np.vstack([xs, ys, np.zeros_like(xs), np.ones_like(xs)])

        with self.model.lock:
            self.model.scan_pts = (T @ pts)[:2].T

    def footprint_cb(self, msg):
        """Process robot footprint."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", msg.header.frame_id,
                rospy.Time(0), rospy.Duration(0.05)
            )
        except Exception:
            return

        T = tft.quaternion_matrix([
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w
        ])
        T[0, 3] = tf.transform.translation.x
        T[1, 3] = tf.transform.translation.y

        pts = np.array([[p.x, p.y, 0, 1] for p in msg.polygon.points]).T

        with self.model.lock:
            self.model.footprint_pts = (T @ pts)[:2].T

# =========================================================
# CUSTOM VIEWBOX FOR LOCALIZATION
# =========================================================

class LocalizeViewBox(pg.ViewBox):
    """Custom ViewBox that handles localization mode interactions."""
    
    def __init__(self, ui):
        super().__init__()
        self.ui = ui
        self.dragging = False

    def mousePressEvent(self, ev):
        if self.ui.localization_mode and ev.button() == Qt.LeftButton:
            pos = self.mapSceneToView(ev.scenePos())
            self.ui._loc_start_press(pos)
            self.dragging = True
            ev.accept()
        else:
            super().mousePressEvent(ev)

    def mouseDragEvent(self, ev, axis=None):
        if self.ui.localization_mode and self.dragging:
            pos = self.mapSceneToView(ev.scenePos())
            self.ui._loc_drag(pos)
            ev.accept()
        else:
            super().mouseDragEvent(ev, axis)

    def mouseReleaseEvent(self, ev):
        if self.ui.localization_mode and self.dragging:
            pos = self.mapSceneToView(ev.scenePos())
            self.ui._loc_release(pos)
            self.dragging = False
            ev.accept()
        else:
            super().mouseReleaseEvent(ev)

# =========================================================
# MAIN UI CLASS
# =========================================================

class Ui_MainWindow(object):
    """Main window UI definition."""
    
    def setupUi(self, MainWindow):
        """Initialize and setup the main window UI components."""
        # Main Window Configuration
        self._setup_main_window(MainWindow)
        
        # Central Widget
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        
        # Main Grid Layout
        self._setup_main_layout()
        
        # Left Panel - Control Buttons
        self._setup_control_panel()
        
        # Right Panel - Map Display
        self._setup_map_panel()
        
        # Menu and Status Bar
        self._setup_menu_statusbar(MainWindow)
        
        # Set text labels
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def _setup_main_window(self, MainWindow):
        """Configure main window properties."""
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1000, 650)
        
        sizePolicy = QtWidgets.QSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding
        )
        MainWindow.setSizePolicy(sizePolicy)

    def _setup_main_layout(self):
        """Setup the main grid layout container."""
        self.gridLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(5, 5, 990, 600))
        
        self.mainGridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.mainGridLayout.setContentsMargins(3, 3, 3, 3)
        self.mainGridLayout.setSpacing(8)

    def _setup_control_panel(self):
        """Setup the left control panel with navigation buttons."""
        self.controlLayout = QtWidgets.QVBoxLayout()
        self.controlLayout.setSpacing(5)
        
        # Title Label
        self.titleLabel = self._create_title_label("Robot Navigasi")
        self.controlLayout.addWidget(
            self.titleLabel,
            0,
            QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop
        )
        
        # Control Buttons
        button_configs = [
            ("buttonMenu", "Menu"),
            ("buttonCreateMap", "Create Map"),
            ("buttonLoadMap", "Load Map"),
            ("buttonSetMap", "Set Map"),
            ("buttonCreateWp", "Create Waypoint"),
            ("buttonSetWp", "Set Waypoint"),
            ("buttonLoadWp", "Load Waypoint"),
            ("buttonWpList", "Waypoint List"),
            ("buttonSelectPoint", "Select Point To Go"),
            ("buttonPointList", "Point List"),
        ]
        
        for obj_name, text in button_configs:
            button = self._create_control_button(obj_name, text)
            self.controlLayout.addWidget(button)
        
        # Add stretch to push buttons to top
        self.controlLayout.addStretch()
        
        self.mainGridLayout.addLayout(self.controlLayout, 0, 0, 1, 1)

    def _setup_map_panel(self):
        """Setup the right panel with map display and controls."""
        self.mapLayout = QtWidgets.QVBoxLayout()
        self.mapLayout.setSpacing(5)
        
        # Map Title
        self.mapTitleLabel = self._create_section_label("Map View")
        self.mapLayout.addWidget(self.mapTitleLabel, 0, QtCore.Qt.AlignHCenter)
        
        # Map Display Area (placeholder for pyqtgraph widget)
        self.mapContainer = QtWidgets.QWidget()
        self.mapContainer.setMinimumSize(600, 350)
        self.mapContainer.setStyleSheet(
            "background-color: #f0f0f0; "
            "border: 2px solid #2d2d2d; "
            "border-radius: 4px;"
        )
        self.mapLayout.addWidget(self.mapContainer, 1)
        
        # Control Section
        self._setup_navigation_controls()
        self.mapLayout.addLayout(self.navigationControlLayout)
        
        self.mainGridLayout.addLayout(self.mapLayout, 0, 1, 1, 1)
        
        # Set column stretch to give map more space
        self.mainGridLayout.setColumnStretch(0, 1)
        self.mainGridLayout.setColumnStretch(1, 3)

    def _setup_navigation_controls(self):
        """Setup navigation controls (WASD, radio buttons, emergency)."""
        self.navigationControlLayout = QtWidgets.QHBoxLayout()
        self.navigationControlLayout.setSpacing(15)
        
        # Left side - Mode selection and WASD
        leftControlLayout = QtWidgets.QVBoxLayout()
        
        # Radio Buttons
        radioLayout = QtWidgets.QHBoxLayout()
        self.radioButtonManual = self._create_radio_button("Manual")
        self.radioButtonAuto = self._create_radio_button("Auto")
        radioLayout.addWidget(self.radioButtonManual)
        radioLayout.addWidget(self.radioButtonAuto)
        radioLayout.addStretch()
        leftControlLayout.addLayout(radioLayout)
        
        # WASD Keys
        self._setup_wasd_keys()
        leftControlLayout.addLayout(self.wasdLayout)
        leftControlLayout.addStretch()
        
        self.navigationControlLayout.addLayout(leftControlLayout, 1)
        
        # Right side - Emergency button
        self.buttonEmergency = self._create_emergency_button()
        self.navigationControlLayout.addWidget(self.buttonEmergency)

    def _setup_wasd_keys(self):
        """Setup WASD keyboard control buttons."""
        self.wasdLayout = QtWidgets.QGridLayout()
        self.wasdLayout.setSpacing(5)
        
        # Create WASD buttons
        self.keyW = self._create_key_button("W")
        self.keyA = self._create_key_button("A")
        self.keyS = self._create_key_button("S")
        self.keyD = self._create_key_button("D")
        
        # Add to grid layout (WASD formation)
        self.wasdLayout.addWidget(self.keyW, 0, 1)  # Top center
        self.wasdLayout.addWidget(self.keyA, 1, 0)  # Bottom left
        self.wasdLayout.addWidget(self.keyS, 1, 1)  # Bottom center
        self.wasdLayout.addWidget(self.keyD, 1, 2)  # Bottom right

    def _setup_menu_statusbar(self, MainWindow):
        """Setup menu bar and status bar."""
        MainWindow.setCentralWidget(self.centralwidget)
        
        # Menu Bar
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1000, 20))
        MainWindow.setMenuBar(self.menubar)
        
        # Status Bar
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        MainWindow.setStatusBar(self.statusbar)

    # Helper Methods for Creating Widgets
    
    def _create_title_label(self, text):
        """Create a styled title label."""
        label = QtWidgets.QLabel()
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        label.setFont(font)
        label.setText(text)
        label.setAlignment(QtCore.Qt.AlignCenter)
        return label

    def _create_section_label(self, text):
        """Create a styled section label."""
        label = QtWidgets.QLabel()
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        label.setFont(font)
        label.setText(text)
        return label

    def _create_control_button(self, object_name, text):
        """Create a styled control button."""
        button = QtWidgets.QPushButton()
        
        sizePolicy = QtWidgets.QSizePolicy(
            QtWidgets.QSizePolicy.Minimum,
            QtWidgets.QSizePolicy.Fixed
        )
        button.setSizePolicy(sizePolicy)
        button.setMinimumHeight(32)
        
        font = QtGui.QFont()
        font.setPointSize(11)
        button.setFont(font)
        
        button.setStyleSheet(self._get_control_button_style())
        button.setObjectName(object_name)
        button.setText(text)
        
        setattr(self, object_name, button)
        return button

    def _create_key_button(self, key_text):
        """Create a WASD keyboard button."""
        button = QtWidgets.QPushButton()
        button.setFixedSize(45, 45)
        
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        button.setFont(font)
        
        button.setStyleSheet(self._get_key_button_style())
        button.setText(key_text)
        
        return button

    def _create_radio_button(self, text):
        """Create a styled radio button."""
        radio = QtWidgets.QRadioButton()
        font = QtGui.QFont()
        font.setPointSize(11)
        font.setBold(True)
        radio.setFont(font)
        radio.setText(text)
        return radio

    def _create_emergency_button(self):
        """Create the emergency stop button."""
        button = QtWidgets.QPushButton()
        button.setMinimumSize(150, 70)
        
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        button.setFont(font)
        
        button.setStyleSheet(self._get_emergency_button_style())
        button.setText("EMERGENCY STOP")
        
        return button

    # Style Sheet Methods
    
    @staticmethod
    def _get_control_button_style():
        """Get the stylesheet for control buttons."""
        return """
            QPushButton {
                background-color: #2d2d2d;
                color: #ffffff;
                border: 2px solid #ffffff;
                border-radius: 4px;
                padding: 5px;
            }
            QPushButton:hover {
                background-color: #3d3d3d;
                color: #ffffff;
            }
            QPushButton:pressed {
                background-color: #1d1d1d;
            }
        """

    @staticmethod
    def _get_key_button_style():
        """Get the stylesheet for WASD key buttons."""
        return """
            QPushButton {
                background-color: #ffffff;
                color: #000000;
                border: 2px solid #2d2d2d;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #f0f0f0;
            }
            QPushButton:pressed {
                background-color: #d0d0d0;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #888888;
                border: 2px solid #999999;
            }
        """

    @staticmethod
    def _get_emergency_button_style():
        """Get the stylesheet for emergency button."""
        return """
            QPushButton {
                background-color: #ff3b3b;
                color: #ffffff;
                border: 3px solid #cc0000;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #ff5555;
            }
            QPushButton:pressed {
                background-color: #cc0000;
            }
        """

    def retranslateUi(self, MainWindow):
        """Set up UI text translations."""
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "AMR Robot Navigation System"))

# =========================================================
# MAIN APPLICATION CLASS
# =========================================================

class AMRMainWindow(QMainWindow):
    """Main application window combining UI and ROS logic."""
    
    def __init__(self, model, ingest):
        super().__init__()
        
        self.model = model
        self.ingest = ingest
        
        # Setup UI
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        # Initialize PyQtGraph in map container
        self._setup_pyqtgraph_map()
        
        # Robot state
        self.manual_mode = False
        self.localization_mode = False
        self.waypoint_mode = False
        self.autonomous_running = False
        self.mapping_active = False
        
        # Teleop variables
        self.x = 0
        self.th = 0
        self.speed = 0.5
        self.turn = 1.0
        
        # Waypoint data
        self.waypoints = []
        self.active_wp_index = -1
        self.global_path = None
        
        # Localization
        self.loc_start = None
        self.loc_arrow = None
        
        # ROS Interfaces
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.initpose_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1
        )
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1
        )
        self.cancel_pub = rospy.Publisher(
            "/move_base/cancel", GoalID, queue_size=1
        )
        
        rospy.Subscriber(
            "/move_base/result",
            MoveBaseActionResult,
            self._goal_result_cb,
            queue_size=1
        )
        rospy.Subscriber(
            "/move_base/NavfnROS/plan",
            Path,
            self._global_path_cb,
            queue_size=1
        )
        
        # Process management
        self.ros_processes = []
        self.bringup_proc = None
        
        # Connect signals
        self._connect_signals()
        
        # Setup timers
        self.draw_timer = QTimer()
        self.draw_timer.timeout.connect(self.draw)
        self.draw_timer.start(50)  # 20 Hz
        
        self.cmd_timer = QTimer()
        self.cmd_timer.timeout.connect(self.publish_cmd)
        self.cmd_timer.start(50)  # 20 Hz - matching reference implementation
        
        # Set focus policy
        self.setFocusPolicy(Qt.StrongFocus)
        
        # Initialize WASD buttons as disabled
        self.ui.keyW.setEnabled(False)
        self.ui.keyA.setEnabled(False)
        self.ui.keyS.setEnabled(False)
        self.ui.keyD.setEnabled(False)
        
        # Status bar
        self.status_label = QtWidgets.QLabel("Status: Initializing...")
        self.statusBar().addPermanentWidget(self.status_label)

    def _setup_pyqtgraph_map(self):
        """Setup PyQtGraph widget for map visualization."""
        # Create plot widget with custom viewbox
        self.view_box = LocalizeViewBox(self)
        self.plot = pg.PlotWidget(viewBox=self.view_box)
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        
        # Add to map container
        layout = QtWidgets.QVBoxLayout(self.ui.mapContainer)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.plot)
        
        # Create plot items
        self.map_item = pg.ImageItem()
        self.scan_item = pg.ScatterPlotItem(pen=None, brush=pg.mkBrush(255, 0, 0, 180), size=3)
        self.fp_item = pg.PlotCurveItem(pen=pg.mkPen('g', width=2))
        self.robot_item = pg.ScatterPlotItem(
            pen=pg.mkPen('b', width=2),
            brush=pg.mkBrush(0, 0, 255, 150),
            size=15,
            symbol='t'
        )
        self.path_item = pg.PlotCurveItem(pen=pg.mkPen('c', width=2))
        self.wp_scatter = pg.ScatterPlotItem(
            pen=None,
            brush=pg.mkBrush(0, 0, 255, 200),
            size=10
        )
        
        # Add items to plot
        self.plot.addItem(self.map_item)
        self.plot.addItem(self.scan_item)
        self.plot.addItem(self.fp_item)
        self.plot.addItem(self.path_item)
        self.plot.addItem(self.wp_scatter)
        self.plot.addItem(self.robot_item)

    def _connect_signals(self):
        """Connect UI button signals to handlers."""
        self.ui.buttonMenu.clicked.connect(self.show_menu)
        self.ui.buttonCreateMap.clicked.connect(self.start_mapping)
        self.ui.buttonLoadMap.clicked.connect(self.load_map)
        self.ui.buttonSetMap.clicked.connect(self.toggle_localization)
        
        self.ui.buttonCreateWp.clicked.connect(self.toggle_waypoint_mode)
        self.ui.buttonSetWp.clicked.connect(self.save_waypoints)
        self.ui.buttonLoadWp.clicked.connect(self.load_waypoints)
        self.ui.buttonWpList.clicked.connect(self.show_waypoint_list)
        
        self.ui.buttonSelectPoint.clicked.connect(self.select_single_goal)
        self.ui.buttonPointList.clicked.connect(self.show_point_list)
        
        self.ui.radioButtonManual.toggled.connect(self.toggle_manual)
        self.ui.radioButtonAuto.toggled.connect(self.toggle_autonomous)
        
        self.ui.buttonEmergency.clicked.connect(self.emergency_stop)
        
        # WASD buttons - using pressed and released signals
        self.ui.keyW.pressed.connect(lambda: self.on_wasd_pressed('w'))
        self.ui.keyA.pressed.connect(lambda: self.on_wasd_pressed('a'))
        self.ui.keyS.pressed.connect(lambda: self.on_wasd_pressed('s'))
        self.ui.keyD.pressed.connect(lambda: self.on_wasd_pressed('d'))
        
        self.ui.keyW.released.connect(self.on_wasd_released)
        self.ui.keyA.released.connect(self.on_wasd_released)
        self.ui.keyS.released.connect(self.on_wasd_released)
        self.ui.keyD.released.connect(self.on_wasd_released)

    # =========================================================
    # ROS CALLBACKS
    # =========================================================

    def _goal_result_cb(self, msg):
        """Handle goal completion results."""
        if not self.autonomous_running:
            return
            
        if msg.status.status == 3:  # SUCCEEDED
            if self.active_wp_index < len(self.waypoints) - 1:
                self.active_wp_index += 1
                self._send_goal(self.waypoints[self.active_wp_index])
            else:
                self.autonomous_running = False
                self.active_wp_index = -1
                self.update_status("Navigation completed!")

    def _global_path_cb(self, msg):
        """Handle global path updates."""
        if msg.poses:
            xs = [p.pose.position.x for p in msg.poses]
            ys = [p.pose.position.y for p in msg.poses]
            self.global_path = (xs, ys)

    # =========================================================
    # BUTTON HANDLERS
    # =========================================================

    def show_menu(self):
        """Show main menu."""
        QMessageBox.information(self, "Menu", "AMR Robot Navigation System\nVersion 1.0")

    def start_mapping(self):
        """Start SLAM mapping mode."""
        # Stop navigation stack
        subprocess.call("rosnode kill /map_server", shell=True)
        subprocess.call("rosnode kill /amcl", shell=True)
        subprocess.call("rosnode kill /move_base", shell=True)
        
        # Clear UI state
        self.model.map_img = None
        self.model.map_extent = None
        
        # Start SLAM
        slam_proc = subprocess.Popen(
            ["roslaunch", "stark", "lidar_slam.launch"],
            preexec_fn=os.setsid
        )
        self.ros_processes.append(slam_proc)
        
        rospy.sleep(1.5)
        self.mapping_active = True
        self.update_status("Mapping mode active")

    def load_map(self):
        """Load existing map and start navigation."""
        name, ok = QInputDialog.getText(self, "Load Map", "Map name:")
        if not ok or not name:
            return
        
        # Kill old nodes
        subprocess.call("rosnode kill /slam_gmapping", shell=True)
        subprocess.call("rosnode kill /map_server", shell=True)
        subprocess.call("rosnode kill /amcl", shell=True)
        subprocess.call("rosnode kill /move_base", shell=True)
        
        # Start navigation
        nav_proc = subprocess.Popen(
            ["roslaunch", "stark", "navigate.launch", f"map_file:={name}"],
            preexec_fn=os.setsid
        )
        self.ros_processes.append(nav_proc)
        
        rospy.sleep(2.0)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.initpose_pub.publish(msg)    
        self.mapping_active = False
        self.update_status("Navigation mode active")

    def toggle_localization(self):
        """Toggle localization mode for setting initial pose."""
        self.localization_mode = not self.localization_mode
        if self.localization_mode:
            self.update_status("Click and drag to set initial pose")
        else:
            self.update_status("Localization mode off")

    def toggle_waypoint_mode(self):
        """Toggle waypoint creation mode."""
        self.waypoint_mode = not self.waypoint_mode
        if self.waypoint_mode:
            self.update_status("Click on map to add waypoints")
        else:
            self.update_status("Waypoint mode off")

    def save_waypoints(self):
        """Save current waypoints to file."""
        if not self.waypoints:
            QMessageBox.warning(self, "Warning", "No waypoints to save!")
            return
            
        name, ok = QInputDialog.getText(self, "Save Waypoints", "Waypoint file name:")
        if ok and name:
            # Save waypoints logic here
            self.update_status(f"Waypoints saved as {name}")

    def load_waypoints(self):
        """Load waypoints from file."""
        name, ok = QInputDialog.getText(self, "Load Waypoints", "Waypoint file name:")
        if ok and name:
            # Load waypoints logic here
            self.update_status(f"Loaded waypoints from {name}")

    def show_waypoint_list(self):
        """Show list of current waypoints."""
        if not self.waypoints:
            QMessageBox.information(self, "Waypoints", "No waypoints defined")
        else:
            wp_text = "\n".join([f"{i}: ({w[0]:.2f}, {w[1]:.2f})" 
                                for i, w in enumerate(self.waypoints)])
            QMessageBox.information(self, "Waypoint List", wp_text)

    def select_single_goal(self):
        """Enable single goal selection mode."""
        self.update_status("Click on map to select goal")

    def show_point_list(self):
        """Show list of saved points."""
        QMessageBox.information(self, "Points", "Point list feature coming soon")

    def toggle_manual(self, checked):
        """Toggle manual control mode."""
        if checked:
            self.manual_mode = True
            self.autonomous_running = False
            self.update_status("🎮 Manual control active - Use WASD keys")
            
            # Enable WASD buttons
            self.ui.keyW.setEnabled(True)
            self.ui.keyA.setEnabled(True)
            self.ui.keyS.setEnabled(True)
            self.ui.keyD.setEnabled(True)
        else:
            self.manual_mode = False
            self.update_status("Manual control disabled")
            
            # Disable WASD buttons
            self.ui.keyW.setEnabled(False)
            self.ui.keyA.setEnabled(False)
            self.ui.keyS.setEnabled(False)
            self.ui.keyD.setEnabled(False)
            
            # Stop robot
            self.x = 0
            self.th = 0
            self.publish_cmd()

    def toggle_autonomous(self, checked):
        """Toggle autonomous navigation mode."""
        if checked:
            if not self.waypoints:
                QMessageBox.warning(self, "Warning", "No waypoints defined!")
                self.ui.radioButtonAuto.setChecked(False)
                return
                
            self.autonomous_running = True
            self.manual_mode = False
            self.active_wp_index = 0
            self._send_goal(self.waypoints[0])
            self.update_status("Autonomous navigation active")
        else:
            self.autonomous_running = False
            self.cancel_pub.publish(GoalID())
            self.update_status("Autonomous navigation stopped")

    def emergency_stop(self):
        """Emergency stop - cancel all goals and stop robot."""
        self.cancel_pub.publish(GoalID())
        
        # Stop robot immediately
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_pub.publish(twist)
        
        self.autonomous_running = False
        self.manual_mode = False
        self.x = 0
        self.th = 0
        
        self.ui.radioButtonManual.setChecked(False)
        self.ui.radioButtonAuto.setChecked(False)
        
        self.update_status("EMERGENCY STOP ACTIVATED")

    # =========================================================
    # TELEOP FUNCTIONS
    # =========================================================

    def keyPressEvent(self, e):
        """Handle keyboard input for manual control."""
        if not self.manual_mode:
            return
            
        k = e.key()
        if k == Qt.Key_W:
            self.x = 1    # Forward
            self.th = 0
        elif k == Qt.Key_S:
            self.x = -1   # Backward
            self.th = 0
        elif k == Qt.Key_A:
            self.x = 0
            self.th = 1   # Turn left
        elif k == Qt.Key_D:
            self.x = 0
            self.th = -1  # Turn right
        elif k in SPEED_BINDINGS:
            self.speed *= SPEED_BINDINGS[k][0]
            self.turn *= SPEED_BINDINGS[k][1]

    def keyReleaseEvent(self, e):
        """Handle keyboard release for manual control."""
        if self.manual_mode:
            k = e.key()
            if k in [Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D]:
                self.x = 0
                self.th = 0

    def on_wasd_pressed(self, key):
        """Handle WASD button press."""
        print(f"DEBUG: WASD pressed - key={key}, manual_mode={self.manual_mode}")
        
        if not self.manual_mode:
            self.update_status("⚠️ Enable Manual mode first!")
            return
        
        # Map key to movement (matching reference implementation)
        if key == 'w':
            self.x = 1    # Forward
            self.th = 0
            self.update_status("⬆️ Moving Forward")
            print(f"DEBUG: Set x={self.x}, th={self.th}")
        elif key == 's':
            self.x = -1   # Backward (Reverse)
            self.th = 0
            self.update_status("⬇️ Moving Backward")
            print(f"DEBUG: Set x={self.x}, th={self.th}")
        elif key == 'a':
            self.x = 0
            self.th = 1   # Turn Left
            self.update_status("⬅️ Turning Left")
            print(f"DEBUG: Set x={self.x}, th={self.th}")
        elif key == 'd':
            self.x = 0
            self.th = -1  # Turn Right
            self.update_status("➡️ Turning Right")
            print(f"DEBUG: Set x={self.x}, th={self.th}")

    def on_wasd_released(self):
        """Handle WASD button release."""
        print(f"DEBUG: WASD released")
        if self.manual_mode:
            self.x = 0
            self.th = 0
            self.update_status("Manual control active - Stopped")
            print(f"DEBUG: Stop - x={self.x}, th={self.th}")

    def virtual_key_press(self, key):
        """Handle virtual button press (deprecated - use on_wasd_pressed)."""
        self.on_wasd_pressed(key)

    def virtual_key_release(self):
        """Handle virtual button release (deprecated - use on_wasd_released)."""
        self.on_wasd_released()

    def publish_cmd(self):
        """Publish velocity commands."""
        if not self.manual_mode:
            return
        
        # Only print if there's actual movement
        if self.x != 0 or self.th != 0:
            print(f"DEBUG: Publishing cmd_vel - linear.x={self.x * self.speed}, angular.z={self.th * self.turn}")
            
        twist = Twist()
        twist.linear.x = self.x * self.speed
        twist.angular.z = self.th * self.turn
        self.cmd_pub.publish(twist)

    # =========================================================
    # LOCALIZATION FUNCTIONS
    # =========================================================

    def _loc_start_press(self, pos):
        """Start localization drag."""
        self.loc_start = (pos.x(), pos.y())

    def _loc_drag(self, pos):
        """Update localization arrow during drag."""
        if self.loc_start:
            # Draw arrow from start to current position
            pass

    def _loc_release(self, pos):
        """Complete localization and publish initial pose."""
        if self.loc_start:
            x0, y0 = self.loc_start
            x1, y1 = pos.x(), pos.y()
            
            yaw = np.arctan2(y1 - y0, x1 - x0)
            
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = rospy.Time.now()
            msg.pose.pose.position.x = x0
            msg.pose.pose.position.y = y0
            
            q = tft.quaternion_from_euler(0, 0, yaw)
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]
            
            self.initpose_pub.publish(msg)
            self.loc_start = None
            self.update_status("Initial pose set")

    # =========================================================
    # GOAL SENDING
    # =========================================================

    def _send_goal(self, waypoint):
        """Send navigation goal."""
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = waypoint[0]
        msg.pose.position.y = waypoint[1]
        msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(msg)

    # =========================================================
    # VISUALIZATION
    # =========================================================

    def draw(self):
        """Update visualization."""
        self.ingest.update_robot_pose()
        m, ext, scan, fp, pose = self.model.snapshot()

        if m is not None:
            self.map_item.setImage(m, autoLevels=False)
            self.map_item.setRect(QRectF(
                ext[0], ext[2],
                ext[1] - ext[0], ext[3] - ext[2]
            ))

        if scan is not None:
            self.scan_item.setData(scan[:, 0], scan[:, 1])

        if fp is not None:
            f = np.vstack([fp, fp[0]])
            self.fp_item.setData(f[:, 0], f[:, 1])

        if pose is not None:
            x, y, yaw = pose
            self.robot_item.setPos(x, y)
            self.robot_item.setStyle(angle=-np.degrees(yaw))

        if self.global_path is not None:
            xs, ys = self.global_path
            self.path_item.setData(xs, ys)

        if self.waypoints:
            xs = [w[0] for w in self.waypoints]
            ys = [w[1] for w in self.waypoints]
            self.wp_scatter.setData(xs, ys)

    # =========================================================
    # UTILITIES
    # =========================================================

    def update_status(self, message):
        """Update status bar message."""
        self.status_label.setText(f"Status: {message}")

    def start_bringup(self):
        """Start robot bringup."""
        self.bringup_proc = subprocess.Popen(
            ["roslaunch", "stark", "bringup.launch"],
            preexec_fn=os.setsid
        )
        self.ros_processes.append(self.bringup_proc)
        self.update_status("Robot initialized")

    def closeEvent(self, event):
        """Clean up on window close."""
        # Stop all ROS processes
        for proc in self.ros_processes:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except:
                pass
        
        event.accept()

# =========================================================
# ROSCORE MANAGEMENT
# =========================================================

def ensure_roscore():
    """Ensure roscore is running."""
    try:
        rospy.get_master().getSystemState()
        return
    except:
        pass
    
    subprocess.Popen(
        ["roscore"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    
    while True:
        try:
            rospy.get_master().getSystemState()
            break
        except:
            time.sleep(0.2)

# =========================================================
# MAIN
# =========================================================

def main():
    """Main application entry point."""
    ensure_roscore()
    rospy.init_node("amr_ui", anonymous=True, disable_signals=True)
    
    app = QApplication(sys.argv)
    
    model = WorldModel()
    ingest = ROSIngest(model)
    window = AMRMainWindow(model, ingest)
    
    window.start_bringup()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()