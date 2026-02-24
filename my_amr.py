#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AMR Robot Navigation System - Zahra - Optimized & Crash-Free"""

import sys, threading, subprocess, numpy as np, rospy, yaml, time, os, signal, traceback
from geometry_msgs.msg import PoseStamped, PolygonStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
import tf2_ros, tf.transformations as tft
from actionlib_msgs.msg import GoalID
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, QRectF, Qt
from PyQt5.QtGui import QFont
import pyqtgraph as pg

MAP_DIR = "/home/delivery/catkin_ws/src/stark/maps"
MOVE_BINDINGS = {Qt.Key_W:(1,0,0,0), Qt.Key_S:(-1,0,0,0), Qt.Key_A:(0,0,0,1), Qt.Key_D:(0,0,0,-1)}
SPEED_BINDINGS = {Qt.Key_Q:(1.1,1.1), Qt.Key_Z:(0.9,0.9), Qt.Key_E:(1.0,1.1), Qt.Key_C:(1.0,0.9)}

class WorldModel:
    def __init__(self):
        self.lock = threading.Lock()
        self.map_img = self.map_extent = self.scan_pts = self.footprint_pts = self.robot_pose = None
    def snapshot(self):
        with self.lock: return (self.map_img, self.map_extent, self.scan_pts, self.footprint_pts, self.robot_pose)

class ROSIngest:
    def __init__(self, model):
        self.model = model
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=1)
        self.fp_sub = rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, self.footprint_cb, queue_size=1)
    
    def map_cb(self, msg):
        try:
            h, w, res = msg.info.height, msg.info.width, msg.info.resolution
            ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
            data = np.array(msg.data, dtype=np.int8).reshape(h, w)
            img = np.zeros_like(data, dtype=np.uint8)
            img[data==-1], img[data==0], img[data>0] = 100, 255, 0
            with self.model.lock: self.model.map_img, self.model.map_extent = img, (ox, ox+w*res, oy, oy+h*res)
        except: pass
    
    def update_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(0.05))
            q = tf.transform.rotation
            yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            with self.model.lock: self.model.robot_pose = (tf.transform.translation.x, tf.transform.translation.y, yaw)
        except: pass
    
    def scan_cb(self, msg):
        try:
            target = "map" if self.model.map_img is not None else "odom"
            tf = self.tf_buffer.lookup_transform(target, msg.header.frame_id, rospy.Time(0), rospy.Duration(0.05))
            ranges, angles = np.array(msg.ranges), msg.angle_min + np.arange(len(msg.ranges))*msg.angle_increment
            valid = np.isfinite(ranges) & (ranges > 0.1)
            xs, ys = ranges[valid]*np.cos(angles[valid]), ranges[valid]*np.sin(angles[valid])
            T = tft.quaternion_matrix([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w])
            T[0,3], T[1,3] = tf.transform.translation.x, tf.transform.translation.y
            pts = np.vstack([xs, ys, np.zeros_like(xs), np.ones_like(xs)])
            with self.model.lock: self.model.scan_pts = (T @ pts)[:2].T
        except: pass
    
    def footprint_cb(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform("map", msg.header.frame_id, rospy.Time(0), rospy.Duration(0.05))
            T = tft.quaternion_matrix([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w])
            T[0,3], T[1,3] = tf.transform.translation.x, tf.transform.translation.y
            pts = np.array([[p.x, p.y, 0, 1] for p in msg.polygon.points]).T
            with self.model.lock: self.model.footprint_pts = (T @ pts)[:2].T
        except: pass

class LocalizeViewBox(pg.ViewBox):
    def __init__(self, ui):
        super().__init__()
        self.ui, self.dragging = ui, False
    def mousePressEvent(self, ev):
        if self.ui.localization_mode and ev.button() == Qt.LeftButton:
            self.ui._loc_start_press(self.mapSceneToView(ev.scenePos()))
            self.dragging = True
            ev.accept()
        else: super().mousePressEvent(ev)
    def mouseDragEvent(self, ev, axis=None):
        if self.ui.localization_mode and self.dragging:
            self.ui._loc_drag(self.mapSceneToView(ev.scenePos()))
            ev.accept()
        else: super().mouseDragEvent(ev, axis)
    def mouseReleaseEvent(self, ev):
        if self.ui.localization_mode and self.dragging:
            self.ui._loc_release(self.mapSceneToView(ev.scenePos()))
            self.dragging = False
            ev.accept()
        else: super().mouseReleaseEvent(ev)

class AMRMainWindow(QMainWindow):
    def __init__(self, model, ingest):
        super().__init__()
        self.model, self.ingest = model, ingest
        self.manual_mode = self.localization_mode = self.waypoint_mode = self.autonomous_running = self.autonomous_loop = self.mapping_active = False
        self.x = self.th = 0
        self.speed, self.turn = 0.5, 1.0
        self.waypoints, self.active_wp_index, self.global_path, self.wp_text_items, self.loc_start = [], -1, None, [], None
        self.ros_processes, self.bringup_proc = [], None
        self._setup_ui()
        self._setup_ros()
        self._connect_signals()
        self.draw_timer = QTimer()
        self.draw_timer.timeout.connect(self.draw)
        self.draw_timer.start(100)
        self.cmd_timer = QTimer()
        self.cmd_timer.timeout.connect(self.publish_cmd)
        self.cmd_timer.start(50)
        self.setFocusPolicy(Qt.StrongFocus)
        for key in [self.keyW, self.keyA, self.keyS, self.keyD]: key.setEnabled(False)
        self.status_label = QLabel("Status: Ready")
        self.statusBar().addPermanentWidget(self.status_label)
    
    def _setup_ui(self):
        self.setWindowTitle("AMR Navigation - Zahra")
        self.resize(1000, 650)
        central = QWidget()
        main_layout = QHBoxLayout(central)
        
        # Left panel
        ctrl_layout = QVBoxLayout()
        title = QLabel("Robot Navigasi")
        font = QFont()
        font.setPointSize(16)
        font.setBold(True)
        title.setFont(font)
        title.setAlignment(Qt.AlignCenter)
        ctrl_layout.addWidget(title)
        
        btn_style = "QPushButton{background:#2d2d2d;color:#fff;border:2px solid #fff;border-radius:4px;padding:5px}QPushButton:hover{background:#3d3d3d}QPushButton:pressed{background:#1d1d1d}"
        for name, text in [("Menu","Menu"),("CreateMap","Create Map"),("LoadMap","Load Map"),("SaveMap","Save Map"),("CreateWp","Create WP"),("SaveWp","Save WP"),("LoadWp","Load WP"),("StartAuto","Start Auto"),("ClearWp","Clear WP"),("StopAuto","Stop Auto")]:
            btn = QPushButton(text)
            btn.setMinimumHeight(32)
            btn.setStyleSheet(btn_style)
            setattr(self, f"btn{name}", btn)
            ctrl_layout.addWidget(btn)
        ctrl_layout.addStretch()
        main_layout.addLayout(ctrl_layout, 1)
        
        # Right panel
        right_layout = QVBoxLayout()
        map_title = QLabel("Map View")
        font.setPointSize(14)
        map_title.setFont(font)
        map_title.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(map_title)
        
        # Map
        map_container = QWidget()
        map_container.setMinimumSize(600, 350)
        map_container.setStyleSheet("background:#f0f0f0;border:2px solid #2d2d2d;border-radius:4px")
        self.view_box = LocalizeViewBox(self)
        self.plot = pg.PlotWidget(viewBox=self.view_box)
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.getViewBox().invertY(True)
        self.plot.setAntialiasing(False)
        map_layout = QVBoxLayout(map_container)
        map_layout.setContentsMargins(0,0,0,0)
        map_layout.addWidget(self.plot)
        
        # Plot items
        self.map_item = pg.ImageItem()
        self.map_item.setOpts(axisOrder="row-major")
        self.scan_item = pg.ScatterPlotItem(size=3, brush=pg.mkBrush(255,0,0), pxMode=True)
        self.fp_item = pg.PlotDataItem(pen=pg.mkPen('g', width=2))
        self.robot_item = pg.ArrowItem(headLen=0.3, tailLen=0.2, brush="y")
        self.path_item = pg.PlotDataItem(pen=pg.mkPen(color=(0,0,255), width=2))
        self.wp_scatter = pg.ScatterPlotItem(size=12, brush=pg.mkBrush(0,0,255), pen=pg.mkPen("w"), pxMode=True)
        for item in [self.map_item, self.scan_item, self.fp_item, self.path_item, self.wp_scatter, self.robot_item]:
            self.plot.addItem(item)
        self.mode_label = pg.TextItem("MODE: IDLE", color="y", anchor=(0,1))
        self.mode_label.setPos(0,0)
        self.plot.addItem(self.mode_label)
        watermark = pg.TextItem("Zahra", color=(255,255,255,120), anchor=(1,0))
        font.setPointSize(24)
        watermark.setFont(font)
        self.plot.addItem(watermark)
        self.watermark = watermark
        self.loc_line = pg.PlotDataItem(pen=pg.mkPen(color=(200,0,200), width=2))
        self.plot.addItem(self.loc_line)
        self.plot.scene().sigMouseClicked.connect(self._on_map_click)
        right_layout.addWidget(map_container, 1)
        
        # Controls
        nav_layout = QHBoxLayout()
        left_nav = QVBoxLayout()
        radio_layout = QHBoxLayout()
        self.radioManual = QRadioButton("Manual")
        radio_layout.addWidget(self.radioManual)
        radio_layout.addStretch()
        left_nav.addLayout(radio_layout)
        
        wasd_layout = QGridLayout()
        key_style = "QPushButton{background:#fff;color:#000;border:2px solid #2d2d2d;border-radius:4px}QPushButton:hover{background:#f0f0f0}QPushButton:pressed{background:#d0d0d0}QPushButton:disabled{background:#ccc;color:#888}"
        self.keyW, self.keyA, self.keyS, self.keyD = QPushButton("W"), QPushButton("A"), QPushButton("S"), QPushButton("D")
        for k in [self.keyW, self.keyA, self.keyS, self.keyD]:
            k.setFixedSize(45,45)
            k.setStyleSheet(key_style)
        wasd_layout.addWidget(self.keyW, 0, 1)
        wasd_layout.addWidget(self.keyA, 1, 0)
        wasd_layout.addWidget(self.keyS, 1, 1)
        wasd_layout.addWidget(self.keyD, 1, 2)
        left_nav.addLayout(wasd_layout)
        left_nav.addStretch()
        nav_layout.addLayout(left_nav, 1)
        
        self.btnEmergency = QPushButton("EMERGENCY")
        self.btnEmergency.setMinimumSize(150,70)
        self.btnEmergency.setStyleSheet("QPushButton{background:#ff3b3b;color:#fff;border:3px solid #c00;border-radius:6px}QPushButton:hover{background:#ff5555}QPushButton:pressed{background:#c00}")
        nav_layout.addWidget(self.btnEmergency)
        right_layout.addLayout(nav_layout)
        
        main_layout.addLayout(right_layout, 3)
        self.setCentralWidget(central)
    
    def _setup_ros(self):
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.initpose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.result_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self._on_nav_result, queue_size=1)
        self.path_sub = rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self._on_global_path, queue_size=1)
    
    def _connect_signals(self):
        self.btnMenu.clicked.connect(lambda: QMessageBox.information(self, "Menu", "AMR Navigation - Zahra v1.0"))
        self.btnCreateMap.clicked.connect(self.start_mapping)
        self.btnLoadMap.clicked.connect(self.load_map)
        self.btnSaveMap.clicked.connect(self.save_map)
        self.btnCreateWp.clicked.connect(self.enable_wp_mode)
        self.btnSaveWp.clicked.connect(self.save_waypoints)
        self.btnLoadWp.clicked.connect(self.load_waypoints)
        self.btnStartAuto.clicked.connect(self.start_autonomous)
        self.btnClearWp.clicked.connect(self.clear_waypoints)
        self.btnStopAuto.clicked.connect(self.stop_autonomous)
        self.radioManual.toggled.connect(self.toggle_manual)
        self.btnEmergency.clicked.connect(self.emergency_stop)
        self.keyW.pressed.connect(lambda: self.on_wasd_pressed('w'))
        self.keyA.pressed.connect(lambda: self.on_wasd_pressed('a'))
        self.keyS.pressed.connect(lambda: self.on_wasd_pressed('s'))
        self.keyD.pressed.connect(lambda: self.on_wasd_pressed('d'))
        for k in [self.keyW, self.keyA, self.keyS, self.keyD]: k.released.connect(self.on_wasd_released)
    
    def _on_nav_result(self, msg):
        try:
            if not self.autonomous_running or msg.status.status != 3: return
            self.active_wp_index += 1
            if self.active_wp_index >= len(self.waypoints):
                if self.autonomous_loop:
                    self.active_wp_index = 0
                    self._send_goal(self.waypoints[0])
                else:
                    self.autonomous_running, self.active_wp_index = False, -1
                    if self.waypoints: self.wp_scatter.setBrush([pg.mkBrush(0,0,255)]*len(self.waypoints))
                    self.global_path = None
                    self.path_item.clear()
                    self.mode_label.setText("MODE: IDLE", color="y")
                    self.update_status("Done!")
                    return
            self._send_goal(self.waypoints[self.active_wp_index])
        except: pass
    
    def _on_global_path(self, msg):
        try: self.global_path = ([p.pose.position.x for p in msg.poses], [p.pose.position.y for p in msg.poses]) if msg.poses else None
        except: pass
    
    def start_mapping(self):
        try:
            for n in ["/map_server","/amcl","/move_base","/relay_scan"]: subprocess.call(f"rosnode kill {n}", shell=True)
            self.model.map_img = self.model.map_extent = self.model.robot_pose = None
            slam_proc = subprocess.Popen(["roslaunch","stark","lidar_slam.launch"], preexec_fn=os.setsid)
            self.ros_processes.append(slam_proc)
            rospy.sleep(1.5)
            self.autonomous_running, self.active_wp_index, self.mapping_active = False, -1, True
            self.update_mode_label()
            self.update_status("Mapping active")
        except Exception as e: QMessageBox.critical(self, "Error", f"Mapping failed: {e}")
    
    def load_map(self):
        try:
            name, ok = QInputDialog.getText(self, "Load Map", "Map name:")
            if not ok or not name: return
            for n in ["/slam_gmapping","/map_server","/amcl","/move_base","/relay_scan"]: subprocess.call(f"rosnode kill {n}", shell=True)
            nav_proc = subprocess.Popen(["roslaunch","stark","navigate.launch",f"map_file:={name}"], preexec_fn=os.setsid)
            self.ros_processes.append(nav_proc)
            rospy.sleep(2.0)
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id, msg.header.stamp = "map", rospy.Time.now()
            msg.pose.pose.position.x = msg.pose.pose.position.y = 0.0
            msg.pose.pose.orientation.w = 1.0
            self.initpose_pub.publish(msg)
            self.mapping_active = False
            self.update_mode_label()
            self.update_status("Nav active")
        except Exception as e: QMessageBox.critical(self, "Error", f"Load failed: {e}")
    
    def save_map(self):
        try:
            name, ok = QInputDialog.getText(self, "Save Map", "Map name:")
            if ok and name:
                save_proc = subprocess.Popen(["rosrun","map_server","map_saver","-f",name], cwd=MAP_DIR, preexec_fn=os.setsid)
                self.ros_processes.append(save_proc)
                self.update_status(f"Saved: {name}")
        except Exception as e: QMessageBox.critical(self, "Error", f"Save failed: {e}")
    
    def enable_wp_mode(self):
        self.localization_mode = False
        self.plot.getViewBox().setMouseEnabled(x=True, y=True)
        self.waypoint_mode = not self.waypoint_mode
        self.manual_mode = False
        if self.waypoint_mode:
            self.mode_label.setText("MODE: WAYPOINT", color="c")
            self.update_status("Click to add WP")
        else:
            self.mode_label.setText("MODE: IDLE", color="y")
            self.update_status("WP mode OFF")
    
    def save_waypoints(self):
        try:
            if not self.waypoints: return QMessageBox.warning(self, "Warning", "No WP!")
            name, ok = QInputDialog.getText(self, "Save WP", "File name:")
            if ok and name:
                with open(f"{MAP_DIR}/{name}_wps.yaml", "w") as f: yaml.dump({"waypoints": self.waypoints}, f)
                self.update_status(f"WP saved: {name}")
        except Exception as e: QMessageBox.critical(self, "Error", f"Save WP failed: {e}")
    
    def load_waypoints(self):
        try:
            name, ok = QInputDialog.getText(self, "Load WP", "File name:")
            if not ok or not name: return
            with open(f"{MAP_DIR}/{name}_wps.yaml") as f: data = yaml.safe_load(f)
            for t in self.wp_text_items: self.plot.removeItem(t)
            self.wp_text_items.clear()
            self.waypoints = data["waypoints"]
            xs, ys = [], []
            for i, (x, y) in enumerate(self.waypoints, 1):
                xs.append(x)
                ys.append(y)
                idx_label = pg.TextItem(str(i), color="k", anchor=(0.5,1.2))
                idx_label.setPos(x, y)
                self.plot.addItem(idx_label)
                coord_label = pg.TextItem(f"({x},{y})", color="k", anchor=(0.5,-0.2))
                coord_label.setPos(x, y)
                self.plot.addItem(coord_label)
                self.wp_text_items.extend([idx_label, coord_label])
            self.wp_scatter.setData(xs, ys)
            self.update_status(f"Loaded {len(self.waypoints)} WP")
        except Exception as e: QMessageBox.critical(self, "Error", f"Load WP failed: {e}")
    
    def start_autonomous(self):
        if not self.waypoints: return QMessageBox.warning(self, "Warning", "No WP!")
        self.autonomous_running, self.active_wp_index = True, 0
        self._send_goal(self.waypoints[0])
        self.update_status("Auto started")
    
    def clear_waypoints(self):
        self.waypoints.clear()
        self.wp_scatter.clear()
        self.active_wp_index, self.autonomous_running = -1, False
        for t in self.wp_text_items: self.plot.removeItem(t)
        self.wp_text_items.clear()
        self.mode_label.setText("MODE: IDLE", color="y")
        self.update_status("WP cleared")
    
    def stop_autonomous(self):
        self.autonomous_running, self.active_wp_index = False, -1
        self.cancel_pub.publish(GoalID())
        if self.waypoints: self.wp_scatter.setBrush([pg.mkBrush(0,0,255)]*len(self.waypoints))
        self.global_path = None
        self.path_item.clear()
        self.mode_label.setText("MODE: IDLE", color="y")
        self.update_status("Auto stopped")
    
    def toggle_manual(self, checked):
        if checked:
            self.manual_mode, self.autonomous_running, self.waypoint_mode, self.localization_mode = True, False, False, False
            self.update_status("Manual - WASD")
            self.update_mode_label()
            for k in [self.keyW, self.keyA, self.keyS, self.keyD]: k.setEnabled(True)
        else:
            self.manual_mode = False
            self.update_status("Manual OFF")
            self.update_mode_label()
            for k in [self.keyW, self.keyA, self.keyS, self.keyD]: k.setEnabled(False)
            self.x = self.th = 0
            self.publish_cmd()
    
    def emergency_stop(self):
        try:
            self.cancel_pub.publish(GoalID())
            self.cmd_pub.publish(Twist())
            self.autonomous_running = self.manual_mode = False
            self.x = self.th = 0
            self.radioManual.setChecked(False)
            self.update_status("EMERGENCY STOP")
        except: pass
    
    def keyPressEvent(self, e):
        if not self.manual_mode: return
        k = e.key()
        if k in MOVE_BINDINGS: self.x, _, _, self.th = MOVE_BINDINGS[k]
        elif k in SPEED_BINDINGS: self.speed *= SPEED_BINDINGS[k][0]; self.turn *= SPEED_BINDINGS[k][1]
    
    def keyReleaseEvent(self, e):
        if self.manual_mode and e.key() in [Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D]: self.x = self.th = 0
    
    def on_wasd_pressed(self, key):
        if not self.manual_mode: return
        if key == 'w': self.x, self.th = 1, 0
        elif key == 's': self.x, self.th = -1, 0
        elif key == 'a': self.x, self.th = 0, 1
        elif key == 'd': self.x, self.th = 0, -1
    
    def on_wasd_released(self):
        if self.manual_mode: self.x = self.th = 0
    
    def publish_cmd(self):
        if not self.manual_mode: return
        try:
            twist = Twist()
            twist.linear.x, twist.angular.z = self.x * self.speed, self.th * self.turn
            self.cmd_pub.publish(twist)
        except: pass
    
    def _loc_start_press(self, pos):
        if not self.localization_mode: return
        self.loc_start = (pos.x(), pos.y())
        self.loc_line.setData([self.loc_start[0]], [self.loc_start[1]])
    
    def _loc_drag(self, pos):
        if not self.localization_mode or self.loc_start is None: return
        self.loc_line.setData([self.loc_start[0], pos.x()], [self.loc_start[1], pos.y()])
    
    def _loc_release(self, pos):
        if not self.localization_mode or self.loc_start is None: return
        try:
            dx, dy = pos.x() - self.loc_start[0], pos.y() - self.loc_start[1]
            if abs(dx) < 0.02 and abs(dy) < 0.02:
                self.loc_line.clear()
                self.loc_start = None
                return
            yaw = np.arctan2(dy, dx)
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id, msg.header.stamp = "map", rospy.Time.now()
            msg.pose.pose.position.x, msg.pose.pose.position.y = self.loc_start[0], self.loc_start[1]
            q = tft.quaternion_from_euler(0, 0, yaw)
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q[0], q[1], q[2], q[3]
            msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05
            msg.pose.covariance[35] = 0.1
            self.initpose_pub.publish(msg)
            self.loc_line.clear()
            self.loc_start = None
            self.update_status("Pose set")
        except: pass
    
    def _on_map_click(self, evt):
        if not self.waypoint_mode: return
        try:
            vb = self.plot.getViewBox()
            pos = evt.scenePos()
            if not vb.sceneBoundingRect().contains(pos): return
            p = vb.mapSceneToView(pos)
            x, y = round(p.x(), 3), round(p.y(), 3)
            self.waypoints.append([float(x), float(y)])
            xs, ys = [w[0] for w in self.waypoints], [w[1] for w in self.waypoints]
            self.wp_scatter.setData(xs, ys)
            idx = len(self.waypoints)
            idx_label = pg.TextItem(str(idx), color="k", anchor=(0.5,1.2))
            idx_label.setPos(x, y)
            self.plot.addItem(idx_label)
            coord_label = pg.TextItem(f"({x},{y})", color="k", anchor=(0.5,-0.2))
            coord_label.setPos(x, y)
            self.plot.addItem(coord_label)
            self.wp_text_items.extend([idx_label, coord_label])
            self.update_status(f"WP {idx} at ({x},{y})")
        except: pass
    
    def _send_goal(self, wp):
        try:
            x, y = wp
            brushes = [pg.mkBrush(255,255,0) if i == self.active_wp_index else pg.mkBrush(0,0,255) for i in range(len(self.waypoints))]
            self.wp_scatter.setBrush(brushes)
            g = PoseStamped()
            g.header.frame_id, g.header.stamp = "map", rospy.Time.now()
            g.pose.position.x, g.pose.position.y, g.pose.orientation.w = x, y, 1.0
            self.goal_pub.publish(g)
            self.update_status(f"WP {self.active_wp_index+1}/{len(self.waypoints)}")
        except: pass
    
    def draw(self):
        try:
            self.ingest.update_robot_pose()
            m, ext, scan, fp, pose = self.model.snapshot()
            if m is not None and ext is not None:
                self.map_item.setImage(m, autoLevels=False)
                self.map_item.setRect(QRectF(ext[0], ext[2], ext[1]-ext[0], ext[3]-ext[2]))
                self.watermark.setPos(ext[1], ext[3])
            if scan is not None and len(scan) > 0: self.scan_item.setData(scan[:,0], scan[:,1])
            if fp is not None and len(fp) > 0:
                f = np.vstack([fp, fp[0]])
                self.fp_item.setData(f[:,0], f[:,1])
            if pose is not None:
                x, y, yaw = pose
                self.robot_item.setPos(x, y)
                self.robot_item.setStyle(angle=-np.degrees(yaw))
            if self.global_path is not None:
                xs, ys = self.global_path
                if len(xs) > 0: self.path_item.setData(xs, ys)
            else: self.path_item.clear()
        except: pass
    
    def update_status(self, msg): self.status_label.setText(f"Status: {msg}")
    
    def update_mode_label(self):
        modes = ["MAPPING" if self.mapping_active else "NAV"]
        if self.manual_mode: modes.append("MANUAL")
        if self.waypoint_mode: modes = ["WAYPOINT"]
        self.mode_label.setText(f"MODE: {' + '.join(modes) if modes else 'IDLE'}", color="y")
    
    def start_bringup(self):
        try:
            if self.bringup_proc is not None: return
            self.update_status("Starting...")
            self.bringup_proc = subprocess.Popen(["roslaunch","stark","bringup.launch"], preexec_fn=os.setsid)
            self.ros_processes.append(self.bringup_proc)
            timeout = rospy.Time.now() + rospy.Duration(10.0)
            while not rospy.is_shutdown():
                topics = rospy.get_published_topics()
                names = [t[0] for t in topics]
                if "/odom" in names and "/scan" in names: break
                if rospy.Time.now() > timeout: break
                rospy.sleep(0.2)
            self.update_status("Ready")
        except Exception as e: QMessageBox.critical(self, "Error", f"Bringup failed: {e}")
    
    def closeEvent(self, event):
        try:
            self.draw_timer.stop()
            self.cmd_timer.stop()
            for proc in self.ros_processes:
                try: os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except: pass
            subprocess.call("rosnode kill -a", shell=True)
            rospy.signal_shutdown("UI closed")
        except: pass
        event.accept()

def ensure_roscore():
    try: rospy.get_master().getSystemState(); return
    except: pass
    subprocess.Popen(["roscore"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    for _ in range(50):
        try: rospy.get_master().getSystemState(); return
        except: time.sleep(0.2)

def main():
    try:
        ensure_roscore()
        rospy.init_node("amr_ui", anonymous=True, disable_signals=True)
        app = QApplication(sys.argv)
        model, ingest = WorldModel(), None
        ingest = ROSIngest(model)
        window = AMRMainWindow(model, ingest)
        window.start_bringup()
        window.show()
        sys.exit(app.exec_())
    except Exception as e:
        print(f"Fatal: {e}")
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__": main()