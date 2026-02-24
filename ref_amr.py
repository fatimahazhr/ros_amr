#!/usr/bin/env python3

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



from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton,
    QVBoxLayout, QHBoxLayout, QInputDialog
)
from PyQt5.QtCore import QTimer, QRectF, Qt

import pyqtgraph as pg

MAP_DIR = "/home/delivery/catkin_ws/src/stark/maps"

# =========================================================
# TELEOP KEY MAP
# =========================================================

MOVE_BINDINGS = {
    Qt.Key_I:(1,0,0,0), Qt.Key_O:(1,0,0,-1), Qt.Key_J:(0,0,0,1),
    Qt.Key_L:(0,0,0,-1), Qt.Key_U:(1,0,0,1), Qt.Key_Comma:(-1,0,0,0),
    Qt.Key_Period:(-1,0,0,1), Qt.Key_M:(-1,0,0,-1)
}

SPEED_BINDINGS = {
    Qt.Key_Q:(1.1,1.1), Qt.Key_Z:(0.9,0.9),
    Qt.Key_W:(1.1,1.0), Qt.Key_X:(0.9,1.0),
    Qt.Key_E:(1.0,1.1), Qt.Key_C:(1.0,0.9)
}

# =========================================================
# WORLD MODEL
# =========================================================

class WorldModel:
    def __init__(self):
        self.lock = threading.Lock()
        self.map_img = None
        self.map_extent = None
        self.scan_pts = None
        self.footprint_pts = None
        self.robot_pose = None  # (x, y, yaw)

    def snapshot(self):
        with self.lock:
            return (
                self.map_img,
                self.map_extent,
                self.scan_pts,
                self.footprint_pts,
                self.robot_pose
            )

# =========================================================
# ROS INGEST (CORRECT TF USAGE)
# =========================================================

class ROSIngest:
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
        rospy.Subscriber(
            "/move_base/global_costmap/footprint",
            PolygonStamped,
            self.footprint_cb,
            queue_size=1
        )
        

    def map_cb(self, msg):
        h, w = msg.info.height, msg.info.width
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y

        data = np.array(msg.data).reshape(h, w)
        img = np.zeros_like(data, dtype=np.uint8)
        img[data == -1] = 100
        img[data == 0]  = 255
        img[data > 0]   = 0

        with self.model.lock:
            self.model.map_img = img
            self.model.map_extent = (ox, ox + w*res, oy, oy + h*res)

    def update_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link",
                rospy.Time(0), rospy.Duration(0.05)
            )
        except Exception:
            return  # DO NOT fall back silently
    
        q = tf.transform.rotation
        yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    
        with self.model.lock:
            self.model.robot_pose = (
                tf.transform.translation.x,
                tf.transform.translation.y,
                yaw
            )
    
    

    def scan_cb(self, msg):
        

        try:
            target = "map" if self.model.map_img is not None else "odom"
            
            tf = self.tf_buffer.lookup_transform(
                target, msg.header.frame_id,
                rospy.Time(0), rospy.Duration(0.05)
            )
            
            
        except Exception:
            return

        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges))*msg.angle_increment
        valid = np.isfinite(ranges)

        xs = ranges[valid]*np.cos(angles[valid])
        ys = ranges[valid]*np.sin(angles[valid])

        T = tft.quaternion_matrix([
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w
        ])
        T[0,3] = tf.transform.translation.x
        T[1,3] = tf.transform.translation.y

        pts = np.vstack([xs,ys,np.zeros_like(xs),np.ones_like(xs)])

        with self.model.lock:
            self.model.scan_pts = (T @ pts)[:2].T

    def footprint_cb(self, msg):
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
        T[0,3] = tf.transform.translation.x
        T[1,3] = tf.transform.translation.y

        pts = np.array([[p.x,p.y,0,1] for p in msg.polygon.points]).T

        with self.model.lock:
            self.model.footprint_pts = (T @ pts)[:2].T

# =========================================================
# UI + TELEOP
# =========================================================
class TeleopKeyboardUI(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        grid = QVBoxLayout(self)

        rows = [
            ["u", "i", "o"],
            ["j", "k", "l"],
            ["m", ",", "."],
        ]

        for row in rows:
            h = QHBoxLayout()
            for key in row:
                b = QPushButton(key.upper())
                b.setFixedSize(50, 50)
                b.pressed.connect(lambda k=key: parent.virtual_key_press(k))
                b.released.connect(parent.virtual_key_release)
                h.addWidget(b)
            grid.addLayout(h)
            
class LocalizeViewBox(pg.ViewBox):
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




class MapUI(QWidget):
    def __init__(self, model, ingest):
        super().__init__()

        self.ingest = ingest
        self.model = model
        self.setWindowTitle("AMR Control Panel")
        self.setFocusPolicy(Qt.StrongFocus)
        self.localization_mode = False
        self.loc_start = None     # (x, y)
        self.loc_arrow = None     # visual arrow
        self.bringup_proc = None
        
        

        # =========================
        # ROS INTERFACES
        # =========================
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.initpose_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped,
            queue_size=1, latch=True
        )
        

        # =========================
        # TELEOP STATE
        # =========================
        self.speed = 0.5
        self.turn = 1.0
        self.x = self.y = self.z = self.th = 0.0
        self.manual_mode = False

        # =========================
        # AUTONOMOUS STATE
        # =========================
        self.waypoint_mode = False
        self.waypoints = []              # [(x, y)]
        self.wp_text_items = []
        self.active_wp_index = -1
        self.autonomous_running = False
        self.autonomous_loop = False
        self.mapping_active = False
        

        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal",
            PoseStamped,
            queue_size=1
        )
        
        self.cancel_pub = rospy.Publisher(
            "/move_base/cancel",
            GoalID,
            queue_size=1
        )
        

        rospy.Subscriber(
            "/move_base/result",
            MoveBaseActionResult,
            self._on_nav_result,
            queue_size=1
        )
        
        rospy.Subscriber(
            "/move_base/GlobalPlanner/plan",
            Path,
            self._on_global_path,
            queue_size=1
        )
        

        # =========================
        # MAIN LAYOUT
        # =========================
        main = QHBoxLayout(self)

        # =========================
        # MAP WIDGET (CREATE FIRST)
        # =========================
        self.viewbox = LocalizeViewBox(self)
        self.plot = pg.PlotWidget(viewBox=self.viewbox)
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True)
        self.plot.getViewBox().invertY(True)
        self.plot.scene().sigMouseClicked.connect(self._on_map_click)
        # ---- Localization drag line ----
        self.loc_start = None
        self.loc_line = pg.PlotDataItem(
            pen=pg.mkPen(color=(200, 0, 200), width=2)  # purple line
        )
        self.plot.addItem(self.loc_line)
        

        # ---- Map Image ----
        self.map_item = pg.ImageItem()
        self.map_item.setOpts(axisOrder="row-major")
        self.plot.addItem(self.map_item)

        # ---- Laser Scan ----
        self.scan_item = pg.ScatterPlotItem(
            size=3,
            brush=pg.mkBrush(255, 0, 0)
        )
        self.plot.addItem(self.scan_item)

        # ---- Footprint ----
        self.fp_item = pg.PlotDataItem(
            pen=pg.mkPen("g", width=2)
        )
        self.plot.addItem(self.fp_item)

        # ---- Robot Pose ----
        self.robot_item = pg.ArrowItem(
            headLen=0.3,
            tailLen=0.2,
            brush="y"
        )
        self.plot.addItem(self.robot_item)

        main.addWidget(self.plot, 4)

        # =========================
        # MODE LABEL (AFTER plot)
        # =========================
        self.mode_label = pg.TextItem(
            "MODE: IDLE",
            color="y",
            anchor=(0, 1)
        )
        self.mode_label.setPos(0, 0)
        self.plot.addItem(self.mode_label)

        # =========================
        # WAYPOINT VISUALS
        # =========================
        self.wp_scatter = pg.ScatterPlotItem(
            size=12,
            brush=pg.mkBrush(0, 0, 255),
            pen=pg.mkPen("w")
        )
        self.plot.addItem(self.wp_scatter)

        # =========================
        # CONTROLS PANEL
        # =========================
        controls = QVBoxLayout()

        for label, fn in [
            ("Save Map", self.save_map),
            ("Load Map", self.load_map),
            ("Mapping Mode", self.start_mapping),
            ("Manual Mode (Keyboard)", self.toggle_manual),
            ("Add Waypoints (Click)", self.enable_wp_mode),
            ("Save Waypoints", self.save_waypoints),
            ("Load Waypoints", self.load_waypoints),
            ("Start Autonomous", self.start_autonomous),
            ("Toggle Loop", self.toggle_loop),
            ("Clear Waypoints", self.clear_waypoints),
            ("Stop Autonomous", self.stop_autonomous),
            ("Localization Mode", self.toggle_localization),
            
        ]:
        
            b = QPushButton(label)
            b.clicked.connect(fn)
            controls.addWidget(b)

        # ---- Virtual Keyboard ----
        self.teleop_keyboard = TeleopKeyboardUI(self)
        self.teleop_keyboard.hide()
        controls.addWidget(self.teleop_keyboard)

        controls.addStretch()
        main.addLayout(controls, 1)

        # =========================
        # TIMERS
        # =========================
        self.timer = QTimer()
        self.timer.timeout.connect(self.draw)
        self.timer.start(30)

        self.teleop_timer = QTimer()
        self.teleop_timer.timeout.connect(self.publish_cmd)
        self.teleop_timer.start(50)
        self.ros_processes = []
        


        # ---- PATH VISUALIZATION ----
        self.global_path = None
        
        self.path_item = pg.PlotDataItem(
            pen=pg.mkPen(color=(0, 0, 255), width=2)  # BLUE path
        )
        self.plot.addItem(self.path_item)
        
        
    def closeEvent(self, event):
        self.shutdown_ros_processes()
        event.accept()
    
        
    def shutdown_ros_processes(self):
        for p in self.ros_processes:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                pass
        self.ros_processes.clear()
    
        # Kill known nodes defensively (safe even if not running)
        subprocess.call("rosnode kill -a", shell=True)
    
        rospy.signal_shutdown("UI closed")
    
    def update_mode_label(self):
        modes = []
    
        if self.mapping_active:
            modes.append("MAPPING")
        elif not self.mapping_active:
            modes.append("NAVIGATION")
    
        if self.manual_mode:
            modes.append("MANUAL")
    
        if self.waypoint_mode:
            modes = ["WAYPOINT INPUT"]
    
        text = " + ".join(modes) if modes else "IDLE"
        self.mode_label.setText(f"MODE: {text}", color="y")
        
    def start_bringup(self):
        if self.bringup_proc is not None:
            return
    
        self.mode_label.setText("MODE: STARTING BRINGUP", color="y")
    
        self.bringup_proc = subprocess.Popen(
            ["roslaunch", "stark", "bringup.launch"],
            preexec_fn=os.setsid
        )
        self.ros_processes.append(self.bringup_proc)
        
        
    
    
    
        # Wait for critical topics
        timeout = rospy.Time.now() + rospy.Duration(10.0)
        while not rospy.is_shutdown():
            topics = rospy.get_published_topics()
            names = [t[0] for t in topics]
    
            if "/odom" in names and "/scan" in names:
                break
    
            if rospy.Time.now() > timeout:
                rospy.logwarn("Bringup timeout — odom/scan not ready")
                break
    
            rospy.sleep(0.2)
    
        self.mode_label.setText("MODE: READY", color="g")
    
    
    
    def enable_wp_mode(self):
        self.localization_mode = False
        self.plot.getViewBox().setMouseEnabled(x=True, y=True)
        self.waypoint_mode = not self.waypoint_mode
        self.manual_mode = False
        self.teleop_keyboard.hide()
    
        if self.waypoint_mode:
            self.mode_label.setText("MODE: WAYPOINT INPUT", color="c")
        else:
            self.mode_label.setText("MODE: IDLE", color="y")
    
    def _on_global_path(self, msg):
        if not msg.poses:
            self.global_path = None
            return
    
        xs = [p.pose.position.x for p in msg.poses]
        ys = [p.pose.position.y for p in msg.poses]
    
        self.global_path = (xs, ys)
    
    def toggle_localization(self):
        self.localization_mode = not self.localization_mode
    
        # Disable other interaction modes
        self.manual_mode = False
        self.waypoint_mode = False
        self.teleop_keyboard.hide()
    
        vb = self.plot.getViewBox()
    
        if self.localization_mode:
            self.mode_label.setText("MODE: LOCALIZATION", color="m")
    
            # 🔒 lock pan/zoom so ViewBox gets mouse control
            vb.setMouseEnabled(x=False, y=False)
    
            # cleanup any stale arrow
            if self.loc_arrow:
                self.plot.removeItem(self.loc_arrow)
                self.loc_arrow = None
            self.loc_start = None
    
        else:
            self.mode_label.setText("MODE: IDLE", color="y")
    
            # 🔓 restore normal interaction
            vb.setMouseEnabled(x=True, y=True)
    
            if self.loc_arrow:
                self.plot.removeItem(self.loc_arrow)
                self.loc_arrow = None
            self.loc_start = None
    
    
    
    
    
    def _loc_start_press(self, p):
        if not self.localization_mode:
            return
    
        self.loc_start = (p.x(), p.y())
    
        # Initialize zero-length line
        self.loc_line.setData(
            [self.loc_start[0], self.loc_start[0]],
            [self.loc_start[1], self.loc_start[1]]
        )
    
    
    def _loc_drag(self, p):
        if not self.localization_mode or self.loc_start is None:
            return
    
        self.loc_line.setData(
            [self.loc_start[0], p.x()],
            [self.loc_start[1], p.y()]
        )
    
    
    def _loc_release(self, p):
        if not self.localization_mode or self.loc_start is None:
            return
    
        dx = p.x() - self.loc_start[0]
        dy = p.y() - self.loc_start[1]
    
        # Ignore accidental clicks
        if abs(dx) < 0.02 and abs(dy) < 0.02:
            self.loc_line.clear()
            self.loc_start = None
            return
    
        yaw = np.arctan2(dy, dx)
    
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
    
        msg.pose.pose.position.x = self.loc_start[0]
        msg.pose.pose.position.y = self.loc_start[1]
    
        q = tft.quaternion_from_euler(0, 0, yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
    
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.1
    
        self.initpose_pub.publish(msg)
    
        # Cleanup
        self.loc_line.clear()
        self.loc_start = None
    
    
    def _loc_mouse_move(self, evt):
        if not self.localization_mode or self.loc_start is None or self.loc_arrow is None:
            return
    
        vb = self.plot.getViewBox()
        if not vb.sceneBoundingRect().contains(evt.scenePos()):
            return
    
        p = vb.mapSceneToView(evt.scenePos())
        dx = p.x() - self.loc_start[0]
        dy = p.y() - self.loc_start[1]
    
        yaw = np.degrees(np.arctan2(dy, dx))
        self.loc_arrow.setStyle(angle=yaw)
    
    
    
    def _on_map_click(self, evt):
        if not self.waypoint_mode:
            return
    
        vb = self.plot.getViewBox()
        pos = evt.scenePos()
        if not vb.sceneBoundingRect().contains(pos):
            return
    
        p = vb.mapSceneToView(pos)
        x, y = round(p.x(), 3), round(p.y(), 3)
    
        self.waypoints.append([float(x), float(y)])
        
    
        # Update waypoint points
        xs = [w[0] for w in self.waypoints]
        ys = [w[1] for w in self.waypoints]
        self.wp_scatter.setData(xs, ys)
    
        idx = len(self.waypoints)
    
        # ---- INDEX LABEL (above point) ----
        idx_label = pg.TextItem(
            str(idx),
            color="k",
            anchor=(0.5, 1.2)
        )
        idx_label.setPos(x, y)
        self.plot.addItem(idx_label)
    
        # ---- COORD LABEL (below point) ----
        coord_label = pg.TextItem(
            f"({x}, {y})",
            color="k",
            anchor=(0.5, -0.2)
        )
        coord_label.setPos(x, y)
        self.plot.addItem(coord_label)
    
        self.wp_text_items.append(idx_label)
        self.wp_text_items.append(coord_label)
    
    
    
    def save_waypoints(self):
        name, ok = QInputDialog.getText(self, "Save Waypoints", "File name:")
        if not ok:
            return
        path = f"{MAP_DIR}/{name}_wps.yaml"
        with open(path, "w") as f:
            yaml.dump({"waypoints": self.waypoints}, f)
    
    
    
    def load_waypoints(self):
        name, ok = QInputDialog.getText(self, "Load Waypoints", "File name:")
        if not ok:
            return
    
        path = f"{MAP_DIR}/{name}_wps.yaml"
        with open(path) as f:
            data = yaml.safe_load(f)
    
        for t in self.wp_text_items:
            self.plot.removeItem(t)
    
        self.wp_text_items.clear()
        self.waypoints = data["waypoints"]
    
        xs, ys = [], []
        for i, (x, y) in enumerate(self.waypoints, 1):
            xs.append(x)
            ys.append(y)
            idx_label = pg.TextItem(str(i), color="k", anchor=(0.5, 1.2))
            idx_label.setPos(x, y)
            self.plot.addItem(idx_label)
            
            coord_label = pg.TextItem(f"({x}, {y})", color="k", anchor=(0.5, -0.2))
            coord_label.setPos(x, y)
            self.plot.addItem(coord_label)
            
            self.wp_text_items.append(idx_label)
            self.wp_text_items.append(coord_label)
            
    
        self.wp_scatter.setData(xs, ys)
    
    
    def start_autonomous(self):
        if not self.waypoints:
            return
        self.autonomous_running = True
        self.active_wp_index = 0
        self._send_goal(self.waypoints[0])
    
    def toggle_loop(self):
        self.autonomous_loop = not self.autonomous_loop
    
    def _send_goal(self, wp):
        x, y = wp
    
        brushes = []
        for i in range(len(self.waypoints)):
            brushes.append(
                pg.mkBrush(255, 255, 0) if i == self.active_wp_index
                else pg.mkBrush(0, 0, 255)
            )
        self.wp_scatter.setBrush(brushes)
    
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = rospy.Time.now()
        g.pose.position.x = x
        g.pose.position.y = y
        g.pose.orientation.w = 1.0
    
        self.goal_pub.publish(g)
        
    def clear_waypoints(self):
        self.waypoints.clear()
        self.wp_scatter.clear()
        self.active_wp_index = -1
        self.autonomous_running = False
    
        for t in self.wp_text_items:
            self.plot.removeItem(t)
        self.wp_text_items.clear()
    
        self.mode_label.setText("MODE: IDLE", color="y")
    
    def stop_autonomous(self):
        self.autonomous_running = False
        self.active_wp_index = -1
    
        self.cancel_pub.publish(GoalID())  # HARD STOP move_base
    
        if self.waypoints:
            brushes = [pg.mkBrush(0, 0, 255)] * len(self.waypoints)
            self.wp_scatter.setBrush(brushes)
    
        self.global_path = None
        self.path_item.clear()
        self.mode_label.setText("MODE: IDLE", color="y")
        
    
    
    
    
    def _on_nav_result(self, msg):
        if not self.autonomous_running:
            return
    
        if msg.status.status != 3:
            return  # ignore failures for now
    
        self.active_wp_index += 1
    
        if self.active_wp_index >= len(self.waypoints):
            if self.autonomous_loop:
                self.active_wp_index = 0
                self._send_goal(self.waypoints[0])
            else:
                # ---- FINAL DESTINATION REACHED ----
                self.autonomous_running = False
                self.active_wp_index = -1
        
                # reset waypoint colors
                if self.waypoints:
                    self.wp_scatter.setBrush(
                        [pg.mkBrush(0, 0, 255)] * len(self.waypoints)
                    )
        
                # clear global path
                self.global_path = None
                self.path_item.clear()
        
                # update UI state
                self.mode_label.setText("MODE: IDLE", color="y")
        
                return
        
    
        self._send_goal(self.waypoints[self.active_wp_index])
    
    # ---- TELEOP ----
    def toggle_manual(self):
        self.localization_mode = False
        self.plot.getViewBox().setMouseEnabled(x=True, y=True)
        
        self.manual_mode = not self.manual_mode
        self.waypoint_mode = False
        self.x = self.y = self.z = self.th = 0.0
    
        if self.manual_mode:
            self.teleop_keyboard.show()
        else:
            self.teleop_keyboard.hide()
        
        self.update_mode_label()
        
            
    

    def keyPressEvent(self, e):
        if not self.manual_mode: return
        k = e.key()
        if k in MOVE_BINDINGS:
            self.x,self.y,self.z,self.th = MOVE_BINDINGS[k]
        elif k in SPEED_BINDINGS:
            self.speed *= SPEED_BINDINGS[k][0]
            self.turn  *= SPEED_BINDINGS[k][1]

    def keyReleaseEvent(self, e):
        if self.manual_mode:
            self.x = self.y = self.z = self.th = 0.0

    def publish_cmd(self):
        if not self.manual_mode: return
        t = Twist()
        t.linear.x = self.x*self.speed
        t.angular.z = self.th*self.turn
        self.cmd_pub.publish(t)
    
    def virtual_key_press(self, key):
        if not self.manual_mode:
            return
    
        key_map = {
            "i": Qt.Key_I, "o": Qt.Key_O, "j": Qt.Key_J,
            "l": Qt.Key_L, "u": Qt.Key_U, "m": Qt.Key_M,
            ",": Qt.Key_Comma, ".": Qt.Key_Period
        }
    
        k = key_map.get(key)
        if k in MOVE_BINDINGS:
            self.x, self.y, self.z, self.th = MOVE_BINDINGS[k]
    
    def virtual_key_release(self):
        if self.manual_mode:
            self.x = self.y = self.z = self.th = 0.0
    

    # ---- DRAW ----
    def draw(self):
        self.ingest.update_robot_pose()
        m, ext, scan, fp, pose = self.model.snapshot()

        if m is not None:
            self.map_item.setImage(m, autoLevels=False)
            self.map_item.setRect(QRectF(
                ext[0], ext[2],
                ext[1]-ext[0], ext[3]-ext[2]
            ))

        if scan is not None:
            self.scan_item.setData(scan[:,0], scan[:,1])

        if fp is not None:
            f = np.vstack([fp, fp[0]])
            self.fp_item.setData(f[:,0], f[:,1])

        if pose is not None:
            x,y,yaw = pose
            self.robot_item.setPos(x,y)
            self.robot_item.setStyle(angle=-np.degrees(yaw))
            
        # ---- GLOBAL PATH ----
        if self.global_path is not None:
            xs, ys = self.global_path
            self.path_item.setData(xs, ys)
        else:
            self.path_item.clear()
        

    # ---- MODES ----
    def save_map(self):
        name, ok = QInputDialog.getText(self,"Save Map","Map name:")
        if ok:
            save_proc = subprocess.Popen(
                ["rosrun", "map_server", "map_saver", "-f", name],
                cwd=MAP_DIR,
                preexec_fn=os.setsid
            )
            self.ros_processes.append(save_proc)
            

    def load_map(self):
        name, ok = QInputDialog.getText(self, "Load Map", "Map name:")
        if not ok or not name:
            return
    
        subprocess.call("rosnode kill /slam_gmapping", shell=True)
        subprocess.call("rosnode kill /map_server", shell=True)
        subprocess.call("rosnode kill /amcl", shell=True)
        subprocess.call("rosnode kill /move_base", shell=True)
        subprocess.call("rosnode kill /relay_scan", shell=True)
    
        nav_proc = subprocess.Popen(
            ["roslaunch", "stark", "navigate.launch", f"map_file:={name}"],
            preexec_fn=os.setsid
        )
        self.ros_processes.append(nav_proc)
        
    
        rospy.sleep(2.0)  # 🔑 allow AMCL + TF to come up
    
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.initpose_pub.publish(msg)
    
        self.mode_label.setText("MODE: NAVIGATION", color="g")
        
        self.mapping_active = False
        self.update_mode_label()
        
    
    
    
    
    def start_mapping(self):
        # Stop NAV stack ONLY
        subprocess.call("rosnode kill /map_server", shell=True)
        subprocess.call("rosnode kill /amcl", shell=True)
        subprocess.call("rosnode kill /move_base", shell=True)
        subprocess.call("rosnode kill /relay_scan", shell=True)
    
        # Clear UI state
        self.model.map_img = None
        self.model.map_extent = None
        self.model.robot_pose = None
    
        # Reset TF buffer so old transforms don’t linger
        self.ingest.tf_buffer.clear()
    
        # Start SLAM
        slam_proc = subprocess.Popen(
            ["roslaunch", "stark", "lidar_slam.launch"],
            preexec_fn=os.setsid
        )
        self.ros_processes.append(slam_proc)
        
    
        # 🔑 CRITICAL: anchor map to current odom pose
        rospy.sleep(1.5)
        self.reset_map_origin_to_current_pose()
    
        self.autonomous_running = False
        self.active_wp_index = -1
        self.mapping_active = True
        self.update_mode_label()
    
    
    def reset_map_origin_to_current_pose(self):
        try:
            tf = self.ingest.tf_buffer.lookup_transform(
                "odom", "base_link",
                rospy.Time(0),
                rospy.Duration(0.5)
            )
        except Exception:
            rospy.logwarn("Cannot reset map origin: TF unavailable")
            return
    
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
    
        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.position.z = 0.0
    
        q = tf.transform.rotation
        msg.pose.pose.orientation = q
    
        # tight covariance = trust this pose
        msg.pose.covariance[0]  = 0.01
        msg.pose.covariance[7]  = 0.01
        msg.pose.covariance[35] = 0.02
    
        self.initpose_pub.publish(msg)
    
    
        
    
    
def ensure_roscore():
        try:
            rospy.get_master().getSystemState()
            return  # roscore already running
        except:
            pass
    
        roscore_proc = subprocess.Popen(
            ["roscore"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        
    
        # Wait for roscore
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
    ensure_roscore()   # 🔑 MUST be first

    rospy.init_node("amr_ui", anonymous=True, disable_signals=True)

    app = QApplication(sys.argv)

    model = WorldModel()
    ingest = ROSIngest(model)
    ui = MapUI(model, ingest)

    ui.start_bringup()  # bringup after roscore

    ui.resize(1400, 800)
    ui.show()

    sys.exit(app.exec_())



if __name__ == "__main__":
    main()
