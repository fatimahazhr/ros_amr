# 🤖 AMR Robot Navigation System — Zahra

A full-featured ROS-based desktop GUI for controlling and navigating an **Autonomous Mobile Robot (AMR)**. Built with Python, PyQt5, and pyqtgraph, the system integrates real-time map visualization, SLAM-based mapping, waypoint navigation, and manual teleoperation — all in one interface.

---

## 📸 Features

- 🗺️ **Real-time Map Visualization** — displays occupancy grid map with live LiDAR scan overlay
- 📍 **Waypoint Navigation** — click-to-place waypoints on the map, save/load to YAML, run autonomously
- 🕹️ **Manual Teleoperation** — WASD keyboard and on-screen button control with adjustable speed
- 🧭 **Localization Tool** — click-and-drag to set initial robot pose (`/initialpose`) directly on the map
- 🗺️ **SLAM Mapping** — launch GMapping SLAM from the UI to build new maps
- 💾 **Map Save/Load** — save maps with `map_saver` and load them for navigation
- 🔁 **Autonomous Loop Mode** — optionally loop waypoints continuously
- 🚨 **Emergency Stop** — one-click full stop, cancels all goals and halts motion
- 📡 **ROS Integration** — manages roscore, bringup, SLAM, and navigation launch files
- 🌊 **Robot Footprint Display** — live robot footprint rendered on the map
- 🔵 **Global Path Visualization** — planned path from `move_base` drawn in real-time

---

## 🧱 Architecture

```
main()
├── ensure_roscore()          # Auto-starts roscore if not running
├── WorldModel                # Thread-safe shared state (map, scan, pose, footprint)
├── ROSIngest                 # ROS subscribers + TF listener
│   ├── /map                  # OccupancyGrid
│   ├── /scan                 # LaserScan
│   ├── /move_base/.../footprint
│   └── TF: map → base_link
└── AMRMainWindow (PyQt5)
    ├── Left Panel: Control Buttons
    ├── Right Panel: Map View (pyqtgraph)
    └── Bottom: Manual Control (WASD)
```

---

## 🛠️ Requirements

### System
- Ubuntu 20.04 (recommended)
- ROS Noetic
- Python 3.8+

### Python Packages

```bash
pip install PyQt5 pyqtgraph numpy pyyaml
```

### ROS Packages

```bash
sudo apt install ros-noetic-move-base ros-noetic-gmapping ros-noetic-map-server ros-noetic-amcl ros-noetic-tf2-ros
```

### ROS Launch Files Required (package: `stark`)

| Launch File           | Purpose                        |
|-----------------------|--------------------------------|
| `bringup.launch`      | Start robot hardware drivers   |
| `lidar_slam.launch`   | Start GMapping SLAM            |
| `navigate.launch`     | Start navigation stack (AMCL + move_base) |

---

## 🚀 Usage

```bash
python3 amr_navigation.py
```

The app will automatically start `roscore` if it's not already running, then launch robot bringup.

---

## 🗺️ Map Workflow

### Create a New Map
1. Click **Create Map** → launches GMapping SLAM
2. Drive the robot manually (WASD) to explore the environment
3. Click **Save Map** → enter a map name to save `.pgm` + `.yaml`

### Load an Existing Map
1. Click **Load Map** → enter the map name
2. The navigation stack (AMCL + move_base) will launch automatically
3. Set the initial robot pose using the **Localization** tool (click + drag on map)

> Maps are stored in: `/home/delivery/catkin_ws/src/stark/maps/`

---

## 📍 Waypoint Navigation

1. Click **Create WP** to enter waypoint mode
2. **Click on the map** to place waypoints (numbered and labeled automatically)
3. Click **Save WP** → enter a name to save as `<name>_wps.yaml`
4. Click **Load WP** to load a previously saved waypoint file
5. Click **Start Auto** to begin sequential navigation
6. Click **Stop Auto** or **EMERGENCY** to halt at any time
7. Click **Clear WP** to remove all waypoints

> Active waypoint is highlighted in **yellow**; pending waypoints are shown in **blue**.

---

## 🕹️ Manual Control

Enable **Manual** radio button to activate teleoperation.

| Key | Action         |
|-----|----------------|
| `W` | Move Forward   |
| `S` | Move Backward  |
| `A` | Turn Left      |
| `D` | Turn Right     |

Default speed: **0.5 m/s** | Default turn rate: **1.0 rad/s**

---

## 🧭 Localization (Set Initial Pose)

When the map is loaded but the robot's position is unknown:

1. Make sure you are **not** in Waypoint or Manual mode
2. **Click and drag** on the map to set position and orientation
3. The arrow direction you drag sets the robot's heading
4. The `/initialpose` message is published to AMCL

---

## 📡 ROS Topics

### Subscribed

| Topic | Type | Purpose |
|-------|------|---------|
| `/map` | `OccupancyGrid` | Map data |
| `/scan` | `LaserScan` | LiDAR data |
| `/move_base/local_costmap/footprint` | `PolygonStamped` | Robot footprint |
| `/move_base/result` | `MoveBaseActionResult` | Navigation result |
| `/move_base/GlobalPlanner/plan` | `Path` | Global planned path |

### Published

| Topic | Type | Purpose |
|-------|------|---------|
| `/cmd_vel` | `Twist` | Velocity command |
| `/initialpose` | `PoseWithCovarianceStamped` | Initial localization pose |
| `/move_base_simple/goal` | `PoseStamped` | Navigation goal |
| `/move_base/cancel` | `GoalID` | Cancel current goal |

---

## 📁 Project Structure

```
amr_navigation.py       # Main application (single-file)
README.md
```

Waypoint files are saved as:
```
/home/delivery/catkin_ws/src/stark/maps/<name>_wps.yaml
```

---

## 🖥️ UI Overview

| Panel | Contents |
|-------|----------|
| Left | Create Map, Load/Save Map, Create/Save/Load WP, Start/Stop Auto, Clear WP |
| Right (top) | Real-time map with scan, footprint, robot pose, path, waypoints |
| Right (bottom) | Manual mode toggle + WASD buttons + Emergency Stop |
| Status Bar | Current mode and status message |

---

## ⚠️ Notes

- Make sure the `stark` ROS package and all launch files are properly configured before running.
- The map directory path `/home/delivery/catkin_ws/src/stark/maps` can be changed via the `MAP_DIR` constant at the top of the script.
- The app handles roscore startup automatically, but ROS must be sourced in the terminal before launching:
  ```bash
  source /opt/ros/noetic/setup.bash
  source ~/catkin_ws/devel/setup.bash
  ```

---

## 📄 License

This project is provided for internal robotics deployment. Modify and extend freely for your environment.
