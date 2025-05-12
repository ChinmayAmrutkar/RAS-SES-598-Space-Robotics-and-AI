# 🛰️Rocky Times Challenge - Search, Map, & Analyze

This project implements a fully autonomous drone system that performs aerial surveying using a **lawnmower pattern**, detects **ArUco markers** in real-time via an onboard camera, estimates their global position, and **performs a stable and precise landing** on the detected marker. Built using **ROS 2 Humble**, **PX4 SITL**, and **OpenCV**, the system is modular, robust, and simulation-ready for feature extension.
This ROS2 package implements an autonomous drone system for geological feature detection, mapping, and analysis using an RGBD camera and PX4 SITL simulation.

---

## 🧠 Key Features

- 🚁 **Autonomous takeoff, offboard control, and arming**
- 🧭 **Lawnmower (boustrophedon) path generation** for wide-area search
- 🎯 **Real-time ArUco detection** using OpenCV 4.11+
- 🌍 **Global position estimation** of the marker using camera pose + odometry
- 🌀 **Smooth approach + hovering over marker** for stable landing
- 🛬 **Controlled vertical descent** and soft landing on the detected marker
- 🧼 **ROS 2 logging** with terminal-level status and debug output

---

## 🎥 Demo

<p align="center">
  <img src="https://github.com/YOUR_USERNAME/YOUR_REPO/assets/demo-gif.gif" width="600"/>
</p>

---

## 🗂️ Project Structure

```
terrain_mapping_drone_control/
├── scripts/
│   └── deploy_px4_model.sh     # PX4 model install script
├── launch/
│   └── cylinder_landing.launch.py  # Launch file for mission
├── terrain_mapping_drone_control/
│   └── aruco_lawnmower_landing.py  # Main ROS 2 mission node
├── worlds/ (optional)
│   └── custom cylinder + marker Gazebo world
```

---

## ⚙️ System Overview

### 🔁 State Machine Flow

```text
WAITING_FOR_ARM
   └── Takeoff (fixed height)
         └── Lawn Mower Survey
               └── Detect ArUco Marker
                     └── Hover & Stabilize
                           └── Descend & Land
```

### 🔎 ArUco Marker Detection

- Detected via downward-facing camera (`/drone/down_mono`)
- Uses `cv2.aruco` with `DICT_4X4_50` markers
- Position is estimated with `solvePnP()` using known marker size
- Once detected, the drone locks onto the estimated global pose
- Image subscription is **unregistered** after detection to reduce noise

---

## 🚀 Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- PX4-Autopilot (main or `9ac03f03eb`)
- OpenCV ≥ 4.11.0
- Python 3.10
- `cv_bridge`, `px4_msgs`, `sensor_msgs`, `trajectory_msgs`

### Clone and Symlink the Package

```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/terrain_mapping_drone_control.git
ln -s ~/terrain_mapping_drone_control/assignments/terrain_mapping_drone_control .
```

### Build and Source

```bash
cd ~/ros2_ws
colcon build --packages-select terrain_mapping_drone_control --symlink-install
source install/setup.bash
```

### Deploy PX4 Model (Optional)

```bash
cd ~/ros2_ws/src/terrain_mapping_drone_control
chmod +x scripts/deploy_px4_model.sh
./scripts/deploy_px4_model.sh -p ~/PX4-Autopilot
```

### Launch Simulation (PX4 + Gazebo)

```bash
make px4_sitl_default gz_x500_depth
```

### Run the ROS 2 Mission Node

```bash
ros2 run terrain_mapping_drone_control aruco_lawnmower_landing.py
```

---

## 🧪 Example Terminal Output

```text
--- Mission node initialized
--- Camera calibration received
--- Sent OFFBOARD command
--- Sent ARM command
--- Takeoff complete. Starting survey
--- ArUco detected. Setting landing target to: (3.82, -2.17)
--- Stopped ArUco detection. Using fixed landing coordinates.
--- Position reached. Starting landing...
--- Landed
```

---

## 🛠️ Tuning Parameters

| Parameter          | Description                            | Default |
|-------------------|----------------------------------------|---------|
| `TAKEOFF_HEIGHT`   | Height to reach after arming           | 11.70m  |
| `LAND_HEIGHT`      | Final height before land command       | 0.20m   |
| `MARKER_SIZE`      | Size of the ArUco marker (in meters)   | 0.80m   |
| `grid_step`        | Lawnmower line spacing                 | 4.0m    |
| `aruco_stable_counter` | Stability check before descent     | ≥10 frames |

---

## 📡 ROS Topics Used

| Topic                            | Type                      | Direction  |
|----------------------------------|---------------------------|------------|
| `/fmu/in/trajectory_setpoint`   | `px4_msgs/TrajectorySetpoint` | 📨 Publish |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | 📨 Publish |
| `/fmu/in/vehicle_command`       | `px4_msgs/VehicleCommand` | 📨 Publish |
| `/fmu/out/vehicle_odometry`     | `px4_msgs/VehicleOdometry` | 📤 Subscribe |
| `/drone/down_mono/image_raw`    | `sensor_msgs/Image`       | 📤 Subscribe |
| `/drone/down_mono/camera_info`  | `sensor_msgs/CameraInfo`  | 📤 Subscribe |

---

## 💡 Notes

- Coordinates of ArUco markers are calculated in camera frame → transformed to global (world) frame using odometry
- Hover logic ensures drone does not descend until marker position is stabilized
- System avoids immediate reaction to noisy detections by requiring multiple stable observations

---

## 📌 Future Improvements (Working on it)

- Use RTAB-Map for full scene 3D reconstruction and mesh export
- Replace position control with PID-tuned velocity control for smoother approach
- Add multiple marker detection support and select tallest rock
- Enable real-time energy + time tracking per run

---

## 📝 License

This project is licensed under the **MIT License**.  
Feel free to use and modify with attribution.

---

## 👨‍💻 Author

**Chinmay Yo**  
Graduate Researcher | Robotics & AI | Arizona State University  


