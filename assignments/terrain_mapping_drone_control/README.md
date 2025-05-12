# 🛰️ Rocky Times Challenge - Search, Map, & Analyze

This project implements a fully autonomous drone system to search, detect, and analyze geological formations using **PX4 SITL**, **ROS 2 Humble**, and an **RGBD camera**. The system performs a **lawnmower survey**, detects **ArUco markers** in real time, estimates their global pose, and executes a **controlled precision landing** on the tallest detected cylinder (10m).

---

## 🚀 Key Capabilities

- ✅ **Autonomous takeoff, offboard control, arming**
- 📐 **Lawnmower (boustrophedon) survey pattern** for area coverage
- 🎯 **Real-time ArUco marker detection** using OpenCV 4.11+
- 🌍 **Global pose estimation** via solvePnP + PX4 odometry fusion
- 🛬 **Stabilized hover and vertical descent** for precise landings
- 📊 **ROS 2-based logging, diagnostics, and telemetry**
- 📦 **Modular mission node** supporting extensibility for SLAM/RTAB-Map

---

## 🎥 Demo Highlights

| Trial 1 | Trial 2 | Trial 3 |
|--------|---------|---------|
| ![TRIALGIF2](https://github.com/user-attachments/assets/1c9cb637-9e3a-43b8-bf91-6a311dd4bee0) | ![TRIALGIF3](https://github.com/user-attachments/assets/9e7fe304-f3e8-4785-a6c3-df1c969eea9c) | ![trialgif4](https://github.com/user-attachments/assets/01897582-220c-4057-8f86-02725953c99d) |

---

## 🧭 Mission Workflow

1. Drone autonomously takes off to a fixed height (e.g., 12m).
2. Executes a lawnmower coverage over the search area.
3. Continuously scans for ArUco markers using a downward-facing camera.
4. Upon detection:
   - Aborts survey
   - Estimates marker’s world-frame pose using `solvePnP` + odometry
   - Approaches marker and stabilizes hover
   - Descends slowly and lands on the marker

---

## 🧠 State Machine

```text
TAKEOFF → SURVEY → GOTO_ARUCO → HOVER → DESCEND → LAND
```

---

## 🗂️ Project Structure

```
terrain_mapping_drone_control/
├── launch/
│   └── cylinder_landing.launch.py
├── scripts/
│   └── deploy_px4_model.sh
├── terrain_mapping_drone_control/
│   └── aruco_lawnmower_landing.py
│   └── aruco_tracker.py
│   └── aruco_landing_controller.py
```

---

## 🧪 Results Summary

| Trial   | Survey Complete | ArUco Detected | Hover Stable | Controlled Descent | Landed on Marker |
|---------|------------------|----------------|---------------|---------------------|-------------------|
| Trial 1 | ✅               | ✅              | ✅             | ✅                   | ❌ Slight Offset   |
| Trial 2 | ✅               | ✅              | ✅             | ✅                   | ❌ Slight Offset   |
| Trial 3 | ✅               | ✅              | ✅             | ✅                   | ❌ Slight Offset   |

📌 **Observation:** All trials executed the complete mission pipeline successfully. Minor landing offset was observed — likely due to slight camera-to-body transform error or noisy solvePnP `tvec`.

---

## 🔧 System Requirements

- Ubuntu 22.04
- ROS 2 Humble
- PX4-Autopilot (main branch)
- OpenCV ≥ 4.11.0
- Python ≥ 3.8
- RTAB-Map (for optional 3D mapping)

---

## 🛠️ Installation & Setup

### 1. Clone & Link the Package

```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/terrain_mapping_drone_control.git
ln -s ~/terrain_mapping_drone_control/assignments/terrain_mapping_drone_control .
```

### 2. Build the Workspace

```bash
cd ~/ros2_ws
colcon build --packages-select terrain_mapping_drone_control --symlink-install
source install/setup.bash
```

### 3. Deploy PX4 Model Files

```bash
cd ~/ros2_ws/src/terrain_mapping_drone_control
chmod +x scripts/deploy_px4_model.sh
./scripts/deploy_px4_model.sh -p ~/PX4-Autopilot
```

### 4. Run PX4 SITL

```bash
cd ~/PX4-Autopilot
make px4_sitl_default gz_x500_depth
```

### 5. Launch ROS 2 Mission Node

```bash
ros2 run terrain_mapping_drone_control aruco_lawnmower_landing.py
```

---

## 🧪 Terminal Output Example

```text
--- Takeoff complete
--- Executing survey pattern...
--- Marker detected at image center
--- Estimating marker pose...
--- Navigating to (3.6, -2.4)
--- Hovering above marker
--- Descending...
--- Landing complete
```

---

## ⚙️ Parameters

| Parameter            | Description                                | Default |
|---------------------|--------------------------------------------|---------|
| `TAKEOFF_HEIGHT`     | Altitude to reach after arming              | 12.0 m  |
| `MARKER_SIZE`        | Physical marker size                        | 0.8 m   |
| `LAND_HEIGHT`        | Final descent threshold                    | 0.2 m   |
| `grid_step`          | Lawn mowing line spacing                    | 4.0 m   |
| `aruco_stable_counter` | Required frames before locking target   | 10      |

---

## 🛰 ROS 2 Topics Used

| Topic                             | Type                         | Role      |
|----------------------------------|------------------------------|-----------|
| `/fmu/in/trajectory_setpoint`    | `px4_msgs/TrajectorySetpoint` | ➡️ Command |
| `/fmu/in/offboard_control_mode`  | `px4_msgs/OffboardControlMode`| ➡️ Mode    |
| `/fmu/in/vehicle_command`        | `px4_msgs/VehicleCommand`     | ➡️ Arm/Mode |
| `/fmu/out/vehicle_odometry`      | `px4_msgs/VehicleOdometry`    | ⬅️ Feedback |
| `/drone/down_mono/image_raw`     | `sensor_msgs/Image`           | ⬅️ Camera  |
| `/drone/down_mono/camera_info`   | `sensor_msgs/CameraInfo`      | ⬅️ Camera Info |

---

## 🔍 Future Enhancements

- 📐 Use RTAB-Map for 3D mapping and mesh export
- 🧭 Add frame alignment calibration to improve final landing precision
- 🧮 Integrate time + energy performance metrics
- 📍 Support multi-marker detection + landing on tallest

---

## 📜 License

This project is released under the **MIT License**.  
You are free to use, share, and modify with proper attribution.

