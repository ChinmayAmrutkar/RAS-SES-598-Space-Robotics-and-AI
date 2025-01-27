# First-Order Boustrophedon Navigator

## Project Overview
This project implements and optimizes a boustrophedon (lawnmower) pattern navigator using ROS2 and Turtlesim. The goal was to achieve precise coverage patterns through PD controller tuning and parameter optimization.

## Background
Boustrophedon patterns (from Greek: "ox-turning", like an ox drawing a plow) are fundamental coverage survey trajectories useful in space exploration and Earth observation. These patterns are useful for:

- **Space Exploration**: Rovers could use boustrophedon patterns to systematically survey areas of interest, ensuring complete coverage when searching for geological samples or mapping terrain. However, due to energy constraints, informative paths are usually optimized, and this results in paths that are sparser than complete coverage sampling, and may still produce high-accuracy reconstructions. 
  
- **Earth Observation**: Aerial vehicles employ these patterns for:
  - Agricultural monitoring and precision farming
  - Search and rescue operations
  - Environmental mapping and monitoring
  - Geological or archaeological surveys
  
- **Ocean Exploration**: Autonomous underwater vehicles (AUVs) use boustrophedon patterns to:
  - Map the ocean floor
  - Search for shipwrecks or aircraft debris
  - Monitor marine ecosystems
  
The efficiency and accuracy of these surveys depend heavily on the robot's ability to follow the prescribed path with minimal deviation (cross-track error). This assignment simulates these real-world challenges in a 2D environment using a first-order dynamical system (the turtlesim robot).

## Objective
Tune a PD controller to make a first-order system execute the most precise boustrophedon pattern possible. The goal is to minimize the cross-track error while maintaining smooth motion.

## Learning Outcomes
- Understanding PD control parameters and their effects on first-order systems
- Practical experience with controller tuning
- Analysis of trajectory tracking performance
- ROS2 visualization and debugging

### System Requirements
- Ubuntu 22.04 + ROS2 Humble
- Ubuntu 23.04 + ROS2 Iron
- Ubuntu 23.10 + ROS2 Iron
- Ubuntu 24.04 + ROS2 Jazzy

### Required Packages
```bash
sudo apt install ros-$ROS_DISTRO-turtlesim
sudo apt install ros-$ROS_DISTRO-rqt*
```

### Python Dependencies
```bash
pip3 install numpy matplotlib
```

## The Challenge

### 1. Controller Tuning (60 points)
Use rqt_reconfigure to tune the following PD controller parameters in real-time:
```python
# Controller parameters to tune
self.Kp_linear = 1.0   # Proportional gain for linear velocity
self.Kd_linear = 0.1   # Derivative gain for linear velocity
self.Kp_angular = 1.0  # Proportional gain for angular velocity
self.Kd_angular = 0.1  # Derivative gain for angular velocity
```

Performance Metrics:
- Average cross-track error (25 points)
- Maximum cross-track error (15 points)
- Smoothness of motion (10 points)
- Cornering performance (10 points)

### 2. Pattern Parameters (20 points)
Optimize the boustrophedon pattern parameters:
```python
# Pattern parameters to tune
self.spacing = 1.0     # Spacing between lines
```
- Coverage efficiency (10 points)
- Pattern completeness (10 points)

### 3. Analysis and Documentation (20 points)
Provide a detailed analysis of your tuning process:
- Methodology used for tuning
- Performance plots and metrics
- Challenges encountered and solutions
- Comparison of different parameter sets

## Approach

To solve this assignment, the following approach was adopted:

1. **Literature Review:**
   - Explored research papers and existing implementations of lawnmower patterns.
   - [Baehnemann et al. (2019)](https://jenjenchung.github.io/anthropomorphic/Papers/Baehnemann2019revisiting.pdf)  revisited coverage path planning, emphasizing how boustrophedon       patterns efficiently provide complete area coverage with minimal overlap and optimized         turns.
     <br>![image](https://github.com/user-attachments/assets/dafe214b-a4b8-45f5-a84a-975e9601d427)
2. **Initial Testing and Observation:**
   - Ran the existing code to identify key performance bottlenecks.
3. **Incremental Tuning Strategy:**
   - Started with low gains and increased them gradually.
   - Focused on tuning one parameter at a time.
   - Balanced speed and accuracy.
4. **Performance Monitoring:**
   - Used `rqt_plot` to visualize trajectory and cross-track error.
   - Analyzed cornering performance and smoothness.

## Getting Started

### Repository Setup
1. Fork the course repository:
   - Visit: https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI
   - Click "Fork" in the top-right corner
   - Select your GitHub account as the destination

2. Clone your fork (outside of ros2_ws):
```bash
cd ~/
git clone https://github.com/YOUR_USERNAME/RAS-SES-598-Space-Robotics-and-AI.git
```

3. Create a symlink to the assignment in your ROS2 workspace:
```bash
cd ~/ros2_ws/src
ln -s ~/RAS-SES-598-Space-Robotics-and-AI/assignments/first_order_boustrophedon_navigator .
```

### Building and Running
1. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select first_order_boustrophedon_navigator
source install/setup.bash
```

2. Launch the demo:
```bash
ros2 launch first_order_boustrophedon_navigator boustrophedon.launch.py
```

3. Monitor performance:
```bash
# View cross-track error as a number
ros2 topic echo /cross_track_error

# Or view detailed statistics in the launch terminal
```

4. Visualize trajectory and performance:
```bash
ros2 run rqt_plot rqt_plot
```
Add these topics:
- /turtle1/pose/x
- /turtle1/pose/y
- /turtle1/cmd_vel/linear/x
- /turtle1/cmd_vel/angular/z
- /cross_track_error

### Initial Testing and Observation

#### System Behavior Analysis
Initial testing revealed several key characteristics of the system:
- The turtlebot showed sensitivity to angular velocity controls
- Cross-track error varied significantly with different gain combinations
- Pattern spacing directly impacted coverage completeness
- Turn behavior required careful tuning of angular gains

#### Key Observations
1. **Path Following Accuracy:**
   - Higher Kp_linear values improved straight-line tracking
   - Angular control required precise tuning to prevent oscillations
   - System showed good stability with proper gain selection

2. **System Response:**
   - Linear velocity maintained consistent speed during straight paths
   - Angular velocity showed expected peaks during turns
   - Cross-track error remained well within acceptable limits

## Parameter Testing and Analysis Results

### Performance Testing Results

| Test Case | Kp_linear | Kd_linear | Kp_angular | Kd_angular | Spacing | Avg Cross-Track Error | Max Cross-Track Error | Linear Velocity Profile | Angular Velocity Profile | Cornering Performance | Coverage Quality | Notes |
|-----------|-----------|-----------|------------|------------|---------|---------------------|---------------------|----------------------|----------------------|---------------------|------------------|-------|
| 1 | 1.0 | 0.1 | 1.0 | 0.1 | 1.0 | 0.984 | 2.209 | Sluggish | Inconsistent | Poor - Wide turns | Uneven coverage | Initial test - Too conservative |
| 2 | 7.0 | 0.1 | 3.0 | 0.2 | 1.0 | 0.556 | 0.995 | Improved | Some oscillations |  Poor - Wide turns | Uneven coverage | Better tracking but unstable |
| 3 | 10.0 | 0.1 | 5.0 | 0.2 | 1.0 | 0.281 | 0.609 | More responsive | Some oscillations | Good | Even spacing | Good Spacing |
| 4 | 10.0 | 0.1 | 7.0 | 0.1 | 1.0 | 0.157 | 0.323 | More responsive | Better stability | Good | Even spacing | Getting closer to target |
| 5 | 10.0 | 0.1 | 10.0 | 0.05 | 1.0 | 0.053 | 0.185 | Agressive and Oscilating | Some oscillations | Not good | Uneven spacing | Too agressive |
| 6 | 8.0 | 0.5 | 10.0 | 0.05 | 1.0 | 0.053 | 0.146 | Stable | Some oscillations | Better | Better evenly spacing | Can be fine tuned |
| 7 | 7.0 | 0.5 | 10.0 | 0.02 | 1.0 | 0.045 | 0.133 | Very good | Smooth | Good evenly spacing | Very good | Close to target |
| **10** | **7.0** | **0.6** | **10.0** | **0.01** | **0.4** | **0.055** | **0.133** | **Excellent** | **Perfect transitions** | **Optimal** | **Optimal spacing** | **Best Performance** ✅ |
| 11 | 8.0 | 0.2 | 7.0 | 0.1 | 1.0 | 0.098 | 0.256 | Too aggressive | Sharp transitions | Good | Slight overlap | Post-optimal test |
| 12 | 9.0 | 0.3 | 8.0 | 0.05 | 0.3 | 0.075 | 0.225 | Oscillatory | Overshooting | Moderate | Excessive overlap | Too aggressive |


## Implemented Solution

### Final Parameter Values
```python
# Optimized Controller Parameters
Kp_linear = 7.0     # Strong linear correction
Kd_linear = 0.6     # Balanced damping
Kp_angular = 10.0   # Quick angular response
Kd_angular = 0.01   # Minimal angular damping
spacing = 0.4       # Tight pattern spacing
```

### Performance Achievements
1. **Cross-Track Error:**
   - Average: 0.055 units (Target: < 0.2 units) ✅
   - Maximum: 0.133 units (Target: < 0.5 units) ✅

2. **Velocity Control:**
   - Linear velocity maintained around 0.632 units
   - Smooth transitions between path segments
   - Stable angular velocity during turns

3. **Pattern Quality:**
   - Complete coverage (100% completion)
   - Uniform line spacing at 0.4 units
   - Clean cornering behavior

### Parameter Justification

1. **Kp_linear = 7.0:**
   - Provides strong path-following behavior
   - Ensures quick correction of linear deviations
   - Maintains stable straight-line motion

2. **Kd_linear = 0.6:**
   - Adds appropriate damping to linear motion
   - Prevents overshooting during corrections
   - Smooths velocity transitions

3. **Kp_angular = 10.0:**
   - Enables sharp, precise turns
   - Maintains heading accuracy
   - Quick response to direction changes

4. **Kd_angular = 0.01:**
   - Minimal damping for responsive turning
   - Prevents oscillations while maintaining agility
   - Allows smooth corner transitions

5. **spacing = 0.4:**
   - Ensures complete coverage
   - Provides optimal path density
   - Balances efficiency with thoroughness

## Performance Analysis

### System Stability
- Consistent cross-track error around 0.055 units
- Smooth velocity profiles throughout execution
- Stable cornering behavior

### Coverage Efficiency
- Achieved 100% completion rate
- Uniform pattern spacing
- No gaps in coverage

### Motion Quality
- Linear velocity maintained steady state
- Angular velocity appropriately peaked during turns
- Minimal oscillations in straight-line segments

## Conclusions

The implemented solution successfully achieved:
- Precise path following with minimal cross-track error
- Complete area coverage with uniform spacing
- Stable and efficient motion throughout the pattern
- All target performance metrics met or exceeded

The final parameter configuration represents an optimal balance between:
- Control responsiveness and stability
- Coverage completeness and efficiency
- Motion smoothness and precision

## Extra Credit (10 points)
Create and implement a custom ROS2 message type to publish detailed performance metrics:
- Define a custom message type with fields for:
  - Cross-track error
  - Current velocity
  - Distance to next waypoint
  - Completion percentage
  - Other relevant metrics
- Implement the message publisher in your node
- Document the message structure and usage

This will demonstrate understanding of:
- ROS2 message definitions
- Custom interface creation
- Message publishing patterns 
