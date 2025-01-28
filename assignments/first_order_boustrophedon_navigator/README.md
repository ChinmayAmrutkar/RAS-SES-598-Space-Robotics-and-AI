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
---
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
---
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
---
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
---
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
| 7 | 7.0 | 0.5 | 10.0 | 0.02 | 0.5 | 0.045 | 0.133 | Very good | Smooth | Good evenly spacing | Very good | Close to target |
| **10** | **7.0** | **0.6** | **10.0** | **0.01** | **0.4** | **0.055** | **0.133** | **Excellent** | **Perfect transitions** | **Optimal** | **Optimal spacing** | **Best Performance** ✅ |
| 11 | 8.0 | 0.6 | 10.0 | 0.01 | 0.3 | 0.055 | 0.134| Too aggressive | Sharp transitions | Good | Slight overlap | Post-optimal test |
| 12 | 9.0 | 0.3 | 10.0 | 0.01 | 0.2 | 0.051 | 0.134 | Oscillatory | Overshooting | Moderate | Excessive overlap | Too aggressive |

---
## Implemented Solution
![image](https://github.com/user-attachments/assets/5de964cc-d75b-4112-876c-922ea1d05dab)

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
   - Linear velocity maintained
   - Smooth transitions between path segments
   - Stable angular velocity during turns

3. **Pattern Quality:**
   - Optimal coverage
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

## Performance Plots

### Cross-Track Error Analysis
Image
#### Key Observations:
- Average cross-track error: 0.055 units
- Maximum deviation: 0.133 units
- Error remains consistently below target threshold (0.2 units)
- Small periodic variations correspond to path segments
- Quick recovery from disturbances

### Robot Trajectory
Image
#### Pattern Characteristics:
- Uniform spacing between lines (0.4 units)
- Clean, consistent turns
- Parallel path segments
- Complete area coverage
- Minimal path overlap

### Velocity Profiles

#### Linear Velocity
Image
**Analysis:**
- Steady-state velocity: ~0.632 m/s
- Consistent speed during straight segments
- Smooth acceleration/deceleration
- Minimal velocity fluctuations

#### Angular Velocity
Image

**Analysis:**
- Peak angular velocity during turns: ±1.2 rad/s
- Sharp, precise turning behavior
- Clean transitions between turns
- Minimal oscillations


## Performance Analysis

### System Stability
- Consistent cross-track error around 0.055 units
- Smooth velocity profiles throughout execution
- Stable cornering behavior

### Coverage Efficiency
- Achieved optimal coverage
- Uniform pattern spacing

### Motion Quality
- Linear velocity maintained steady state
- Angular velocity appropriately peaked during turns
- Minimal oscillations in straight-line segments

## Conclusions

The implemented solution successfully achieved:
- Precise path following with minimal cross-track error
- Complete area coverage with uniform spacing
- Stable and efficient motion throughout the pattern
- All target performance metrics met

The final parameter configuration represents an optimal balance between:
- Control responsiveness and stability
- Coverage completeness and efficiency
- Motion smoothness and precision
---
## Custom Message Performance Metrics Implementation in ROS2

The implementation of a custom ROS2 message allows comprehensive monitoring of the robot's performance during pattern execution by publishing detailed performance metrics in the boustrophedon navigator project.

### Custom Message Package Setup

#### 1. Create the Package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake metrics_interfaces
```

#### 2. Directory Structure
```
metrics_interfaces/
├── msg/
│   └── PerformanceMetrics.msg
├── package.xml
└── CMakeLists.txt
```

#### 3. Message Definition
Create the message file at `metrics_interfaces/msg/PerformanceMetrics.msg`:

```
# Performance metrics for boustrophedon pattern execution
float64 cross_track_error          
float64 linear_velocity            
float64 angular_velocity           
float64 distance_to_next_waypoint  
float64 completion_percentage      
float64 avg_cross_track_error      
float64 max_cross_track_error      
float64 angular_error             
```

#### 4. Package Configuration

##### Update package.xml
Add the following dependencies to `package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>metrics_interfaces</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="chinmayamrutkar01@gmail.com">chinmay</maintainer>
  <license>TODO: License declaration</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend> 
  <member_of_group>rosidl_interface_packages</member_of_group>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

##### Update CMakeLists.txt
Add the following to `CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(metrics_interfaces)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PerformanceMetrics.msg"
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

### Building and Verifying the Package

1. **Build the package:**
```bash
cd ~/ros2_ws
colcon build --packages-select metrics_interfaces
source install/setup.bash
```

2. **Verify the message type:**
```bash
# Check if message type is available
ros2 interface show metrics_interfaces/msg/PerformanceMetrics
```

### Using the Custom Message

#### 1. In BoustrophedonController Node

##### Add Dependencies
Update first_order_boustrophedon_navigator package's `package.xml`:
```xml
<depend>metrics_interfaces</depend>
```

#### Implementation 
```python
#!/usr/bin/env python3

from collections import deque
from std_msgs.msg import Float64
from rcl_interfaces.msg import SetParametersResult
from metrics_interfaces.msg import PerformanceMetrics


class BoustrophedonController(Node):
  def __init__(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_linear', 7.0),
                ('Kd_linear', 0.6),
                ('Kp_angular', 10),
                ('Kd_angular', 0.01),
                ('spacing', 0.4)
            ]
        )
        # Create publisher and subscriber
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        #Custom ROS2 message type to publish detailed performance metrics
        self.performance_metrics_publisher = self.create_publisher(PerformanceMetrics, 'performance_metrics', 10) 
        # Lawnmower pattern parameters
        self.waypoints = self.generate_waypoints()
        self.current_waypoint = 0

    def publish_performace_metrics(self, cross_track_error, linear_velocity, angular_velocity, angular_error):
        metrics_msg = PerformanceMetrics()
        metrics_msg.cross_track_error = cross_track_error
        metrics_msg.linear_velocity = linear_velocity
        metrics_msg.angular_velocity = angular_velocity
        metrics_msg.distance_to_next_waypoint = self.get_distance(self.pose.x, 
                                                      self.pose.y, 
                                                      self.waypoints[self.current_waypoint-1][0], 
                                                      self.waypoints[self.current_waypoint-1][1])
        
        metrics_msg.completion_percentage = (self.current_waypoint / (len(self.waypoints))) * 100
        
        if len(self.cross_track_errors) > 1:
            metrics_msg.avg_cross_track_error = (sum(self.cross_track_errors)/len(self.cross_track_errors))
            metrics_msg.max_cross_track_error = max(self.cross_track_errors)
        else :
            metrics_msg.avg_cross_track_error = 0.0
            metrics_msg.max_cross_track_error = 0.0
        metrics_msg.angular_error = angular_error
        self.performance_metrics_publisher.publish(metrics_msg)

    def control_loop(self):

        #Publishing Metrics Message
        self.publish_performace_metrics(cross_track_error, linear_velocity, angular_velocity, angular_error)


```

### 2. Monitoring Metrics

#### Using Command Line
```bash
# Monitor metrics in real-time
ros2 topic echo /performance_metrics
```

#### Using rqt_plot
```bash
ros2 run rqt_plot rqt_plot
```
Add topics:
- `/performance_metrics/cross_track_error`
- `/performance_metrics/linear_velocity`
- `/performance_metrics/angular_velocity`
- `/performance_metrics/completion_percentage`
---
## Challenges and Solutions

### 1. Parameter Tuning

#### Challenge:
Manual tuning of multiple PD controller parameters was time-consuming and often resulted in suboptimal performance.

#### Solution:
- Developed systematic tuning methodology by listing out all the outcomes achieved for different Kp_linear, Kd_linear, Kp_angular, Kd_angular and spacing values
- Started with conservative gains and incrementally adjusted
- Used rqt_reconfigure for real-time parameter adjustment
- Documented parameter effects and interactions
- Results: Found optimal parameter set through structured testing

### 2. System Stability

#### Challenge:
Early implementations showed oscillatory behavior and instability, especially during direction changes.

#### Solution:
- Carefully tuned derivative gains (Kd_linear and Kd_angular)
- Implemented error threshold monitoring
- Added stability checks in control loop
- Results: Achieved stable operation throughout the pattern
---
### Future Improvements

1. **Adaptive Control**
   - Implement gain scheduling based on path segments
   - Add dynamic parameter adjustment

2. **Optimization**
   - Develop automated parameter tuning
   - Implement path optimization algorithms

3. **Enhanced Monitoring**
   - Add visualization tools for coverage mapping
   - Implement performance prediction

4. **Robustness**
   - Add disturbance rejection capabilities
   - Implement fault detection and recovery
---
### Lessons Learned

1. **Controller Design**
   - The Proportional (P) component serves as the primary driver of the system response:
     - Acts as an immediate response to position errors
     - Larger P gains result in more aggressive corrections
     - Too high P gains can lead to overshooting and oscillations
     - Too low P gains result in sluggish response
   - The Derivative (D) component acts as a motion predictor:
     - Monitors the rate of change of the error
     - Provides damping effect to prevent overshooting
     - Higher D gains smooth the motion but can make system sluggish
     - Too low D gains may not sufficiently dampen oscillations
   - Parameter tuning requires systematic approach
   - Balance between P and D gains is crucial for optimal performance

2. **Pattern Generation**
   - Spacing parameter significantly affects coverage quality
   - Corner handling requires special attention

3. **Implementation**
   - Real-time monitoring is essential for tuning
   - Custom message types enhance analysis capabilities
---
