# Cart-Pole Optimal Control 

![image](https://github.com/user-attachments/assets/46e046e4-2a3e-49b3-bb92-a8b3b50990d0)


## Overview
This project involves tuning and analyzing an LQR controller for a cart-pole system subject to earthquake disturbances. The goal is to maintain the pole's stability while keeping the cart within its physical constraints under external perturbations. The earthquake force generator introduces simulating and controlling systems under seismic disturbances. The ability to handle dynamic disturbances and maintain system stability is crucial for the optimal control of space-based robotic systems, including applications such as Lunar landers and orbital debris removal robots.

## System Description
The project is based on the problem formalism here: https://underactuated.mit.edu/acrobot.html#cart_pole <br>
<br>
![image](https://github.com/user-attachments/assets/b8c3d79d-6e14-409f-91ef-8a63ab45087d)

### Physical Setup
- Inverted pendulum mounted on a cart
- Cart traversal range: ±2.5m (total range: 5m)
- Pole length: 1m
- Cart mass: 1.0 kg
- Pole mass: 1.0 kg

### Disturbance Generator
The system includes an earthquake force generator that introduces external disturbances:
- Generates continuous, earthquake-like forces using superposition of sine waves
- Base amplitude: 15.0N (default setting)
- Frequency range: 0.5-4.0 Hz (default setting)
- Random variations in amplitude and phase
- Additional Gaussian noise

## Objectives

### Core Requirements
1. Analyze and tune the provided LQR controller to:
   - Maintain the pendulum in an upright position
   - Keep the cart within its ±2.5m physical limits
   - Achieve stable operation under earthquake disturbances
2. Document your LQR tuning approach:
   - Analysis of the existing Q and R matrices
   - Justification for any tuning changes made
   - Analysis of performance trade-offs
   - Experimental results and observations
3. Analyze system performance:
   - Duration of stable operation
   - Maximum cart displacement
   - Pendulum angle deviation
   - Control effort analysis

### Learning Outcomes
- Understanding of LQR control parameters and their effects
- Experience with competing control objectives
- Analysis of system behavior under disturbances
- Practical experience with ROS2 and Gazebo simulation

### Extra Credit Options
Students can implement reinforcement learning for extra credit (up to 30 points):

1. Reinforcement Learning Implementation:
   - Implement a basic DQN (Deep Q-Network) controller
   - Train the agent to stabilize the pendulum
   - Compare performance with the LQR controller
   - Document training process and results
   - Create training progress visualizations
   - Analyze and compare performance with LQR

## Implementation

### Controller Description
The package includes a complete LQR controller implementation (`lqr_controller.py`) with the following features:
- State feedback control
- Configurable Q and R matrices
- Real-time force command generation
- State estimation and processing

Current default parameters:
```python
# State cost matrix Q (default values)
Q = np.diag([1.0, 1.0, 10.0, 10.0])  # [x, x_dot, theta, theta_dot]

# Control cost R (default value)
R = np.array([[0.1]])  # Control effort cost
```

### Earthquake Disturbance
The earthquake generator (`earthquake_force_generator.py`) provides realistic disturbances:
- Configurable through ROS2 parameters
- Default settings:
  ```python
  parameters=[{
      'base_amplitude': 15.0,    # Strong force amplitude (N)
      'frequency_range': [0.5, 4.0],  # Wide frequency range (Hz)
      'update_rate': 50.0  # Update rate (Hz)
  }]
  ```

## Getting Started

### Prerequisites
- ROS2 Jazzy
- Gazebo Garden
- Python 3.8+
- Required Python packages: numpy, scipy

#### Installation Commands
```bash
# Set ROS_DISTRO as per your configuration
export ROS_DISTRO=jazzy

# Install ROS2 packages
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-interfaces \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-rviz2

# Install Python dependencies
pip3 install numpy scipy control
```


### Building and Running
```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select cart_pole_optimal_control --symlink-install

# Source the workspace
source install/setup.bash

# Launch the simulation with visualization
ros2 launch cart_pole_optimal_control cart_pole_rviz.launch.py
```

This will start:
- Gazebo simulation (headless mode)
- RViz visualization showing:
  * Cart-pole system
  * Force arrows (control and disturbance forces)
  * TF frames for system state
- LQR controller
- Earthquake force generator
- Force visualizer

### Visualization Features
The RViz view provides a side perspective of the cart-pole system with:

#### Force Arrows
Two types of forces are visualized:
1. Control Forces (at cart level):
   - Red arrows: Positive control force (right)
   - Blue arrows: Negative control force (left)

2. Earthquake Disturbances (above cart):
   - Orange arrows: Positive disturbance (right)
   - Purple arrows: Negative disturbance (left)

Arrow lengths are proportional to force magnitudes.

## Analysis

### 1. LQR Tuning Approach

#### 1.1 Analysis of Q and R Matrices

- **Baseline (Case 1):**
  - **Q Matrix:** `diag([1.0, 1.0, 10.0, 10.0])`
  - **R Matrix:** `[[0.1]]`
  - *Observation:* The default parameters produced significant cart deviations (reaching the ±2.5 m limit) and moderate pole deviations.

- **Tuning Objectives:**
  - **Improve Cart Regulation:** Increase the weight on cart position to keep the cart closer to the desired setpoint.
  - **Enhance Pendulum Stability:** Increase the weight on the pole angle to reduce deviations.
  - **Control Effort Trade-off:** Adjust R to balance aggressiveness against actuator saturation.
 
#### 1.2 Justification for Tuning Changes
- **Case 1:** Baseline 
- **Case 2:** Increased the cart position weight (Q[0,0] = 50.0) to improve cart regulation.
- **Case 3 & 4:** Increased the pole angle weight (Q[2,2] = 50.0) to force quicker correction of pendulum deviations. (Note: Case 4 yielded worse performance due to a combination of factors, highlighting trade-offs.)
- **Case 5:** Balanced moderate weights on both cart and pole (Q = diag([20.0, 1.0, 20.0, 10.0])) combined with a lower control cost (R = 0.05) to achieve more aggressive control without excessive actuator saturation.

### 2. Experimental Results

We collected the following metrics for each test case over a 180-seconds simulation (or until the cart reached ±2.5 m) using data_logger.py

- **Maximum Pole Angle Deviation (rad)**
- **RMS Cart Position Error (m)**
- **Peak Control Force Used (N)**
- **Max Recovery Time After Disturbances (s)**
- **Maximum Cart Displacement (m)**
- **RMS Pole Angle Deviation (rad)**
- **Longest Stable Duration (s)**
- **Total Control Effort**
- **Average Control Effort**

### 3. Comparison Table

| **Test Case** | **Q Matrix**                                 | **R Matrix** | **Max Pole Angle** (rad) | **RMS Cart Error** (m) | **Peak Control Force** (N) | **Recovery Time** (s) | **Max Cart Displacement** (m) | **Overall Performance** |
|---------------|----------------------------------------------|--------------|--------------------------|------------------------|----------------------------|-----------------------|-------------------------------|-------------------------|
| **Case 1**    | `diag([1.0, 1.0, 10.0, 10.0])`               | 0.1          | 0.1273 ❌                | 0.6029 ❌              | 75.12 ✅                   | 2.87 ❌               | 2.50 ❌                       | ❌                      |
| **Case 2**    | `diag([50.0, 1.0, 10.0, 10.0])`              | 0.1          | 0.0741 ✅                | 0.1089 ✅              | 71.30 ✅                   | 0.00 ✅               | 0.25 ✅                       | ✅                      |
| **Case 3**    | `diag([1.0, 1.0, 50.0, 10.0])`               | 0.1          | 0.1281 ❌                | 0.7107 ❌              | 76.64 ✅                   | 3.04 ❌               | 2.50 ❌                       | ❌                      |
| **Case 4**    | `diag([1.0, 1.0, 50.0, 10.0])`               | 0.1          | 0.1922 ❌                | 1.2798 ❌              | 75.61 ✅                   | 21.96 ❌              | 2.50 ❌                       | ❌                      |
| **Case 5**    | `diag([20.0, 1.0, 20.0, 10.0])`              | 0.05         | 0.0643 ✅                | 0.1529 ✅              | 78.13 ✅                   | 0.00 ✅               | 0.59 ✅                       | ✅                      |

### 4. Analysis and Discussion

#### 4.1 Baseline Performance (Case 1)
- **System Behavior:**  
  The baseline controller (Case 1) allowed the cart to reach the physical limit (±2.5 m), resulting in high RMS cart error and a maximum cart displacement of 2.5 m. Recovery times were moderate, indicating that once disturbed, the system took about 2.87 s to recover.<br>
<br>
CASE_1 Video:<br>
<br>

https://github.com/user-attachments/assets/4ffc95ac-5e5e-48de-8d5f-46d6dfa11622


- **Bottlenecks:**  
  The low cart weight in Q contributed to poor cart regulation, resulting in significant displacement.

#### 4.2 Parameter Effects
- **Cart Position Weight (Case 2):**  
  Increasing the weight on cart position reduced RMS cart error dramatically (0.1089 m) and kept the cart near the setpoint (maximum displacement of only 0.2516 m). This case showed no recovery time (indicating the system remained stable without significant disturbances).<br>

CASE_2 Image:<br>

![image](https://github.com/user-attachments/assets/d16008df-9862-4dc5-8f8a-d2ebeefce644)

- **Pole Angle Weight (Cases 3 & 4):**  
  Increasing the pole angle weight improved the pendulum stability in some respects (lower RMS pole angle in Case 3) but at the cost of higher cart error and, in Case 4, very poor performance with long recovery times (21.96 s) and high RMS cart error (1.2798 m).<br>

CASE_3 Image:<br>

![image](https://github.com/user-attachments/assets/fe3ec03f-6928-49f9-a459-855a68368e50)

CASE_4 Image:<br>

 ![image](https://github.com/user-attachments/assets/757b18d8-d8fc-468e-8769-9e8f404f788b)

- **Control Effort (Case 5):**  
  With a balanced Q (moderate weights on both cart and pole) and a lower R, the controller was more aggressive. This led to the lowest maximum pole angle deviation (0.0643 rad) and low RMS cart error (0.1529 m), along with the longest stable operation (105.29 s). However, the aggressive control also resulted in a higher total control effort.

  <br>
CASE_5 Video:
  <br>


https://github.com/user-attachments/assets/2dd4a5e5-ea5c-43a4-b50c-270a4b1bfc5f



#### 4.3 Disturbance Response
- **Recovery Behavior:**  
  Cases 2 and 5 demonstrated instantaneous or no measurable recovery time, indicating that the system quickly counteracted disturbances. In contrast, Cases 1, 3, and especially 4 exhibited nonzero recovery times, with Case 4 showing severe recovery delays.
- **Control Effort Distribution:**  
  The average control effort varied across cases, highlighting the trade-off between achieving a quick, stable response and minimizing actuator load. Lower R values (as in Case 5) encouraged aggressive control, reducing deviations but at the cost of higher force usage.


## Conclusions

- **Stability and Constraint Satisfaction:**  
  - **Case 2** achieved excellent cart regulation and maintained the cart well within ±2.5 m while keeping the pole near vertical.
  - **Case 5** showed the best overall performance in terms of stability duration and low deviations, with the added trade-off of increased control effort.
- **Performance Trade-offs:**  
  - Increasing Q weights for specific states improves the regulation of those states, but may adversely affect other performance metrics.
  - A higher R value can reduce control effort, yet may compromise the rapidity of disturbance rejection.
- **Overall Recommendation:**  
  Based on our experimental results, **Case 5** appears to offer a balanced solution, with low deviations, extended stable operation, and acceptable control effort, making it a promising candidate for robust operation under earthquake disturbances.


### Extra Credit (up to 30 points)
- Reinforcement Learning Implementation (30 points)

## Tips for Success
1. Start with understanding the existing controller behavior
2. Document baseline performance thoroughly
3. Make systematic parameter adjustments
4. Keep detailed records of all tests
5. Focus on understanding trade-offs
6. Use visualizations effectively

## Submission Requirements
1. Technical report including:
   - Analysis of controller behavior
   - Performance data and plots
   - Discussion of findings
2. Video demonstration of system performance
3. Any additional analysis tools or visualizations created

## License
This work is licensed under a [Creative Commons Attribution 4.0 International License](http://creativecommons.org/licenses/by/4.0/).
[![Creative Commons License](https://i.creativecommons.org/l/by/4.0/88x31.png)](http://creativecommons.org/licenses/by/4.0/) 
