## Dashboard Modules

### 1. 3D Robot Visualization
Interactive visualization of the 3-DOF robotic manipulator.

### 2. Telemetry Monitoring
Displays simulated joint angles, end-effector coordinates,
and position error through graphs and metrics.

### 3. Movement History
Maintains a history of recorded robot target movements.

### 4. Adaptive Motion Planning
Generates a simulated path between start and goal positions
while checking reachability and a spherical obstacle.
## System Architecture

The project is divided into five major layers:

1. Input Layer: Accepts target X, Y and Z coordinates.
2. Kinematics Layer: Performs inverse and forward kinematics calculations.
3. Visualization Layer: Displays the robotic arm in an interactive 3D environment.
4. Telemetry Layer: Monitors simulated joint angles, end-effector coordinates and position error.
5. Motion Planning Layer: Generates simulated paths while checking reachability and spherical obstacle clearance.

## Technology Stack

- Python
- Streamlit
- Plotly
- NumPy
- Matplotlib
- Pandas
- Tinkercad for circuit simulation 
## Installation and Setup

### Requirements
- Python 3.10 or later
- VS Code
- Required Python libraries

### Install Dependencies

```bash
pip install -r requirements.txt
## Testing and Validation

The following tests were performed on the software prototype:

1. Inverse Kinematics: Tested target coordinate inputs and joint angle calculations.
2. Forward Kinematics: Verified the calculated end-effector position.
3. 3D Visualization: Verified that the robot visualization updates when target coordinates change.
4. Telemetry Monitoring: Checked simulated joint angle and position data.
5. Adaptive Motion Planning: Tested direct paths, alternative waypoint paths, unreachable targets and failed path searches.
6. Tinkercad Simulation: Tested the movement of all three servo motors and the home position.

### Position Error Test

A test configuration produced a position error of 0 cm.
## Project Limitations

- The robotic arm is currently a software and simulation prototype.
- Telemetry values are simulated, not collected from physical sensors.
- Adaptive motion planning checks sampled end-effector positions against one spherical obstacle.
- Complete robot-link collision detection is not implemented.
- Physical servo control is not integrated with the dashboard.

## Future Scope

- Integrate physical servo motors with a microcontroller.
- Add real-time sensor-based telemetry.
- Implement complete robot-link collision detection.
- Improve adaptive motion planning with multiple obstacles.
- Explore predictive analytics for robot performance monitoring.
## Project File Structure

```text
3 dof robotic manipulator/
│
├── inverse_kinematics.py
├── robot_3d.py
├── robot_3d_backup.py
├── serial_sender.py
├── serial_test.py
├── telemetry_dashboard.py
├── test_inverse_kinematics.py
├── requirements.txt
└── README.md