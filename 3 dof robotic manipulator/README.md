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