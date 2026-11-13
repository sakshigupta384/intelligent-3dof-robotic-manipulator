
import math
from datetime import datetime

import streamlit as st
import plotly.graph_objects as go

# ==========================================
# 1. ROBOT CONFIGURATION
# ==========================================

H = 5
L1 = 10
L2 = 10

st.set_page_config(
    page_title="Intelligent 3-DOF Robotic Manipulator",
    layout="wide"
)

st.title("Intelligent 3-DOF Robotic Manipulator")
st.caption(
    "Inverse Kinematics | Forward Kinematics | "
    "Digital Twin | Adaptive Motion Planning | Telemetry"
)

# ==========================================
# 2. SESSION STATE
# ==========================================

if "telemetry_history" not in st.session_state:
    st.session_state.telemetry_history = []

if "plan_result" not in st.session_state:
    st.session_state.plan_result = None

# ==========================================
# 3. SIDEBAR CONTROLS
# ==========================================

st.sidebar.header("Robot Target Controls")

if st.sidebar.button("Reset to Home Position"):
    st.session_state.target_x = 10.0
    st.session_state.target_y = 0.0
    st.session_state.target_z = 15.0

x = st.sidebar.slider(
    "Target X (cm)",
    -100.0, 100.0, 10.0, 0.5,
    key="target_x"
)

y = st.sidebar.slider(
    "Target Y (cm)",
    -20.0, 20.0, 0.0, 0.5,
    key="target_y"
)

z = st.sidebar.slider(
    "Target Z (cm)",
    0.0, 25.0, 15.0, 0.5,
    key="target_z"
)

# ==========================================
# 4. INVERSE KINEMATICS
# ==========================================

def calculate_ik(x, y, z):

    r = math.hypot(x, y)
    vertical = z - H

    if r < 1e-9:
        raise ValueError(
            "Target is on the base axis. "
            "Choose non-zero X or Y."
        )

    D = (
        r**2 + vertical**2 - L1**2 - L2**2
    ) / (2 * L1 * L2)

    if D < -1 or D > 1:
        raise ValueError(
            "Target is outside the robot workspace."
        )

    D = max(-1.0, min(1.0, D))

    q1 = math.atan2(y, x)
    q3 = math.acos(D)

    q2 = math.atan2(vertical, r) - math.atan2(
        L2 * math.sin(q3),
        L1 + L2 * math.cos(q3)
    )

    return q1, q2, q3


# ==========================================
# 5. FORWARD KINEMATICS
# ==========================================

def calculate_fk(q1, q2, q3):

    p0 = (0, 0, H)

    r1 = L1 * math.cos(q2)

    p1 = (
        r1 * math.cos(q1),
        r1 * math.sin(q1),
        H + L1 * math.sin(q2)
    )

    r2 = (
        L1 * math.cos(q2)
        + L2 * math.cos(q2 + q3)
    )

    p2 = (
        r2 * math.cos(q1),
        r2 * math.sin(q1),
        H + L1 * math.sin(q2)
        + L2 * math.sin(q2 + q3)
    )

    return p0, p1, p2


# ==========================================
# 6. MOTION PLANNING FUNCTIONS
# ==========================================

def is_reachable(point):

    try:
        calculate_ik(*point)
        return True

    except ValueError:
        return False


def interpolate_path(start, end, steps=30):

    path = []

    for i in range(steps + 1):

        t = i / steps

        point = (
            start[0] + t * (end[0] - start[0]),
            start[1] + t * (end[1] - start[1]),
            start[2] + t * (end[2] - start[2])
        )

        path.append(point)

    return path


def path_is_clear(path, obstacle, radius):

    for point in path:

        if not is_reachable(point):
            return False

        distance = math.dist(point, obstacle)

        if distance <= radius:
            return False

    return True


def plan_adaptive_path(start, goal, obstacle, radius):

    # Attempt 1: Direct trajectory

    direct_path = interpolate_path(start, goal)

    if path_is_clear(direct_path, obstacle, radius):

        return direct_path, "Direct path selected"

    # Attempt 2: Alternative waypoint paths

    midpoint = (
        (start[0] + goal[0]) / 2,
        (start[1] + goal[1]) / 2,
        (start[2] + goal[2]) / 2
    )

    offset = radius + 3

    candidates = [
        (
            midpoint[0],
            midpoint[1] + offset,
            midpoint[2]
        ),
        (
            midpoint[0],
            midpoint[1] - offset,
            midpoint[2]
        ),
        (
            midpoint[0],
            midpoint[1],
            midpoint[2] + offset
        )
    ]

    for waypoint in candidates:

        first_part = interpolate_path(start, waypoint)
        second_part = interpolate_path(waypoint, goal)

        complete_path = first_part + second_part[1:]

        if path_is_clear(complete_path, obstacle, radius):

            return (
                complete_path,
                "Alternative waypoint path selected"
            )

    return [], "No collision-free reachable path found"


# ==========================================
# 7. MAIN ROBOT DASHBOARD
# ==========================================

try:

    q1, q2, q3 = calculate_ik(x, y, z)

    p0, p1, p2 = calculate_fk(q1, q2, q3)

    error = math.dist(p2, (x, y, z))

    angles = [
        math.degrees(q1),
        math.degrees(q2),
        math.degrees(q3)
    ]

    # ======================================
    # 8. TELEMETRY DATA COLLECTION
    # ======================================

    history = st.session_state.telemetry_history

    if (
        not history
        or history[-1]["Target X"] != x
        or history[-1]["Target Y"] != y
        or history[-1]["Target Z"] != z
    ):

        history.append({

            "Time": datetime.now().strftime("%H:%M:%S"),

            "Target X": x,
            "Target Y": y,
            "Target Z": z,

            "Base Angle": angles[0],
            "Shoulder Angle": angles[1],
            "Elbow Angle": angles[2],

            "Actual X": p2[0],
            "Actual Y": p2[1],
            "Actual Z": p2[2],

            "Position Error": error

        })

        if len(history) > 100:
            history.pop(0)

    # ======================================
    # 9. SYSTEM STATUS
    # ======================================

    st.success("Robot Status: Target is reachable")

    st.subheader("Robot Telemetry Overview")

    c1, c2, c3, c4 = st.columns(4)

    c1.metric("Base Angle", f"{angles[0]:.2f}°")
    c2.metric("Shoulder Angle", f"{angles[1]:.2f}°")
    c3.metric("Elbow Angle", f"{angles[2]:.2f}°")
    c4.metric("Position Error", f"{error:.6f} cm")

    st.divider()

    # ======================================
    # 10. DASHBOARD TABS
    # ======================================

    tab1, tab2, tab3, tab4 = st.tabs([

        "3D Robot Visualization",
        "Telemetry Monitoring",
        "Movement History",
        "Adaptive Motion Planning"

    ])

    # ======================================
    # TAB 1: 3D ROBOT VISUALIZATION
    # ======================================

    with tab1:

        col1, col2 = st.columns([2, 1])

        with col1:

            fig = go.Figure()

            # Base support

            fig.add_trace(go.Scatter3d(
                x=[0, p0[0]],
                y=[0, p0[1]],
                z=[0, p0[2]],
                mode="lines",
                line=dict(color="gray", width=12),
                name="Base Support"
            ))

            # Shoulder link

            fig.add_trace(go.Scatter3d(
                x=[p0[0], p1[0]],
                y=[p0[1], p1[1]],
                z=[p0[2], p1[2]],
                mode="lines+markers",
                line=dict(color="royalblue", width=12),
                marker=dict(size=8),
                name="Shoulder Link"
            ))

            # Elbow link

            fig.add_trace(go.Scatter3d(
                x=[p1[0], p2[0]],
                y=[p1[1], p2[1]],
                z=[p1[2], p2[2]],
                mode="lines+markers",
                line=dict(color="darkorange", width=12),
                marker=dict(size=8),
                name="Elbow Link"
            ))

            # Joint labels

            for point, label in zip(
                [p0, p1, p2],
                ["Shoulder", "Elbow", "End Effector"]
            ):

                fig.add_trace(go.Scatter3d(
                    x=[point[0]],
                    y=[point[1]],
                    z=[point[2]],
                    mode="markers+text",
                    marker=dict(size=6, color="red"),
                    text=[label],
                    textposition="top center",
                    name=label
                ))

            # Target marker

            fig.add_trace(go.Scatter3d(
                x=[x],
                y=[y],
                z=[z],
                mode="markers",
                marker=dict(
                    size=9,
                    color="green",
                    symbol="diamond"
                ),
                name="Target"
            ))

            fig.update_layout(
                title="Interactive 3D Robot Arm",
                scene=dict(
                    xaxis_title="X (cm)",
                    yaxis_title="Y (cm)",
                    zaxis_title="Z (cm)",
                    aspectmode="cube",
                    xaxis=dict(range=[-22, 22]),
                    yaxis=dict(range=[-22, 22]),
                    zaxis=dict(range=[0, 25])
                ),
                height=650
            )

            st.plotly_chart(
                fig,
                use_container_width=True
            )

        with col2:

            st.subheader("Joint Angles")

            st.metric("Base", f"{angles[0]:.2f}°")
            st.metric("Shoulder", f"{angles[1]:.2f}°")
            st.metric("Elbow", f"{angles[2]:.2f}°")

            st.divider()

            st.subheader("End-Effector Position")

            st.write(f"X: {p2[0]:.2f} cm")
            st.write(f"Y: {p2[1]:.2f} cm")
            st.write(f"Z: {p2[2]:.2f} cm")

            st.metric("Position Error", f"{error:.6f} cm")

            st.subheader("Target Coordinates")

            st.write(f"X: {x:.2f} cm")
            st.write(f"Y: {y:.2f} cm")
            st.write(f"Z: {z:.2f} cm")

    # ======================================
    # TAB 2: TELEMETRY MONITORING
    # ======================================

    with tab2:

        st.subheader("Joint Angle Monitoring")

        if history:

            fig_angles = go.Figure()

            for key, label in [

                ("Base Angle", "Base"),
                ("Shoulder Angle", "Shoulder"),
                ("Elbow Angle", "Elbow")

            ]:

                fig_angles.add_trace(go.Scatter(
                    x=list(range(1, len(history) + 1)),
                    y=[item[key] for item in history],
                    mode="lines+markers",
                    name=label
                ))

            fig_angles.update_layout(
                title="Joint Angles Over Movement History",
                xaxis_title="Reading Number",
                yaxis_title="Angle (degrees)",
                height=400
            )

            st.plotly_chart(
                fig_angles,
                use_container_width=True
            )

            st.subheader("End-Effector XYZ Monitoring")

            fig_xyz = go.Figure()

            for key, label in [

                ("Actual X", "X Position"),
                ("Actual Y", "Y Position"),
                ("Actual Z", "Z Position")

            ]:

                fig_xyz.add_trace(go.Scatter(
                    x=list(range(1, len(history) + 1)),
                    y=[item[key] for item in history],
                    mode="lines+markers",
                    name=label
                ))

            fig_xyz.update_layout(
                title="End-Effector Position History",
                xaxis_title="Reading Number",
                yaxis_title="Position (cm)",
                height=400
            )

            st.plotly_chart(
                fig_xyz,
                use_container_width=True
            )

            st.subheader("Target vs Actual Position")

            fig_position = go.Figure()

            fig_position.add_trace(go.Bar(
                x=["X", "Y", "Z"],
                y=[x, y, z],
                name="Target Position"
            ))

            fig_position.add_trace(go.Bar(
                x=["X", "Y", "Z"],
                y=[p2[0], p2[1], p2[2]],
                name="Actual Position"
            ))

            fig_position.update_layout(
                title="Target vs Actual Coordinates",
                xaxis_title="Coordinate",
                yaxis_title="Position (cm)",
                barmode="group",
                height=400
            )

            st.plotly_chart(
                fig_position,
                use_container_width=True
            )

            st.subheader("Position Error Monitoring")

            fig_error = go.Figure()

            fig_error.add_trace(go.Scatter(
                x=list(range(1, len(history) + 1)),
                y=[item["Position Error"] for item in history],
                mode="lines+markers",
                name="Position Error"
            ))

            fig_error.update_layout(
                title="Position Error History",
                xaxis_title="Reading Number",
                yaxis_title="Error (cm)",
                height=350
            )

            st.plotly_chart(
                fig_error,
                use_container_width=True
            )

        else:

            st.info("No telemetry readings available yet.")

    # ======================================
    # TAB 3: MOVEMENT HISTORY
    # ======================================

    with tab3:

        st.subheader("Robot Movement History")

        st.write(
            "Previous target configurations "
            "and calculated robot positions."
        )

        st.metric(
            "Total Recorded Readings",
            len(history)
        )

        if history:

            st.dataframe(
                list(reversed(history)),
                use_container_width=True,
                hide_index=True
            )

            if st.button("Clear Movement History"):

                st.session_state.telemetry_history = []
                st.rerun()

        else:

            st.info("Movement history is empty.")

    # ======================================
    # TAB 4: ADAPTIVE MOTION PLANNING
    # ======================================

    with tab4:

        st.subheader("Adaptive Motion Planning")

        st.write(
            "Generate a trajectory between two "
            "robot positions with obstacle checking."
        )

        st.info(
            "The planner checks sampled end-effector "
            "positions against a spherical obstacle."
        )

        # Start position

        st.markdown("### Start Position")

        c1, c2, c3 = st.columns(3)

        with c1:
            sx = st.number_input(
                "Start X (cm)",
                value=10.0,
                key="plan_sx"
            )

        with c2:
            sy = st.number_input(
                "Start Y (cm)",
                value=0.0,
                key="plan_sy"
            )

        with c3:
            sz = st.number_input(
                "Start Z (cm)",
                value=15.0,
                key="plan_sz"
            )

        # Goal position

        st.markdown("### Goal Position")

        c1, c2, c3 = st.columns(3)

        with c1:
            gx = st.number_input(
                "Goal X (cm)",
                value=10.0,
                key="plan_gx"
            )

        with c2:
            gy = st.number_input(
                "Goal Y (cm)",
                value=5.0,
                key="plan_gy"
            )

        with c3:
            gz = st.number_input(
                "Goal Z (cm)",
                value=15.0,
                key="plan_gz"
            )

        # Obstacle configuration

        st.markdown("### Obstacle Configuration")

        c1, c2, c3, c4 = st.columns(4)

        with c1:
            ox = st.number_input(
                "Obstacle X (cm)",
                value=11.0,
                key="obstacle_x"
            )

        with c2:
            oy = st.number_input(
                "Obstacle Y (cm)",
                value=2.5,
                key="obstacle_y"
            )

        with c3:
            oz = st.number_input(
                "Obstacle Z (cm)",
                value=15.0,
                key="obstacle_z"
            )

        with c4:
            radius = st.number_input(
                "Obstacle Radius (cm)",
                min_value=0.5,
                max_value=8.0,
                value=2.0,
                step=0.5,
                key="obstacle_radius"
            )

        # Generate path

        if st.button(
            "Generate Adaptive Motion Plan",
            type="primary"
        ):

            start = (sx, sy, sz)
            goal = (gx, gy, gz)
            obstacle = (ox, oy, oz)

            if not is_reachable(start):

                st.session_state.plan_result = (
                    [],
                    "Start position is unreachable."
                )

            elif not is_reachable(goal):

                st.session_state.plan_result = (
                    [],
                    "Goal position is unreachable."
                )

            else:

                st.session_state.plan_result = (
                    plan_adaptive_path(
                        start,
                        goal,
                        obstacle,
                        radius
                    )
                )

        # Display planning results

        if st.session_state.plan_result is not None:

            planned_path, plan_message = (
                st.session_state.plan_result
            )

            if planned_path:

                st.success(plan_message)

                total_distance = sum(
                    math.dist(a, b)
                    for a, b in zip(
                        planned_path[:-1],
                        planned_path[1:]
                    )
                )

                c1, c2 = st.columns(2)

                c1.metric(
                    "Total Waypoints",
                    len(planned_path)
                )

                c2.metric(
                    "Path Distance",
                    f"{total_distance:.2f} cm"
                )

                # 3D path visualization

                path_fig = go.Figure()

                path_fig.add_trace(go.Scatter3d(
                    x=[p[0] for p in planned_path],
                    y=[p[1] for p in planned_path],
                    z=[p[2] for p in planned_path],
                    mode="lines+markers",
                    line=dict(
                        color="royalblue",
                        width=7
                    ),
                    marker=dict(size=3),
                    name="Planned Trajectory"
                ))

                path_fig.add_trace(go.Scatter3d(
                    x=[sx, gx],
                    y=[sy, gy],
                    z=[sz, gz],
                    mode="markers+text",
                    marker=dict(
                        size=8,
                        color="green"
                    ),
                    text=["Start", "Goal"],
                    name="Start and Goal"
                ))

                path_fig.add_trace(go.Scatter3d(
                    x=[ox],
                    y=[oy],
                    z=[oz],
                    mode="markers+text",
                    marker=dict(
                        size=12,
                        color="red",
                        symbol="x"
                    ),
                    text=["Obstacle Center"],
                    name="Obstacle"
                ))

                path_fig.update_layout(
                    title="Adaptive Robot Trajectory",
                    scene=dict(
                        xaxis_title="X (cm)",
                        yaxis_title="Y (cm)",
                        zaxis_title="Z (cm)",
                        aspectmode="cube"
                    ),
                    height=550
                )

                st.plotly_chart(
                    path_fig,
                    use_container_width=True
                )

                # Waypoint table

                st.subheader("Generated Waypoints")

                waypoint_data = [

                    {
                        "Waypoint": i + 1,
                        "X (cm)": round(p[0], 2),
                        "Y (cm)": round(p[1], 2),
                        "Z (cm)": round(p[2], 2)
                    }

                    for i, p in enumerate(planned_path)

                ]

                st.dataframe(
                    waypoint_data,
                    use_container_width=True,
                    hide_index=True
                )

            else:

                st.error(plan_message)

# ==========================================
# 11. ERROR HANDLING
# ==========================================

except ValueError as e:

    st.error(str(e))

    st.warning(
        "Please select a reachable target position."
    )

# ==========================================
# 12. FOOTER
# ==========================================

st.divider()

st.caption(
    "Intelligent 3-DOF Robotic Manipulator | "
    "IK | FK | Digital Twin | "
    "Adaptive Motion Planning | Simulated Telemetry"
)
