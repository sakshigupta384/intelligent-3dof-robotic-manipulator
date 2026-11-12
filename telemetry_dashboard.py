
import math
import pandas as pd
import streamlit as st
import plotly.graph_objects as go

H, L1, L2 = 5, 10, 10

st.set_page_config(page_title="Robot Telemetry", layout="wide")

st.title("3-DOF Robotic Manipulator")
st.subheader("Telemetry and Joint Monitoring System")

st.sidebar.header("Robot Target Control")
x = st.sidebar.slider("Target X (cm)", -20.0, 20.0, 10.0, 0.5)
y = st.sidebar.slider("Target Y (cm)", -20.0, 20.0, 0.0, 0.5)
z = st.sidebar.slider("Target Z (cm)", 0.0, 25.0, 15.0, 0.5)

def calculate_ik(x, y, z):
    r = math.hypot(x, y)
    vertical = z - H

    if r < 1e-9:
        raise ValueError("Target lies on the base axis.")

    D = (r**2 + vertical**2 - L1**2 - L2**2) / (2*L1*L2)

    if not -1 <= D <= 1:
        raise ValueError("Target outside workspace.")

    q1 = math.atan2(y, x)
    q3 = math.acos(D)
    q2 = math.atan2(vertical, r) - math.atan2(
        L2 * math.sin(q3),
        L1 + L2 * math.cos(q3)
    )
    return q1, q2, q3

def calculate_fk(q1, q2, q3):
    p0 = (0, 0, H)

    r1 = L1 * math.cos(q2)
    p1 = (r1*math.cos(q1), r1*math.sin(q1),
          H + L1*math.sin(q2))

    r2 = L1*math.cos(q2) + L2*math.cos(q2+q3)
    p2 = (r2*math.cos(q1), r2*math.sin(q1),
          H + L1*math.sin(q2) + L2*math.sin(q2+q3))

    return p0, p1, p2

try:
    q1, q2, q3 = calculate_ik(x, y, z)
    p0, p1, p2 = calculate_fk(q1, q2, q3)

    angles = [
        math.degrees(q1),
        math.degrees(q2),
        math.degrees(q3)
    ]

    error = math.dist(p2, (x, y, z))

    st.success("Robot target is mathematically reachable")

    # Joint telemetry cards
    st.subheader("Joint Telemetry")
    c1, c2, c3 = st.columns(3)

    c1.metric("Base Angle", f"{angles[0]:.2f}°")
    c2.metric("Shoulder Angle", f"{angles[1]:.2f}°")
    c3.metric("Elbow Angle", f"{angles[2]:.2f}°")

    # Position telemetry
    st.subheader("End-Effector Monitoring")
    a, b, c, d = st.columns(4)

    a.metric("Actual X", f"{p2[0]:.2f} cm")
    b.metric("Actual Y", f"{p2[1]:.2f} cm")
    c.metric("Actual Z", f"{p2[2]:.2f} cm")
    d.metric("Position Error", f"{error:.6f} cm")

    # 3D robot view
    st.subheader("3D Robot Visualization")

    fig = go.Figure()

    fig.add_trace(go.Scatter3d(
        x=[p0[0], p1[0], p2[0]],
        y=[p0[1], p1[1], p2[1]],
        z=[p0[2], p1[2], p2[2]],
        mode="lines+markers",
        line=dict(width=10),
        marker=dict(size=8),
        name="Robot Arm"
    ))

    fig.add_trace(go.Scatter3d(
        x=[x], y=[y], z=[z],
        mode="markers",
        marker=dict(size=9, color="red", symbol="diamond"),
        name="Target"
    ))

    fig.update_layout(
        height=550,
        scene=dict(
            xaxis_title="X (cm)",
            yaxis_title="Y (cm)",
            zaxis_title="Z (cm)",
            aspectmode="cube",
            xaxis=dict(range=[-22, 22]),
            yaxis=dict(range=[-22, 22]),
            zaxis=dict(range=[0, 25])
        )
    )

    st.plotly_chart(fig, use_container_width=True)

    # Joint angle chart
    st.subheader("Joint Angle Analysis")

    chart_data = pd.DataFrame({
        "Joint": ["Base", "Shoulder", "Elbow"],
        "Angle (degrees)": angles
    })

    chart = go.Figure(go.Bar(
        x=chart_data["Joint"],
        y=chart_data["Angle (degrees)"]
    ))

    chart.update_layout(
        yaxis_title="Angle (degrees)",
        height=350
    )

    st.plotly_chart(chart, use_container_width=True)

    st.info(
        "Telemetry source: Python IK/FK simulation. "
        "This is calculated simulation data, not physical sensor feedback."
    )

except ValueError as e:
    st.error(str(e))
    st.warning("Robot status: Target unreachable or singular.")
