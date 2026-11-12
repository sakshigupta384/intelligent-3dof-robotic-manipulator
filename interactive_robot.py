
import math
import streamlit as st
import plotly.graph_objects as go

# Robot dimensions in cm
H = 5
L1 = 10
L2 = 10

st.set_page_config(page_title="3-DOF Robot Arm", layout="wide")
st.title("3-DOF Robotic Manipulator")
st.caption("Interactive Inverse Kinematics and 3D Visualization")

st.sidebar.header("Target Coordinates")

x = st.sidebar.slider("Target X (cm)", -20.0, 20.0, 10.0, 0.5)
y = st.sidebar.slider("Target Y (cm)", -20.0, 20.0, 0.0, 0.5)
z = st.sidebar.slider("Target Z (cm)", 0.0, 25.0, 15.0, 0.5)

def calculate_ik(x, y, z):
    r = math.hypot(x, y)
    vertical = z - H

    if r < 1e-9:
        raise ValueError("Target is on the base axis; choose non-zero X or Y.")

    D = (r*r + vertical*vertical - L1*L1 - L2*L2) / (2*L1*L2)

    if D < -1 or D > 1:
        raise ValueError("Target is outside the robot workspace.")

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
    p1 = (r1 * math.cos(q1), r1 * math.sin(q1),
          H + L1 * math.sin(q2))

    r2 = L1 * math.cos(q2) + L2 * math.cos(q2 + q3)
    p2 = (r2 * math.cos(q1), r2 * math.sin(q1),
          H + L1 * math.sin(q2) + L2 * math.sin(q2 + q3))

    return p0, p1, p2

try:
    q1, q2, q3 = calculate_ik(x, y, z)
    p0, p1, p2 = calculate_fk(q1, q2, q3)

    error = math.dist(p2, (x, y, z))

    col1, col2 = st.columns([2, 1])

    with col1:
        fig = go.Figure()

        fig.add_trace(go.Scatter3d(
            x=[p0[0], p1[0], p2[0]],
            y=[p0[1], p1[1], p2[1]],
            z=[p0[2], p1[2], p2[2]],
            mode="lines+markers",
            line=dict(width=10),
            marker=dict(size=7),
            name="Robot Arm"
        ))

        fig.add_trace(go.Scatter3d(
            x=[x], y=[y], z=[z],
            mode="markers",
            marker=dict(size=9, color="red", symbol="diamond"),
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
            height=600
        )

        st.plotly_chart(fig, use_container_width=True)

    with col2:
        st.subheader("Joint Angles")
        st.metric("Base", f"{math.degrees(q1):.2f}°")
        st.metric("Shoulder", f"{math.degrees(q2):.2f}°")
        st.metric("Elbow", f"{math.degrees(q3):.2f}°")

        st.subheader("Position")
        st.write(f"X: {p2[0]:.2f} cm")
        st.write(f"Y: {p2[1]:.2f} cm")
        st.write(f"Z: {p2[2]:.2f} cm")
        st.metric("Position Error", f"{error:.6f} cm")

        st.success("Target reached mathematically!")

except ValueError as e:
    st.error(str(e))
