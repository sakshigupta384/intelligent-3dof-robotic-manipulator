
import math
import numpy as np
import matplotlib.pyplot as plt

# Robot dimensions (cm)
H = 5
L1 = 10
L2 = 10

def inverse_kinematics(x, y, z):
    r = math.sqrt(x**2 + y**2)
    vertical = z - H

    D = (r**2 + vertical**2 - L1**2 - L2**2) / (2 * L1 * L2)

    if D < -1 or D > 1:
        raise ValueError("Target is outside robot workspace!")

    # Elbow-up mathematical solution
    q3 = math.acos(D)
    q2 = math.atan2(vertical, r) - math.atan2(
        L2 * math.sin(q3),
        L1 + L2 * math.cos(q3)
    )
    q1 = math.atan2(y, x)

    return q1, q2, q3


def forward_kinematics(q1, q2, q3):
    # Shoulder position
    x1 = 0
    y1 = 0
    z1 = H

    # Elbow position
    r2 = L1 * math.cos(q2)
    x2 = r2 * math.cos(q1)
    y2 = r2 * math.sin(q1)
    z2 = H + L1 * math.sin(q2)

    # End-effector position
    r3 = L1 * math.cos(q2) + L2 * math.cos(q2 + q3)
    x3 = r3 * math.cos(q1)
    y3 = r3 * math.sin(q1)
    z3 = H + L1 * math.sin(q2) + L2 * math.sin(q2 + q3)

    return (x1, y1, z1), (x2, y2, z2), (x3, y3, z3)


# Target position
target_x = 10
target_y = 0
target_z = 15

q1, q2, q3 = inverse_kinematics(
    target_x, target_y, target_z
)

p0, p1, p2 = forward_kinematics(q1, q2, q3)

# Plot 3D robot
fig = plt.figure(figsize=(9, 7))
ax = fig.add_subplot(111, projection="3d")

# Base
ax.scatter(0, 0, 0, color="black", s=100, label="Base")

# Arm links
ax.plot(
    [p0[0], p1[0], p2[0]],
    [p0[1], p1[1], p2[1]],
    [p0[2], p1[2], p2[2]],
    marker="o",
    linewidth=6,
    markersize=10,
    label="Robot arm"
)

# Target marker
ax.scatter(
    target_x, target_y, target_z,
    color="red", marker="*", s=180,
    label="Target position"
)

ax.set_xlabel("X (cm)")
ax.set_ylabel("Y (cm)")
ax.set_zlabel("Z (cm)")
ax.set_title("3-DOF Robotic Manipulator")
ax.set_xlim(-22, 22)
ax.set_ylim(-22, 22)
ax.set_zlim(0, 25)
ax.legend()

print("3-DOF Robotic Manipulator")
print("-------------------------")
print("Base angle:", round(math.degrees(q1), 2))
print("Shoulder angle:", round(math.degrees(q2), 2))
print("Elbow angle:", round(math.degrees(q3), 2))
print("End-effector:", p2)
print("Target:", (target_x, target_y, target_z))
print("Position error:", round(
    math.dist(p2, (target_x, target_y, target_z)), 6
), "cm")

plt.show()
