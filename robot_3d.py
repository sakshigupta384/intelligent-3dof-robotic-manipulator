[1mdiff --git a/robot_3d.py b/robot_3d.py[m
[1mindex ff32dfa..04769c2 100644[m
[1m--- a/robot_3d.py[m
[1m+++ b/robot_3d.py[m
[36m@@ -8,6 +8,7 @@[m [mH = 5[m
 L1 = 10[m
 L2 = 10[m
 [m
[32m+[m
 def inverse_kinematics(x, y, z):[m
     r = math.sqrt(x**2 + y**2)[m
     vertical = z - H[m
[36m@@ -17,7 +18,8 @@[m [mdef inverse_kinematics(x, y, z):[m
     if D < -1 or D > 1:[m
         raise ValueError("Target is outside robot workspace!")[m
 [m
[31m-    # Elbow-up mathematical solution[m
[32m+[m[32m    D = max(-1.0, min(1.0, D))[m
[32m+[m
     q3 = math.acos(D)[m
     q2 = math.atan2(vertical, r) - math.atan2([m
         L2 * math.sin(q3),[m
[36m@@ -30,23 +32,40 @@[m [mdef inverse_kinematics(x, y, z):[m
 [m
 def forward_kinematics(q1, q2, q3):[m
     # Shoulder position[m
[31m-    x1 = 0[m
[31m-    y1 = 0[m
[31m-    z1 = H[m
[32m+[m[32m    p0 = (0, 0, H)[m
 [m
     # Elbow position[m
     r2 = L1 * math.cos(q2)[m
     x2 = r2 * math.cos(q1)[m
     y2 = r2 * math.sin(q1)[m
     z2 = H + L1 * math.sin(q2)[m
[32m+[m[32m    p1 = (x2, y2, z2)[m
 [m
     # End-effector position[m
     r3 = L1 * math.cos(q2) + L2 * math.cos(q2 + q3)[m
     x3 = r3 * math.cos(q1)[m
     y3 = r3 * math.sin(q1)[m
     z3 = H + L1 * math.sin(q2) + L2 * math.sin(q2 + q3)[m
[32m+[m[32m    p2 = (x3, y3, z3)[m
[32m+[m
[32m+[m[32m    return p0, p1, p2[m
[32m+[m
 [m
[31m-    return (x1, y1, z1), (x2, y2, z2), (x3, y3, z3)[m
[32m+[m[32mdef draw_base(ax):[m
[32m+[m[32m    # Cylindrical robot base[m
[32m+[m[32m    theta = np.linspace(0, 2 * np.pi, 40)[m
[32m+[m[32m    height = np.linspace(0, H, 10)[m
[32m+[m[32m    theta, height = np.meshgrid(theta, height)[m
[32m+[m
[32m+[m[32m    radius = 2[m
[32m+[m[32m    x = radius * np.cos(theta)[m
[32m+[m[32m    y = radius * np.sin(theta)[m
[32m+[m
[32m+[m[32m    ax.plot_surface([m
[32m+[m[32m        x, y, height,[m
[32m+[m[32m        color="gray",[m
[32m+[m[32m        alpha=0.8[m
[32m+[m[32m    )[m
 [m
 [m
 # Target position[m
[36m@@ -54,55 +73,118 @@[m [mtarget_x = 10[m
 target_y = 0[m
 target_z = 15[m
 [m
[32m+[m[32m# Calculate joint angles[m
 q1, q2, q3 = inverse_kinematics([m
     target_x, target_y, target_z[m
 )[m
 [m
[32m+[m[32m# Calculate joint positions[m
 p0, p1, p2 = forward_kinematics(q1, q2, q3)[m
 [m
[31m-# Plot 3D robot[m
[31m-fig = plt.figure(figsize=(9, 7))[m
[32m+[m[32m# Calculate position error[m
[32m+[m[32mtarget = (target_x, target_y, target_z)[m
[32m+[m[32mposition_error = math.dist(p2, target)[m
[32m+[m
[32m+[m[32m# Create 3D figure[m
[32m+[m[32mfig = plt.figure(figsize=(11, 8))[m
 ax = fig.add_subplot(111, projection="3d")[m
 [m
[31m-# Base[m
[31m-ax.scatter(0, 0, 0, color="black", s=100, label="Base")[m
[32m+[m[32m# Draw cylindrical base and vertical support[m
[32m+[m[32mdraw_base(ax)[m
[32m+[m
[32m+[m[32max.plot([m
[32m+[m[32m    [0, p0[0]],[m
[32m+[m[32m    [0, p0[1]],[m
[32m+[m[32m    [0, p0[2]],[m
[32m+[m[32m    color="black",[m
[32m+[m[32m    linewidth=7,[m
[32m+[m[32m    label="Base support"[m
[32m+[m[32m)[m
[32m+[m
[32m+[m[32m# Draw first arm link[m
[32m+[m[32max.plot([m
[32m+[m[32m    [p0[0], p1[0]],[m
[32m+[m[32m    [p0[1], p1[1]],[m
[32m+[m[32m    [p0[2], p1[2]],[m
[32m+[m[32m    color="royalblue",[m
[32m+[m[32m    linewidth=8,[m
[32m+[m[32m    marker="o",[m
[32m+[m[32m    markersize=11,[m
[32m+[m[32m    label="Shoulder link"[m
[32m+[m[32m)[m
 [m
[31m-# Arm links[m
[32m+[m[32m# Draw second arm link[m
 ax.plot([m
[31m-    [p0[0], p1[0], p2[0]],[m
[31m-    [p0[1], p1[1], p2[1]],[m
[31m-    [p0[2], p1[2], p2[2]],[m
[32m+[m[32m    [p1[0], p2[0]],[m
[32m+[m[32m    [p1[1], p2[1]],[m
[32m+[m[32m    [p1[2], p2[2]],[m
[32m+[m[32m    color="darkorange",[m
[32m+[m[32m    linewidth=8,[m
     marker="o",[m
[31m-    linewidth=6,[m
[31m-    markersize=10,[m
[31m-    label="Robot arm"[m
[32m+[m[32m    markersize=11,[m
[32m+[m[32m    label="Elbow link"[m
 )[m
 [m
[31m-# Target marker[m
[32m+[m[32m# Highlight joints[m
[32m+[m[32mfor point, name in zip([m
[32m+[m[32m    [p0, p1, p2],[m
[32m+[m[32m    ["Shoulder", "Elbow", "End-effector"][m
[32m+[m[32m):[m
[32m+[m[32m    ax.scatter(*point, color="red", s=75)[m
[32m+[m[32m    ax.text([m
[32m+[m[32m        point[0], point[1], point[2] + 1,[m
[32m+[m[32m        name,[m
[32m+[m[32m        fontsize=9[m
[32m+[m[32m    )[m
[32m+[m
[32m+[m[32m# Target position[m
 ax.scatter([m
     target_x, target_y, target_z,[m
[31m-    color="red", marker="*", s=180,[m
[32m+[m[32m    color="green",[m
[32m+[m[32m    marker="*",[m
[32m+[m[32m    s=220,[m
     label="Target position"[m
 )[m
 [m
[32m+[m[32m# Dashed line between actual and target positions[m
[32m+[m[32max.plot([m
[32m+[m[32m    [p2[0], target_x],[m
[32m+[m[32m    [p2[1], target_y],[m
[32m+[m[32m    [p2[2], target_z],[m
[32m+[m[32m    linestyle="--",[m
[32m+[m[32m    color="green",[m
[32m+[m[32m    label="Position error"[m
[32m+[m[32m)[m
[32m+[m
[32m+[m[32m# Graph settings[m
 ax.set_xlabel("X (cm)")[m
 ax.set_ylabel("Y (cm)")[m
 ax.set_zlabel("Z (cm)")[m
[31m-ax.set_title("3-DOF Robotic Manipulator")[m
[32m+[m
[32m+[m[32max.set_title([m
[32m+[m[32m    "3-DOF Robotic Manipulator - 3D Visualization",[m
[32m+[m[32m    fontsize=13[m
[32m+[m[32m)[m
[32m+[m
 ax.set_xlim(-22, 22)[m
 ax.set_ylim(-22, 22)[m
 ax.set_zlim(0, 25)[m
[32m+[m
[32m+[m[32max.set_box_aspect((44, 44, 25))[m
[32m+[m[32max.view_init(elev=25, azim=45)[m
[32m+[m
 ax.legend()[m
[32m+[m[32max.grid(True)[m
 [m
[31m-print("3-DOF Robotic Manipulator")[m
[32m+[m[32m# Display robot information[m
[32m+[m[32mprint("\n3-DOF Robotic Manipulator")[m
 print("-------------------------")[m
 print("Base angle:", round(math.degrees(q1), 2))[m
 print("Shoulder angle:", round(math.degrees(q2), 2))[m
 print("Elbow angle:", round(math.degrees(q3), 2))[m
 print("End-effector:", p2)[m
[31m-print("Target:", (target_x, target_y, target_z))[m
[31m-print("Position error:", round([m
[31m-    math.dist(p2, (target_x, target_y, target_z)), 6[m
[31m-), "cm")[m
[32m+[m[32mprint("Target:", target)[m
[32m+[m[32mprint("Position error:", round(position_error, 6), "cm")[m
 [m
[32m+[m[32mplt.tight_layout()[m
 plt.show()[m
