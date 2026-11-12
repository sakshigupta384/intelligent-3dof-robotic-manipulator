import math

# Robot dimensions in centimetres
H = 5
L1 = 10
L2 = 10

def inverse_kinematics(x, y, z):
    # Base rotation
    base = math.degrees(math.atan2(y, x))

    # Convert target to 2D coordinates
    r = math.sqrt(x**2 + y**2)
    vertical = z - H

    # Calculate elbow angle
    D = (r**2 + vertical**2 - L1**2 - L2**2) / (2 * L1 * L2)

    if D < -1 or D > 1:
        print("Target is outside robot workspace!")
        return

    elbow = math.acos(D)

    # Calculate shoulder angle
    shoulder = math.atan2(vertical, r) - math.atan2(
        L2 * math.sin(elbow),
        L1 + L2 * math.cos(elbow)
    )

    return base, math.degrees(shoulder), math.degrees(elbow)


# Test target
result = inverse_kinematics(10, 0, 15)

if result:
    print("Inverse Kinematics Results")
    print("--------------------------")
    print("Base angle:", round(result[0], 2))
    print("Shoulder angle:", round(result[1], 2))
    print("Elbow angle:", round(result[2], 2))