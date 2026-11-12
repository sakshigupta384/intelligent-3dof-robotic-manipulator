
import math

# Robot dimensions in centimetres
H = 5
L1 = 10
L2 = 10


def inverse_kinematics(x, y, z, elbow_up=False):
    """
    Calculate joint angles for a 3-DOF robotic manipulator.

    Parameters:
        x, y, z: Target coordinates in centimetres.
        elbow_up: Select elbow-up configuration if True.

    Returns:
        Dictionary containing joint angles, or None if unreachable.
    """

    # Validate input coordinates
    coordinates = (x, y, z)

    if not all(isinstance(value, (int, float)) and math.isfinite(value)
               for value in coordinates):
        raise ValueError("Target coordinates must be finite numbers.")

    # Base rotation
    base = math.degrees(math.atan2(y, x))

    # Convert target into 2D coordinates
    r = math.sqrt(x**2 + y**2)
    vertical = z - H

    # Calculate elbow cosine
    D = (
        r**2 + vertical**2 - L1**2 - L2**2
    ) / (2 * L1 * L2)

    # Check workspace limits
    if D < -1 or D > 1:
        return None

    # Avoid floating-point errors
    D = max(-1.0, min(1.0, D))

    # Calculate elbow angle
    elbow = math.acos(D)

    if elbow_up:
        elbow = -elbow

    # Calculate shoulder angle
    shoulder = math.atan2(vertical, r) - math.atan2(
        L2 * math.sin(elbow),
        L1 + L2 * math.cos(elbow)
    )

    return {
        "base": round(base, 2),
        "shoulder": round(math.degrees(shoulder), 2),
        "elbow": round(math.degrees(elbow), 2),
        "reachable": True
    }


# Test the inverse kinematics function
if __name__ == "__main__":

    result = inverse_kinematics(10, 0, 15)

    print("\nInverse Kinematics Results")
    print("--------------------------")

    if result:
        for joint, angle in result.items():
            print(f"{joint.capitalize()}: {angle}")
    else:
        print("Target is outside robot workspace!")
