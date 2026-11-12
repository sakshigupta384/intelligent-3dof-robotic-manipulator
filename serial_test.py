
import json

# Example joint commands from inverse kinematics
robot_data = {
    "base": 0,
    "shoulder": 0,
    "elbow": 90
}

# Convert data into a JSON message
message = json.dumps(robot_data)

print("Robot command generated:")
print(message)

print("\nSerial communication format prepared!")
