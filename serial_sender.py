
import serial
import json
import time

# Replace COM5 with your virtual serial port
PORT = "COM5"
BAUD_RATE = 9600

try:
    robot = serial.Serial(PORT, BAUD_RATE, timeout=1)
    time.sleep(2)

    print("Connected to robot controller!")

    while True:
        base = int(input("Enter Base angle (0-180): "))
        shoulder = int(input("Enter Shoulder angle (0-180): "))
        elbow = int(input("Enter Elbow angle (0-180): "))

        # Safety limits
        angles = [base, shoulder, elbow]

        if any(a < 0 or a > 180 for a in angles):
            print("Angles must be between 0 and 180.")
            continue

        command = {
            "base": base,
            "shoulder": shoulder,
            "elbow": elbow
        }

        message = json.dumps(command) + "\n"
        robot.write(message.encode("utf-8"))

        print("Command sent:", message.strip())

        response = robot.readline().decode(
            "utf-8", errors="replace"
        ).strip()

        if response:
            print("Arduino response:", response)

except serial.SerialException as e:
    print("Serial connection error:", e)

except KeyboardInterrupt:
    print("\nProgram stopped.")

finally:
    if "robot" in locals() and robot.is_open:
        robot.close()
