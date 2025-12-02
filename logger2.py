# logger.py
import board
import busio
import json
import sys
import time
from datetime import datetime
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE_CALIBRATED,
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_GRAVITY,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_EULER,
    BNO_REPORT_TEMPERATURE
)

# --- Parse duration ---
duration_sec = 10
if len(sys.argv) > 1:
    duration_sec = int(sys.argv[1])

# --- Initialize I2C and BNO08x ---
i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c)

REPORT_INTERVAL = 10000  # 100 Hz = 10000 us
features = [
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE_CALIBRATED,
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_GRAVITY,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_EULER,
    BNO_REPORT_TEMPERATURE
]

for feature in features:
    bno.enable_feature(feature, REPORT_INTERVAL)

# --- Open JSON file ---
start_time = datetime.now()
filename = start_time.strftime("%Y%m%d_%H%M%S") + ".json"
print(f"Logging to {filename} for {duration_sec} seconds")
data_list = []

# --- Logging loop ---
end_time = time.time() + duration_sec
while time.time() < end_time:
    sample = {"host_ts_ns": int(time.time() * 1e9)}
    
    # Accelerometer
    if hasattr(bno, "acceleration") and bno.acceleration is not None:
        sample["accel"] = {
            "x": bno.acceleration[0],
            "y": bno.acceleration[1],
            "z": bno.acceleration[2],
        }
    
    # Gyroscope
    if hasattr(bno, "gyro") and bno.gyro is not None:
        sample["gyro"] = {
            "x": bno.gyro[0],
            "y": bno.gyro[1],
            "z": bno.gyro[2],
        }

    # Linear acceleration
    if hasattr(bno, "linear_acceleration") and bno.linear_acceleration is not None:
        sample["linear_accel"] = {
            "x": bno.linear_acceleration[0],
            "y": bno.linear_acceleration[1],
            "z": bno.linear_acceleration[2],
        }

    # Gravity
    if hasattr(bno, "gravity") and bno.gravity is not None:
        sample["gravity"] = {
            "x": bno.gravity[0],
            "y": bno.gravity[1],
            "z": bno.gravity[2],
        }

    # Rotation vector / quaternion
    if hasattr(bno, "quaternion") and bno.quaternion is not None:
        sample["quat"] = {
            "w": bno.quaternion[0],
            "x": bno.quaternion[1],
            "y": bno.quaternion[2],
            "z": bno.quaternion[3],
        }

    # Euler angles
    if hasattr(bno, "euler") and bno.euler is not None:
        sample["euler"] = {
            "yaw": bno.euler[0],
            "pitch": bno.euler[1],
            "roll": bno.euler[2],
        }

    # Temperature
    if hasattr(bno, "temperature") and bno.temperature is not None:
        sample["temp_c"] = bno.temperature

    data_list.append(sample)
    time.sleep(REPORT_INTERVAL / 1e6)

# --- Write JSON file ---
with open(filename, "w") as f:
    json.dump(data_list, f, indent=2)

print(f"Logging complete. Wrote {len(data_list)} samples to {filename}")
