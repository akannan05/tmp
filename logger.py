import board
import busio
import json
from datetime import datetime, timedelta
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE_CALIBRATED,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_GRAVITY,
    BNO_REPORT_EULER,
    BNO_REPORT_TEMPERATURE
)

# ----- Configuration -----
REPORT_INTERVAL_US = 2000  # 500 Hz -> 2000 us
JSON_FLUSH_INTERVAL = 5 * 60  # 5 minutes
DURATION_SEC = None  # None for indefinite run

# ----- Setup I2C + BNO08X -----
i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c)

# Enable features
bno.enable_feature(BNO_REPORT_ACCELEROMETER, REPORT_INTERVAL_US)
bno.enable_feature(BNO_REPORT_GYROSCOPE_CALIBRATED, REPORT_INTERVAL_US)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, REPORT_INTERVAL_US)
bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION, REPORT_INTERVAL_US)
bno.enable_feature(BNO_REPORT_GRAVITY, REPORT_INTERVAL_US)
bno.enable_feature(BNO_REPORT_EULER, REPORT_INTERVAL_US)
bno.enable_feature(BNO_REPORT_TEMPERATURE, REPORT_INTERVAL_US)

# ----- JSON buffer -----
buffer = []

start_time = datetime.now()
last_flush_time = start_time
json_filename = start_time.strftime("%Y%m%d_%H%M%S") + ".json"

def flush_buffer():
    global buffer, json_filename, last_flush_time
    if buffer:
        with open(json_filename, "a") as f:
            for sample in buffer:
                f.write(json.dumps(sample) + "\n")
        buffer = []
        last_flush_time = datetime.now()
        print(f"[INFO] Flushed JSON buffer at {last_flush_time}, file: {json_filename}")

# ----- Main logging loop -----
print("[INFO] Starting BNO08X logging...")
while True:
    now = datetime.now()
    if DURATION_SEC is not None and now - start_time >= timedelta(seconds=DURATION_SEC):
        break

    sample = {
        "host_ts_ns": int(now.timestamp() * 1e9)
    }

    # Read all enabled features
    try:
        accel = bno.acceleration
        if accel is not None:
            sample["accel"] = {"x": accel[0], "y": accel[1], "z": accel[2]}

        gyro = bno.gyroscope
        if gyro is not None:
            sample["gyro"] = {"x": gyro[0], "y": gyro[1], "z": gyro[2]}

        quat = bno.quaternion
        if quat is not None:
            sample["quat"] = {"w": quat[0], "x": quat[1], "y": quat[2], "z": quat[3]}

        lin_accel = bno.linear_acceleration
        if lin_accel is not None:
            sample["linear_accel"] = {"x": lin_accel[0], "y": lin_accel[1], "z": lin_accel[2]}

        gravity = bno.gravity
        if gravity is not None:
            sample["gravity"] = {"x": gravity[0], "y": gravity[1], "z": gravity[2]}

        euler = bno.euler
        if euler is not None:
            sample["euler"] = {"yaw": euler[0], "pitch": euler[1], "roll": euler[2]}

        temp = bno.temperature
        if temp is not None:
            sample["temp_c"] = temp

        buffer.append(sample)

    except Exception as e:
        print(f"[ERROR] Sensor read failed: {e}")

    # Flush buffer every 5 minutes
    if now - last_flush_time >= timedelta(seconds=JSON_FLUSH_INTERVAL):
        flush_buffer()
