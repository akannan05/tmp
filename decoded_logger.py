#!/usr/bin/env python3
import smbus2
import struct
import time
import json
import os
from datetime import datetime

# ---------- Configuration ----------
I2C_BUS = 1
BNO085_ADDR = 0x4A
LOG_DIR = "logs"
BASE_FN = "bno085"
DURATION_SEC = 10
SAMPLE_INTERVAL = 1 / 500  # 500 Hz

# ---------- Initialize bus ----------
bus = smbus2.SMBus(I2C_BUS)
os.makedirs(LOG_DIR, exist_ok=True)
ts = datetime.now().strftime("%Y%m%d_%H%M%S")
log_path = f"{LOG_DIR}/{BASE_FN}_{ts}.ndjson"

# SH2 sensor IDs (partial)
SH2_ACCELEROMETER = 0x01
SH2_GYROSCOPE_CALIBRATED = 0x02
SH2_MAGNETIC_FIELD_CALIBRATED = 0x03
SH2_ROTATION_VECTOR = 0x05
SH2_LINEAR_ACCELERATION = 0x0A
SH2_GRAVITY = 0x0B
SH2_TEMPERATURE = 0x0C

# ---------- Helpers ----------
def read_packet():
    """Read SHTP packet from BNO085"""
    try:
        # Read first 4 bytes of header (little endian length + channel)
        header = bus.read_i2c_block_data(BNO085_ADDR, 0x00, 4)
        length = header[0] | (header[1] << 8)
        if length < 4:
            return None
        payload_len = length - 4
        payload = bus.read_i2c_block_data(BNO085_ADDR, 0x00, payload_len)
        return bytes(payload)
    except Exception as e:
        print("[I2C ERROR]", e)
        return None

def parse_sensor_payload(payload):
    """Parse sensor event payload (simplified for common sensors)"""
    if len(payload) < 4:
        return None
    sensor_id = payload[0]
    # 2-byte sequence, 4-byte timestamp
    sequence = payload[1]
    timestamp = struct.unpack("<I", payload[2:6])[0]

    sample = {"sensor_id": sensor_id, "sequence": sequence, "timestamp": timestamp}

    # Simplified decoding: accelerometer, gyro, rotation vector
    if sensor_id == SH2_ACCELEROMETER and len(payload) >= 14:
        x, y, z = struct.unpack("<hhh", payload[6:12])
        sample.update({"accel": {"x": x/1000, "y": y/1000, "z": z/1000}})
    elif sensor_id == SH2_GYROSCOPE_CALIBRATED and len(payload) >= 14:
        x, y, z = struct.unpack("<hhh", payload[6:12])
        sample.update({"gyro": {"x": x/1000, "y": y/1000, "z": z/1000}})
    elif sensor_id == SH2_ROTATION_VECTOR and len(payload) >= 16:
        i, j, k, real = struct.unpack("<hhhh", payload[6:14])
        sample.update({"quat": {"x": i/16384, "y": j/16384, "z": k/16384, "w": real/16384}})
    return sample

# ---------- Logging loop ----------
with open(log_path, "w") as f:
    start = time.time()
    seq = 0
    while time.time() - start < DURATION_SEC:
        pkt = read_packet()
        if pkt:
            sample = parse_sensor_payload(pkt)
            if sample:
                sample["host_ts_ns"] = int(time.time()*1e9)
                sample["seq"] = seq
                f.write(json.dumps(sample) + "\n")
                seq += 1
        time.sleep(SAMPLE_INTERVAL)

print(f"Logging complete: {log_path}")
