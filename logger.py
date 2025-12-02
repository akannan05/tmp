#!/usr/bin/env python3
import smbus2
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

# ---------- Logging setup ----------
os.makedirs(LOG_DIR, exist_ok=True)
ts = datetime.now().strftime("%Y%m%d_%H%M%S")
log_path = f"{LOG_DIR}/{BASE_FN}_{ts}.ndjson"

def read_sensor():
    # Dummy read: just read first 6 bytes of register 0 (replace with actual SH-2 packets if needed)
    try:
        data = bus.read_i2c_block_data(BNO085_ADDR, 0x00, 6)
        return data
    except Exception as e:
        print("[I2C ERROR]", e)
        return None

with open(log_path, "w") as f:
    start = time.time()
    seq = 0
    while time.time() - start < DURATION_SEC:
        ts_ns = int(time.time() * 1e9)
        data = read_sensor()
        if data:
            sample = {
                "host_ts_ns": ts_ns,
                "seq": seq,
                "raw_bytes": data
            }
            f.write(json.dumps(sample) + "\n")
            seq += 1
        time.sleep(SAMPLE_INTERVAL)

print(f"Logging complete: {log_path}")
