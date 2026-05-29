#!/usr/bin/env python3
"""
init_lidar.py — Initialize the Livox Mid-360 LiDAR via SDK2.
Must run on the robot (192.168.123.164) to start LiDAR streaming.
The LiDAR streams UDP multicast to 224.1.1.5:56301.
Keep this process alive — killing it stops the LiDAR.
"""

import ctypes
import time
import signal
import sys
import os

# Path to the Livox SDK2 shared library on the G1
LIB_PATH = "/usr/local/lib/liblivox_lidar_sdk_shared.so"
# Config JSON — must be in the same directory or absolute path
CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "MID360_config.json")

def main():
    print("Loading Livox SDK2 from %s ..." % LIB_PATH, flush=True)
    print("Config: %s" % CONFIG_PATH, flush=True)
    sdk = ctypes.CDLL(LIB_PATH)

    # bool LivoxLidarSdkInit(const char *path, const char *host_point_ipaddr, const LivoxLidarLoggerCfgInfo* log_cfg_info)
    sdk.LivoxLidarSdkInit.restype = ctypes.c_bool
    sdk.LivoxLidarSdkInit.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_void_p]

    ret = sdk.LivoxLidarSdkInit(CONFIG_PATH.encode(), None, None)
    print("LivoxLidarSdkInit returned: %s" % ret, flush=True)

    if not ret:
        print("ERROR: SDK init failed!", flush=True)
        sys.exit(1)

    print("LiDAR initialized successfully. Streaming to 224.1.1.5:56301", flush=True)
    print("Keep this process alive. Ctrl+C to stop.", flush=True)

    # Keep alive — the SDK runs in background threads
    def handler(sig, frame):
        print("\nShutting down LiDAR SDK...", flush=True)
        try:
            sdk.LivoxLidarSdkUninit()
        except Exception:
            pass
        sys.exit(0)
        

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    while True:
        time.sleep(1.0)


if __name__ == '__main__':
    main()
