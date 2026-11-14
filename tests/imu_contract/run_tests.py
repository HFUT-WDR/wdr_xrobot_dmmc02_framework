#!/usr/bin/env python3
"""IMU 契约回归入口：校验 Attitude 布局一致且姿态角为 roll,pitch,yaw。"""
import subprocess
import sys
from pathlib import Path

here = Path(__file__).resolve().parent
subprocess.run([sys.executable, str(here / "check_layout.py")], check=True)
