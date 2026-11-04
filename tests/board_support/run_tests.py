#!/usr/bin/env python3
"""只验证内存零偏存储契约，不替代 SPI、IMU 或 LED 板测。"""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[2]
libxr = root / "Middlewares/Third_Party/LibXR"
with tempfile.TemporaryDirectory(prefix="wdr-board-test-") as directory:
    executable = pathlib.Path(directory) / "ram_database_test"
    command = ["g++", "-std=c++20", "-Wall", "-Wextra", "-Werror",
               "-fsanitize=address,undefined", "-static-libasan", "-g",
               "-I" + str(root / "User"),
               "-I" + str(libxr / "src/core"),
               "-I" + str(libxr / "src/middleware"),
               str(root / "tests/board_support/ram_database_test.cpp"),
               "-o", str(executable)]
    subprocess.run(command, check=True)
    subprocess.run([str(executable)], check=True)
