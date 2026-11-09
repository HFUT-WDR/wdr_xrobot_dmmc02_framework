#!/usr/bin/env python3
"""测试工具使用Python准备错帧与黄金帧，运行时代码全部为C++。"""
from pathlib import Path
import struct
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]


def crc8(data):
    value = 255
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8c if value & 1 else 0)
    return value


def crc32(data):
    value = 0xffffffff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0xedb88320 if value & 1 else 0)
    return value


def frame(payload):
    header = (b'\x5a' + len(payload).to_bytes(3, 'little')
              + crc32(b'host_aim').to_bytes(4, 'little')
              + (7654321).to_bytes(6, 'little') + b'\x01')
    packet = header + bytes([crc8(header)]) + payload
    return packet + bytes([crc8(packet)])


with tempfile.TemporaryDirectory(prefix='wdr-host-test-') as directory:
    stage = Path(directory)
    subprocess.run(['cmake', '-S', str(root/'tests/host'), '-B', str(stage/'build'),
                    '-DCMAKE_BUILD_TYPE=Debug'], check=True)
    subprocess.run(['cmake', '--build', str(stage/'build'), '-j4'], check=True)
    payload = struct.pack('<Q8f5B', 1234567, 1.25, 2, 3, -.25, 4, 5, 6, 7,
                          2, 1, 1, 3, 4)
    golden = frame(payload)
    corrupted = bytearray(golden)
    corrupted[-1] ^= 1
    source, output = stage/'input.bin', stage/'output.bin'
    source.write_bytes(b'junk'+golden+frame(payload[:-1])+frame(payload+b'\x00')
                       + corrupted + golden)
    subprocess.run([str(stage/'build/host_tests'), str(source), str(output)], check=True)
    assert output.read_bytes() == golden
print('PASS: real LibXR bidirectional golden frame, malformed lengths, CRC, independent aim/nav timeout')
