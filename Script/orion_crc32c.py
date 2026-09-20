"""Orion FW成果物とFlash backupに共通のCRC32Cを計算する。"""

from __future__ import annotations

import argparse
from pathlib import Path


CRC32C_POLYNOMIAL = 0x82F63B78


def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            mask = -(crc & 1) & 0xFFFFFFFF
            crc = ((crc >> 1) ^ (CRC32C_POLYNOMIAL & mask)) & 0xFFFFFFFF
    return crc ^ 0xFFFFFFFF


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("file", type=Path)
    args = parser.parse_args()
    print(f"0x{crc32c(args.file.read_bytes()):08X}")


if __name__ == "__main__":
    main()

