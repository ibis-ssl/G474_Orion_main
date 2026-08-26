"""G474 Slot Aバイナリを検査し、M1ブートローダー用CRC32C metadataを生成する。"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

from orion_crc32c import crc32c


SLOT_BASES = {"a": 0x08008000, "b": 0x08040000}
SLOT_SIZE = 0x00038000
SRAM1_BASE = 0x20000000
SRAM1_END = 0x20020000
CCMRAM_BASE = 0x10000000
CCMRAM_END = 0x10008000
METADATA_MAGIC = 0x3157464F
METADATA_FORMAT = 1
METADATA_STATE_CONFIRMED = 4
METADATA_SLOTS = {"a": 0, "b": 1}
METADATA_RECORD_SIZE = 36
def validate_vector(image: bytes, slot: str) -> tuple[int, int]:
    slot_base = SLOT_BASES[slot]
    if len(image) < 8:
        raise ValueError("image is too short to contain a vector table")
    if len(image) > SLOT_SIZE:
        raise ValueError(f"image size {len(image)} exceeds slot size {SLOT_SIZE}")

    stack_pointer, reset_handler = struct.unpack_from("<II", image)
    stack_in_range = SRAM1_BASE <= stack_pointer <= SRAM1_END or CCMRAM_BASE <= stack_pointer <= CCMRAM_END
    if not stack_in_range or stack_pointer % 8 != 0:
        raise ValueError(f"invalid initial stack pointer: 0x{stack_pointer:08X}")

    handler_address = reset_handler & ~1
    if reset_handler & 1 == 0 or not slot_base <= handler_address < slot_base + len(image):
        raise ValueError(f"invalid Slot {slot.upper()} reset handler: 0x{reset_handler:08X}")
    return stack_pointer, reset_handler


def build_metadata(image: bytes, generation: int, slot: str) -> bytes:
    image_crc = crc32c(image)
    record_without_crc = struct.pack(
        "<IHHIIIIII",
        METADATA_MAGIC,
        METADATA_FORMAT,
        METADATA_RECORD_SIZE,
        generation,
        METADATA_STATE_CONFIRMED,
        METADATA_SLOTS[slot],
        SLOT_BASES[slot],
        len(image),
        image_crc,
    )
    if len(record_without_crc) != METADATA_RECORD_SIZE - 4:
        raise AssertionError("metadata packing does not match the C structure")
    return record_without_crc + struct.pack("<I", crc32c(record_without_crc))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--generation", type=int, default=1)
    parser.add_argument("--slot", choices=("a", "b"), default="a")
    args = parser.parse_args()

    image = args.image.read_bytes()
    stack_pointer, reset_handler = validate_vector(image, args.slot)
    metadata = build_metadata(image, args.generation, args.slot)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(metadata)

    image_crc = crc32c(image)
    record_crc = struct.unpack_from("<I", metadata, METADATA_RECORD_SIZE - 4)[0]
    print(f"image={args.image}")
    print(f"size={len(image)}")
    print(f"initial_sp=0x{stack_pointer:08X}")
    print(f"reset_handler=0x{reset_handler:08X}")
    print(f"image_crc32c=0x{image_crc:08X}")
    print(f"metadata_crc32c=0x{record_crc:08X}")
    print(f"metadata={args.output}")


if __name__ == "__main__":
    main()
