#!/usr/bin/env python3
"""ELFの固定FW version sectionへbuild IDを書き込み、Git情報をビルドログへ保存する。"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import pathlib
import struct
import subprocess
import tempfile
import time

MAGIC = 0x52565746


def git_output(repo: pathlib.Path, *args: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(repo), *args], text=True, encoding="utf-8"
    ).strip()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("elf", nargs="?", type=pathlib.Path)
    parser.add_argument("--objcopy")
    parser.add_argument("--log-only", action="store_true")
    parser.add_argument("--repo", required=True, type=pathlib.Path)
    parser.add_argument("--target", required=True)
    parser.add_argument("--log-dir", required=True, type=pathlib.Path)
    parser.add_argument("--build-id", type=int)
    args = parser.parse_args()

    if not args.log_only and (args.elf is None or args.objcopy is None):
        parser.error("elf and --objcopy are required unless --log-only is used")
    build_id = args.build_id if args.build_id is not None else int(time.time())
    if not 1 <= build_id <= 0xFFFFFFFF:
        raise ValueError("build ID is outside uint32 range")

    git_hash = git_output(args.repo, "rev-parse", "HEAD")
    git_dirty = bool(git_output(args.repo, "status", "--porcelain"))
    utc = dt.datetime.now(dt.timezone.utc)
    args.log_dir.mkdir(parents=True, exist_ok=True)

    if not args.log_only:
        descriptor = struct.pack("<II", MAGIC, build_id)
        with tempfile.TemporaryDirectory(prefix="fw_version_") as temp_dir:
            descriptor_path = pathlib.Path(temp_dir) / "fw_version.bin"
            output_elf = pathlib.Path(temp_dir) / args.elf.name
            descriptor_path.write_bytes(descriptor)
            subprocess.run(
                [
                    args.objcopy,
                    "--update-section",
                    f".fw_version={descriptor_path}",
                    str(args.elf),
                    str(output_elf),
                ],
                check=True,
            )
            os.replace(output_elf, args.elf)

    record = {
        "target": args.target,
        "build_id": build_id,
        "build_utc": utc.isoformat(),
        "git_hash": git_hash,
        "git_dirty": git_dirty,
    }
    stamp = utc.strftime("%Y%m%dT%H%M%SZ")
    log_path = args.log_dir / f"build_{stamp}_{args.target}.json"
    log_text = json.dumps(record, ensure_ascii=False, indent=2) + "\n"
    log_path.write_text(log_text, encoding="utf-8")
    (args.log_dir / f"latest_{args.target}.json").write_text(log_text, encoding="utf-8")
    print(
        f"FW_BUILD_ID={build_id} GIT_HASH={git_hash} "
        f"GIT_DIRTY={int(git_dirty)} LOG={log_path}"
    )


if __name__ == "__main__":
    main()
