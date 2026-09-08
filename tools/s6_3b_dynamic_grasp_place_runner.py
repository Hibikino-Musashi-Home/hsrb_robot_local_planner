#!/usr/bin/env python3
"""Run S6.3b: BridgeA online obstacle tracking during grasp and placement."""

from __future__ import annotations

import sys

from s5_recognition_grasp_runner import main as _run_s5_sequence


def main() -> int:
    arguments = list(sys.argv[1:])
    required = (
        ("--detect-table", None),
        ("--place-on-table", None),
        ("--dynamic-bridge", None),
        ("--table-observation-pan", "0.80"),
        ("--table-patch-size", "0.24"),
        ("--place-clearance", "0.015"),
        ("--place-high-offset", "0.0"),
        ("--place-offset-y", "-0.08"),
    )
    for option, value in required:
        if option in arguments:
            continue
        arguments.append(option)
        if value is not None:
            arguments.append(value)
    if "--object-id" not in arguments:
        arguments.extend(["--object-id", "s6_3b_dynamic_object"])
    return _run_s5_sequence(arguments)


if __name__ == "__main__":
    raise SystemExit(main())
