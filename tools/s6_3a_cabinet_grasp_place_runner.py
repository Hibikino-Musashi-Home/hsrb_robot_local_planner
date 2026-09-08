#!/usr/bin/env python3
"""Run the S6.3a cabinet-style tabletop placement regression.

The scene contains a real static overhead shelf above the placement table.
The runner keeps the safe placement point in the open front part, verifies
that an attached-object target inside the shelf is rejected, and then places
the object without contacting the shelf.
"""

from __future__ import annotations

import sys

from s5_recognition_grasp_runner import main as _run_s5_sequence


def main() -> int:
    arguments = list(sys.argv[1:])
    required = (
        ("--detect-table", None),
        ("--place-on-table", None),
        ("--cabinet-overhead", None),
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
        arguments.extend(["--object-id", "s6_3a_cabinet_object"])
    return _run_s5_sequence(arguments)


if __name__ == "__main__":
    raise SystemExit(main())
