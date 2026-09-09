#!/usr/bin/env python3
"""Run the S6.3a cabinet-style tabletop placement regression.

The scene contains a real static overhead shelf above the placement table.
The runner aligns the detected tabletop patch and places the object directly
under the shelf.  After grasping from above, it rotates the hand at the shelf
front and inserts the attached object horizontally from the open side.  It
also verifies that a target penetrating the shelf is rejected and that the
actual placement completes without shelf contact.
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
        ("--cabinet-place-under-shelf", None),
        ("--cabinet-side-insertion", None),
        ("--cabinet-side-place-lift", "0.045"),
        ("--table-observation-pan", "0.80"),
        ("--table-patch-size", "0.24"),
        ("--place-clearance", "0.015"),
        ("--place-high-offset", "0.0"),
        ("--final-linear-distance", "0.2"),
    )
    for option, value in required:
        if option in arguments:
            continue
        arguments.append(option)
        if value is not None:
            arguments.append(value)
    if "--final-linear-axis" not in arguments:
        arguments.extend(["--final-linear-axis", "0.0", "0.0", "-1.0"])
    if "--object-id" not in arguments:
        arguments.extend(["--object-id", "s6_3a_cabinet_object"])
    return _run_s5_sequence(arguments)


if __name__ == "__main__":
    raise SystemExit(main())
