#!/usr/bin/env python3
"""Run the S6.3 static table-recognition -> grasp -> place regression.

The motion/attachment sequence is shared with the S5 runner.  This entrypoint
only selects the S6.3 mode, in which GroundingDINO supplies a table bbox and
the organized ``hma_pcl_reconst2`` cloud supplies the tabletop geometry used
for placement.  The Sim table truth topic is an oracle for diagnostics only.
"""

from __future__ import annotations

import sys

from s5_recognition_grasp_runner import main as _run_s5_sequence


def main() -> int:
    arguments = list(sys.argv[1:])
    if "--detect-table" not in arguments:
        arguments.append("--detect-table")
    if "--place-on-table" not in arguments:
        arguments.append("--place-on-table")
    if "--object-id" not in arguments:
        arguments.extend(["--object-id", "s6_3_detected_object"])
    defaults = (
        ("--table-observation-pan", "0.80"),
        ("--table-patch-size", "0.20"),
        ("--place-clearance", "0.015"),
        ("--place-high-offset", "0.0"),
    )
    for option, value in defaults:
        if option not in arguments:
            arguments.extend([option, value])
    return _run_s5_sequence(arguments)


if __name__ == "__main__":
    raise SystemExit(main())
