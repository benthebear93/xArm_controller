"""Automatically capture ten NEW poses and verify an existing calibration.

Default: dry-run, latest successful calibration, ten local poses around the
CURRENT pose, no return, and a new session. Add --execute for actual robot motion.
"""

import sys

from auto_calibrate import main
from robot_common import run_cli


if __name__ == "__main__":
    run_cli(lambda: main(["--verify", *sys.argv[1:]]))
