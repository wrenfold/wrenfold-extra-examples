"""Generate all the examples."""

import subprocess
import sys
from pathlib import Path

CURRENT_DIR = Path(__file__).parent.absolute()


def main():
    scripts = [
        "examples.ceres_simple_bundle_adjuster.gen_snavely_reprojection_error",
        "examples.gtsam_bal.gen_bundle_adjustment_factor",
        "examples.sophus_pose3_interpolation.gen_pose3_interpolation",
    ]
    for script in scripts:
        subprocess.check_call([sys.executable, "-m", script], cwd=CURRENT_DIR.parent)


if __name__ == "__main__":
    main()
