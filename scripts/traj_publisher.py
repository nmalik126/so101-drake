import pickle
from pathlib import Path
import numpy as np
import time
import lcm

from pydrake.all import (
    BsplineTrajectory
)

import os
import sys
module_path = os.path.abspath(os.path.join('..'))
if module_path not in sys.path:
    sys.path.append(module_path)
from lcmdefs.messages.so101 import lcmt_so101_configuration


def main():
    msg = lcmt_so101_configuration()
    lc = lcm.LCM()

    project_dir = Path("/home/noor/so101-drake")

    with open(project_dir / "assets" / "example_full_traj.pkl", "rb") as f:
        trajectory: BsplineTrajectory = pickle.load(f)

    time_step = 1.0 / 200
    for t in np.append(
        np.arange(trajectory.start_time(), trajectory.end_time(), time_step),
        trajectory.end_time(),
    ):
        # print(trajectory.value(t))
        msg.q = trajectory.value(t)
        lc.publish("SO101_COMMAND", msg.encode())

        time.sleep(time_step)

if __name__ == "__main__":
    main()
