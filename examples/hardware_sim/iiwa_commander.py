"""A simple program to actuate an IIWA arm with a cyclic motion."""

import argparse
import math
import time

import numpy as np

from drake import lcmt_iiwa_command
from pydrake.lcm import DrakeLcm

# The maximum deflection from the initial joint position. I.e., each joint will
# move between +/-MAX_DEFLECTION. The number is selected to ensure there is no
# collision given the pre-defined arm initial pose.
MAX_DEFLECTION_RADIANS = 0.5

# The initial positions (in radians). This needs to be in sync with the
# scenario file.
INITIAL_POSITION = np.array([-0.2, 0.79, 0.32, -1.76, -0.36, 0.64, -0.73])

# Use a non-default LCM url to send iiwa commands.
lcm = DrakeLcm("udpm://239.255.2.1:6776")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
    )
    parser.add_argument(
        "--iiwa_command_hz",
        type=int,
        required=False,
        default=10,
        help="The rate to send an Iiwa command.",
    )
    parser.add_argument(
        "--cycle_time",
        type=int,
        required=False,
        default=10,
        help="How much time in seconds for a motion cycle.",
    )
    parser.add_argument(
        "--unit_test",
        action="store_true",
        help="Whether to run only once for smoke testing.",
    )
    args = parser.parse_args()

    iiwa_command = lcmt_iiwa_command()
    iiwa_command.num_joints = 7
    while True:
        if args.unit_test:
            # Only send command once for testing.
            partition = 1
        else:
            # The number of increments for the robot motion.
            partition = args.cycle_time * args.iiwa_command_hz

        for i in range(partition):
            delta = (
                math.sin(2 * math.pi / partition * i) * MAX_DEFLECTION_RADIANS
            )
            iiwa_command.joint_position = INITIAL_POSITION + delta
            lcm.Publish(channel="IIWA_COMMAND", buffer=iiwa_command.encode())
            time.sleep(1 / args.iiwa_command_hz)

        if args.unit_test:
            break


if __name__ == "__main__":
    main()
