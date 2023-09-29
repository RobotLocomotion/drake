"""A simple program to actuate the robot, an IIWA arm and a WSG gripper, with a
cyclic motion."""

import argparse
import math
import sys
import time

import numpy as np

from drake import lcmt_iiwa_command, lcmt_schunk_wsg_command
from pydrake.lcm import DrakeLcm

# The maximum deflection from IIWA and WSG's initial position. I.e., each joint
# will move between +/-MAX_DEFLECTION. The number is selected to ensure there
# is no collision given the pre-defined arm initial pose.
MAX_IIWA_DEFLECTION_RADIANS = 0.5
MAX_WSG_DEFLECTION_MM = 20

# The initial positions of the robot. This needs to be in sync with the
# scenario file.
IIWA_INITIAL_POSITION = np.array([-0.2, 0.79, 0.32, -1.76, -0.36, 0.64, -0.73])
WSG_INITIAL_POSITION = 20

# Use a non-default LCM url to communicate with the robot.
COMMANDER_LCM = DrakeLcm("udpm://239.255.2.1:6776?ttl=0")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
    )
    parser.add_argument(
        "--robot_command_hz",
        type=float,
        required=False,
        default=20.0,
        help="The rate to send the robot command.",
    )
    parser.add_argument(
        "--cycle_time",
        type=float,
        required=False,
        default=10.0,
        help="How much time in seconds for a motion cycle.",
    )
    parser.add_argument(
        "--unit_test",
        action="store_true",
        help="Whether to run only once for smoke testing.",
    )
    args = parser.parse_args()

    wsg_command = lcmt_schunk_wsg_command()
    wsg_command.force = 20.0
    iiwa_command = lcmt_iiwa_command()
    iiwa_command.num_joints = 7

    counter = 0
    joint_increment = 2 * math.pi / (args.cycle_time * args.robot_command_hz)
    while True:
        iiwa_delta = (
            math.sin(joint_increment * counter) * MAX_IIWA_DEFLECTION_RADIANS
        )
        wsg_delta = math.sin(joint_increment * counter) * MAX_WSG_DEFLECTION_MM
        iiwa_command.joint_position = IIWA_INITIAL_POSITION + iiwa_delta
        wsg_command.target_position_mm = WSG_INITIAL_POSITION + wsg_delta
        COMMANDER_LCM.Publish(
            channel="IIWA_COMMAND", buffer=iiwa_command.encode()
        )
        COMMANDER_LCM.Publish(
            channel="SCHUNK_WSG_COMMAND", buffer=wsg_command.encode()
        )

        # Only send the command once for unit testing.
        if args.unit_test:
            sys.exit(0)

        counter += 1
        time.sleep(1 / args.robot_command_hz)


if __name__ == "__main__":
    main()
