import math
import time

import numpy as np

from drake import lcmt_iiwa_command
from pydrake.lcm import DrakeLcm


INITIAL_POSITION = np.array([-0.2, 0.79, 0.32, -1.76, -0.36, 0.64, -0.73])

lcm = DrakeLcm()

iiwa_command = lcmt_iiwa_command()
iiwa_command.num_joints = 7

while True:
    max_increment = 0.5
    for i in range(100):
        delta = math.sin(2 * math.pi / 100 * i) * max_increment
        new_position = INITIAL_POSITION + delta
        iiwa_command.joint_position = new_position
        lcm.Publish(channel="IIWA_COMMAND", buffer=iiwa_command.encode())
        time.sleep(0.1)