"""A main() program (plus an reusable standalone function) that simulates a
spong-controlled acrobot.
"""

import argparse
import sys

from pydrake.systems.primitives import LogOutput
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.examples.acrobot import (
    AcrobotPlant, AcrobotSpongController, AcrobotState)
from drake.examples.acrobot.dev.acrobot_io import load_scenario, save_output


# TODO(#13494) This file is a placeholder (just holding down its dependencies)
# until it can be brought up to full stochastic support.

if __name__ == "__main__":
    sys.exit(main(1))
