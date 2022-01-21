from pydrake.multibody.meshcat import (
    JointSliders,
)

import os
import unittest

from pydrake.common import (
    FindResourceOrThrow,
)
from pydrake.geometry import (
    Meshcat,
)
from pydrake.multibody.parsing import (
    Parser,
)
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    MultibodyPlant,
)
from pydrake.systems.framework import (
    DiagramBuilder,
)


class TestMeshcat(unittest.TestCase):

    def test_joint_sliders(self):
        # Load up an acrobot.
        acrobot_file = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.urdf")
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        parser = Parser(plant)
        parser.AddModelFromFile(acrobot_file)
        plant.Finalize()

        # Construct a sliders system, using every available option.
        meshcat = Meshcat()
        dut = JointSliders(
            meshcat=meshcat,
            plant=plant,
            initial_value=[0.1, 0.2],
            lower_limit=[-3.0, -6.0],
            upper_limit=[3.0, 6.0],
            step=[0.25, 0.50],
        )

        # Various methods should not crash.
        dut.get_output_port()
        dut.Delete()

        # The constructor also accepts single values for broadcast (except for
        # the initial value).
        dut = JointSliders(
            meshcat=meshcat,
            plant=plant,
            initial_value=[0.1, 0.2],
            lower_limit=-3.0,
            upper_limit=3.0,
            step=0.1,
        )
        dut.Delete()

        # The constructor also accepts None directly, for optionals.
        dut = JointSliders(
            meshcat=meshcat,
            plant=plant,
            initial_value=None,
            lower_limit=None,
            upper_limit=None,
            step=None,
        )
        dut.Delete()

        # The constructor has default values, in any case.
        dut = JointSliders(meshcat, plant)
