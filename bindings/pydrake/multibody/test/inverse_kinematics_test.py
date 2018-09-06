from pydrake.multibody.multibody_tree.multibody_plant import (
    MultibodyPlant,
)
from pydrake.multibody import inverse_kinematics as ik
from pydrake.multibody.multibody_tree.parsing import (
    AddModelFromSdfFile,
)
from pydrake.multibody.benchmarks.acrobot import (
    MakeAcrobotPlant,
)

import unittest

from pydrake.common import FindResourceOrThrow
class TestInverseKinematics(unittest.TestCase):
    def setUp(self):
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        self.plant = MultibodyPlant(time_step=0.01)
        model_instance = AddModelFromSdfFile(
            file_name=file_name, plant=self.plant, scene_graph=None)
        self.plant.Finalize()
        self.link1_frame = self.plant.GetBodyByName("Link1").body_frame()
        self.link2_frame = self.plant.GetBodyByName("Link2").body_frame()

    def test_AddPositionConstraint(self):
        ik_two_bodies = ik.InverseKinematics(self.plant)
        ik_two_bodies.AddPositionConstraint(self.link1_frame, [0, 0.1, 0.2], self.link2_frame, [-1, -1, -1], [1, 1, 1])

