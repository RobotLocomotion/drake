from pydrake.multibody import inverse_kinematics as ik
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant 
from pydrake.multibody.multibody_tree.parsing import (
    AddModelFromSdfFile,
)
from pydrake.multibody.benchmarks.acrobot import (
    MakeAcrobotPlant,
)
import unittest
import pydrake.solvers.mathematicalprogram as mp

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
        ik_acrobot = ik.InverseKinematics(self.plant)
        ik_acrobot.AddPositionConstraint(self.link1_frame, [0, 0.1, 0.2], self.link2_frame, [-1, -1, -1], [1, 1, 1])
        result = ik_acrobot.get_mutable_prog().Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)

    def test_AddOrientationConstraint(self):
        ik_acrobot = ik.InverseKinematics(self.plant)
        ik_acrobot.AddOrientationConstraint(self.link1_frame, self.link2_frame, 0.5 * math.pi)
        result = ik_acrobot.get_mutable_prog().Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)
