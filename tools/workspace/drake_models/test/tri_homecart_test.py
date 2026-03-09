import unittest

from pydrake.planning import RobotDiagramBuilder


class TestTriHomecart(unittest.TestCase):
    def test(self):
        builder = RobotDiagramBuilder()
        builder.parser().AddModels(
            url="package://drake_models/tri_homecart/homecart.dmd.yaml"
        )
        diagram = builder.Build()

        # 6 positions per ur3e and 2 per wsg.
        self.assertEqual(diagram.plant().num_positions(), 6 + 2 + 6 + 2)
