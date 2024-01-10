import pydrake.planning as mut

import logging
import textwrap
import unittest

import numpy as np


class TestCollisionCheckerTrace(unittest.TestCase):
    """Confirms that trace-level logging from the CollisionChecker's C++ worker
    threads code doesn't deadlock the Python interpreter.
    """

    def setUp(self):
        # Enable all log levels (even C++ "trace" level).
        # logging.getLogger("drake").setLevel(1)
        pass

    def _make_robot_diagram(self):
        builder = mut.RobotDiagramBuilder()
        scene_yaml = textwrap.dedent("""
        directives:
        - add_model:
            name: box
            file: package://drake/multibody/models/box.urdf
        - add_model:
            name: ground
            file: package://drake/planning/test_utilities/collision_ground_plane.sdf  # noqa
        - add_weld:
            parent: world
            child: ground::ground_plane_box
        """)
        builder.parser().AddModelsFromString(scene_yaml, "dmd.yaml")
        model_instance_index = builder.plant().GetModelInstanceByName("box")
        robot_diagram = builder.Build()
        return (robot_diagram, model_instance_index)

    def test_tracing(self):
        # Prepare a checker with a box that can collide with the ground.
        robot, index = self._make_robot_diagram()
        dut = mut.SceneGraphCollisionChecker(
            model=robot,
            robot_model_instances=[index],
            edge_step_size=0.125)

        # Check an edge that has a box-ground collision.
        q1 = np.array([-0.5] * 7)
        q2 = np.array([0.5] * 7)
        with self.assertLogs("drake", level=1) as manager:
            free = dut.CheckEdgeCollisionFreeParallel(q1, q2, parallelize=True)
        self.assertEqual(free, False)
        self.assertRegex(manager.output[-1], "collision.*box.*ground")
