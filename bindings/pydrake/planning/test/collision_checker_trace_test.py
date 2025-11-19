import pydrake.planning as mut  # ruff: isort: skip

import textwrap
import unittest

import numpy as np


class TestCollisionCheckerTrace(unittest.TestCase):
    """Confirms that trace-level logging from the CollisionChecker's C++ worker
    threads doesn't deadlock the Python interpreter.

    A deadlock can happen when:
    - the default pydrake logging configuration is still intact, where all text
      logging flows through Python (i.e., use_native_cpp_logging() was not
      called, DRAKE_PYTHON_LOGGING was not set, etc);
    - some Python code calls a bound C++ function;
    - that binding is NOT annotated with `gil_scoped_release`, so keeps hold of
      its lock on the Python GIL;
    - the C++ function spawns worker thread(s), and blocks awaiting their
      completion (while still holding the GIL);
    - one of the worker threads posts a log message using a log level that
      isn't filtered out;
    - the log sink blocks forever while trying to acquire the GIL, because the
      lock is still held by the original function.

    In short, here we are proving that the `gil_scoped_release` annotation is
    present on the relevant function (CheckEdgeCollisionFreeParallel) and that
    that annotation is sufficient to prevent a deadlock.
    """

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
            model=robot, robot_model_instances=[index], edge_step_size=0.125
        )

        # Check an edge that has a box-ground collision.
        q1 = np.array([-0.5] * 7)
        q2 = np.array([0.5] * 7)
        with self.assertLogs("drake", level=1) as manager:
            free = dut.CheckEdgeCollisionFreeParallel(q1, q2, parallelize=True)
        self.assertEqual(free, False)
        self.assertRegex(manager.output[-1], "collision.*box.*ground")
