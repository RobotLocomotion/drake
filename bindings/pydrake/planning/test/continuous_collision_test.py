import pydrake.planning.continuous_collision as mut  # ruff: isort: skip

import unittest

import numpy as np

from pydrake.common import Parallelism
from pydrake.geometry import Box, Sphere
from pydrake.math import RigidTransform
from pydrake.multibody.plant import CoulombFriction
from pydrake.multibody.tree import (
    FixedOffsetFrame,
    PrismaticJoint,
    RevoluteJoint,
    SpatialInertia,
    UnitInertia,
)
from pydrake.planning import RobotDiagramBuilder
from pydrake.trajectories import PiecewisePolynomial


def _inertia():
    return SpatialInertia(
        mass=1.0,
        p_PScm_E=np.zeros(3),
        G_SP_E=UnitInertia(Ixx=1.0, Iyy=1.0, Izz=1.0),
    )


def _friction():
    return CoulombFriction(1.0, 1.0)


def _make_model():
    """A planar 2-dof arm (revolute shoulder, then prismatic slide carrying a
    tool sphere) with one anchored post at (0, 0.60, 0). q = (theta, slide);
    the tool sits at the post's center at q = (pi/2, 0.30).
    """
    builder = RobotDiagramBuilder()
    plant = builder.plant()
    link = plant.AddRigidBody(name="link", M_BBo_B=_inertia())
    tool = plant.AddRigidBody(name="tool", M_BBo_B=_inertia())
    plant.AddJoint(
        RevoluteJoint(
            name="shoulder",
            frame_on_parent=plant.world_frame(),
            frame_on_child=link.body_frame(),
            axis=[0, 0, 1],
        )
    )
    slide_frame = plant.AddFrame(
        FixedOffsetFrame(
            name="slide_offset",
            P=link.body_frame(),
            X_PF=RigidTransform([0.30, 0.0, 0.0]),
        )
    )
    plant.AddJoint(
        PrismaticJoint(
            name="slide",
            frame_on_parent=slide_frame,
            frame_on_child=tool.body_frame(),
            axis=[1, 0, 0],
        )
    )
    plant.RegisterCollisionGeometry(
        body=link,
        X_BG=RigidTransform([0.15, 0.0, 0.0]),
        shape=Box(0.30, 0.05, 0.05),
        name="link_geom",
        coulomb_friction=_friction(),
    )
    plant.RegisterCollisionGeometry(
        body=tool,
        X_BG=RigidTransform(),
        shape=Sphere(0.04),
        name="tool_geom",
        coulomb_friction=_friction(),
    )
    post = plant.AddRigidBody(name="post", M_BBo_B=_inertia())
    plant.WeldFrames(
        frame_on_parent_F=plant.world_frame(),
        frame_on_child_M=post.body_frame(),
        X_FM=RigidTransform([0.0, 0.60, 0.0]),
    )
    plant.RegisterCollisionGeometry(
        body=post,
        X_BG=RigidTransform(),
        shape=Sphere(0.08),
        name="post_geom",
        coulomb_friction=_friction(),
    )
    return builder.Build()


class TestContinuousCollision(unittest.TestCase):
    def setUp(self):
        self.model = _make_model()
        options = mut.Options()
        options.parallelism = Parallelism(num_threads=1)
        self.checker = mut.ContinuousCollisionChecker(
            model=self.model, default_options=options
        )

    def test_options_round_trip(self):
        dut = mut.Options()
        self.assertEqual(dut.margin, 0.0)
        self.assertEqual(dut.distance_resolution, 1e-6)
        dut.margin = 0.01
        dut.distance_resolution = 1e-4
        dut.continuous_revolute_indices = [0]
        dut.parallelism = Parallelism(num_threads=2)
        self.assertEqual(dut.margin, 0.01)
        self.assertEqual(dut.distance_resolution, 1e-4)
        self.assertEqual(dut.continuous_revolute_indices, [0])
        self.assertEqual(dut.parallelism.num_threads(), 2)
        self.assertIsInstance(mut.Options(margin=0.02), mut.Options)

    def test_model(self):
        self.assertIs(self.checker.model(), self.model)

    def test_free_edge(self):
        result = self.checker.CheckEdge(q1=[0.0, 0.0], q2=[0.0, 0.2])
        self.assertEqual(result.verdict, mut.Verdict.kCertifiedFree)
        self.assertIsNone(result.finding)
        self.assertGreater(result.num_nodes, 0)

    def test_colliding_edge(self):
        result = self.checker.CheckEdge(q1=[0.0, 0.0], q2=[np.pi / 2, 0.30])
        self.assertEqual(result.verdict, mut.Verdict.kViolationFound)
        finding = result.finding
        self.assertIsInstance(finding, mut.Finding)
        self.assertLess(finding.distance, 0.0)
        self.assertEqual(finding.q.shape, (2,))
        self.assertIsNotNone(finding.geometry_a)
        self.assertIsNotNone(finding.body_a)
        self.assertEqual(finding.nearest_a_W.shape, (3,))
        self.assertEqual(finding.nearest_b_W.shape, (3,))

    def test_trajectory_and_path(self):
        trajectory = PiecewisePolynomial.FirstOrderHold(
            breaks=[0.0, 1.0], samples=np.array([[0.0, 0.0], [0.0, 0.2]])
        )
        options = mut.Options(margin=0.01)
        result = self.checker.CheckTrajectory(
            trajectory=trajectory, options=options
        )
        self.assertEqual(result.verdict, mut.Verdict.kCertifiedFree)
        result = self.checker.CheckPath(
            waypoints=np.array([[0.0, 0.0, 0.0], [0.0, 0.1, 0.2]])
        )
        self.assertEqual(result.verdict, mut.Verdict.kCertifiedFree)

    def test_throw(self):
        with self.assertRaisesRegex(RuntimeError, "generalized positions"):
            self.checker.CheckEdge(q1=[0.0], q2=[0.0])
