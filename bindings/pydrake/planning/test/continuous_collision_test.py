import pydrake.planning.continuous_collision as mut

import unittest

import numpy as np

from pydrake.common import Parallelism
from pydrake.geometry import Box, Sphere
from pydrake.geometry.optimization import VPolytope
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
from pydrake.trajectories import BezierCurve


def _inertia():
    return SpatialInertia(mass=1.0, p_PScm_E=np.zeros(3),
                          G_SP_E=UnitInertia(Ixx=1.0, Iyy=1.0, Izz=1.0))


def _make_arm_builder():
    """A planar 2-dof arm (revolute, then prismatic) with one anchored post
    obstacle -- the same world planning/continuous_collision/test/api_test.cc
    uses, so the verdicts asserted below match the C++ suite. Returns the
    not-yet-built RobotDiagramBuilder (its plant is not finalized).
    """
    builder = RobotDiagramBuilder()
    plant = builder.plant()
    link = plant.AddRigidBody(name="link", M_BBo_B=_inertia())
    tool = plant.AddRigidBody(name="tool", M_BBo_B=_inertia())
    plant.AddJoint(RevoluteJoint(
        name="shoulder",
        frame_on_parent=plant.world_frame(),
        frame_on_child=link.body_frame(),
        axis=[0, 0, 1]))
    slide_frame = plant.AddFrame(FixedOffsetFrame(
        name="slide_offset",
        P=link.body_frame(),
        X_PF=RigidTransform([0.30, 0.0, 0.0])))
    plant.AddJoint(PrismaticJoint(
        name="slide",
        frame_on_parent=slide_frame,
        frame_on_child=tool.body_frame(),
        axis=[1, 0, 0]))
    plant.RegisterCollisionGeometry(
        body=link, X_BG=RigidTransform([0.15, 0.0, 0.0]),
        shape=Box(0.30, 0.05, 0.05), name="link_geom",
        coulomb_friction=CoulombFriction(1.0, 1.0))
    plant.RegisterCollisionGeometry(
        body=tool, X_BG=RigidTransform(), shape=Sphere(0.04),
        name="tool_geom", coulomb_friction=CoulombFriction(1.0, 1.0))
    post = plant.AddRigidBody(name="post", M_BBo_B=_inertia())
    plant.WeldFrames(frame_on_parent_F=plant.world_frame(),
                     frame_on_child_M=post.body_frame(),
                     X_FM=RigidTransform([0.0, 0.60, 0.0]))
    plant.RegisterCollisionGeometry(
        body=post, X_BG=RigidTransform(), shape=Sphere(0.08),
        name="post_geom", coulomb_friction=CoulombFriction(1.0, 1.0))
    return builder


def _serial_options():
    options = mut.Options()
    options.parallelism = Parallelism(num_threads=1)
    return options


class TestContinuousCollision(unittest.TestCase):
    def setUp(self):
        self.model = _make_arm_builder().Build()
        self.checker = mut.ContinuousCollisionChecker(
            model=self.model, default_options=_serial_options())

    def test_options(self):
        """Exercises the Options / PaddingSpec / enum surface."""
        dut = mut.Options()
        self.assertEqual(dut.margin, 0.0)
        self.assertEqual(dut.mode, mut.SearchMode.kCertifyAll)
        self.assertIsNone(dut.max_nodes)
        dut.margin = 0.01
        dut.continuity_tolerance = 1e-6
        dut.query_tolerance = 1e-5
        dut.certificate_slack = 1e-8
        dut.min_interval = 1e-8
        dut.continuous_revolute_indices = [0]
        dut.max_conversion_degree = 8
        dut.mode = mut.SearchMode.kFindFirstViolation
        dut.max_reported_findings = 4
        dut.max_nodes = 10000
        dut.emit_certificate = True
        dut.parallelism = Parallelism(num_threads=1)
        self.assertEqual(dut.margin, 0.01)
        self.assertEqual(dut.max_nodes, 10000)
        self.assertTrue(dut.emit_certificate)
        self.assertEqual(dut.parallelism.num_threads(), 1)
        self.assertEqual(dut.continuous_revolute_indices, [0])

        # kwargs-init round trip.
        kwargs_dut = mut.Options(margin=0.02, max_reported_findings=7)
        self.assertEqual(kwargs_dut.margin, 0.02)
        self.assertEqual(kwargs_dut.max_reported_findings, 7)

        padding = mut.PaddingSpec(env_padding=0.001, self_padding=0.002)
        self.assertEqual(padding.env_padding, 0.001)
        self.assertEqual(padding.self_padding, 0.002)
        self.assertIsNone(padding.per_body_pair)

        # The enums are complete.
        self.assertEqual(len(mut.Verdict.__members__), 4)
        self.assertEqual(len(mut.SearchMode.__members__), 2)
        self.assertEqual(len(mut.DistanceRoute.__members__), 3)

    def test_params_and_introspection(self):
        params = mut.ContinuousCollisionChecker.Params()
        params.model = self.model
        params.padding = mut.PaddingSpec(env_padding=0.0)
        params.default_options = _serial_options()
        self.assertIs(params.model, self.model)
        checker = mut.ContinuousCollisionChecker(params=params)

        self.assertIs(checker.model(), self.model)
        self.assertGreater(len(checker.pairs()), 0)
        self.assertIsInstance(checker.pairs()[0], mut.PairRecord)
        self.assertIsInstance(checker.pairs()[0].id, mut.PairId)
        self.assertIsInstance(checker.pairs()[0].route, mut.DistanceRoute)

        oracle = checker.distance_oracle()
        self.assertIsInstance(oracle, mut.DistanceOracle)
        self.assertGreater(oracle.tolerance(), 0.0)
        self.assertIsInstance(oracle.support_report(), str)
        self.assertGreater(len(oracle.support_report()), 0)

        engine = checker.kinematics_engine()
        self.assertIsInstance(engine, mut.KinematicsEngine)
        self.assertEqual(engine.num_positions(), 2)
        pair = checker.pairs()[0].id
        coords = engine.CoordinatesAffectingPair(body_a=pair.body_a,
                                                 body_b=pair.body_b)
        self.assertIsInstance(coords, list)

    def test_check_edge_free_and_colliding(self):
        """A free edge certifies; a sweep past the post reports a violation."""
        free = self.checker.CheckEdge(q1=[0.0, 0.0], q2=[0.3, 0.05])
        self.assertEqual(free.verdict, mut.Verdict.kCertifiedFree)
        self.assertEqual(len(free.findings), 0)
        self.assertGreater(free.stats.nodes, 0)
        self.assertGreaterEqual(free.stats.max_depth, 0)
        self.assertIsNone(free.certificate)

        # Sweeping theta from 0 to 2.4 rad with the tool extended drives the
        # tool sphere through the anchored post.
        hit = self.checker.CheckEdge(q1=[0.0, 0.25], q2=[2.4, 0.25])
        self.assertEqual(hit.verdict, mut.Verdict.kViolationFound)
        self.assertGreater(len(hit.findings), 0)
        finding = hit.findings[0]
        self.assertIsInstance(finding, mut.Finding)
        self.assertTrue(finding.definite)
        self.assertEqual(len(finding.q), 2)
        self.assertIsInstance(finding.pair, mut.PairId)
        self.assertLess(finding.distance, 1.0)

    def test_check_path_and_trajectory(self):
        waypoints = np.array([[0.0, 0.3], [0.0, 0.05]])
        result = self.checker.CheckPath(waypoints=waypoints)
        self.assertEqual(result.verdict, mut.Verdict.kCertifiedFree)

        trajectory = BezierCurve(0.0, 1.0, waypoints)
        result = self.checker.CheckTrajectory(trajectory=trajectory)
        self.assertEqual(result.verdict, mut.Verdict.kCertifiedFree)

        # Normalize + ComputeMotionBounds introspection seams.
        path = self.checker.Normalize(trajectory=trajectory)
        self.assertIsInstance(path, mut.PiecewiseBezierPath)
        table = self.checker.ComputeMotionBounds(path=path)
        self.assertIsInstance(table, mut.MotionBoundTable)
        self.assertEqual(table.num_pairs(), len(self.checker.pairs()))
        w = np.full(path.num_positions(), 0.1)
        self.assertGreaterEqual(table.MotionBound(pair_index=0, w=w), 0.0)
        self.assertGreaterEqual(table.carveout_slack(pair_index=0), 0.0)
        self.assertIsInstance(table.pair_is_static(pair_index=0), bool)
        self.assertIsInstance(table.GetEntries(pair_index=0), list)
        self.assertGreaterEqual(table.num_entries(), 0)

    def test_certificate_round_trip(self):
        options = _serial_options()
        options.emit_certificate = True
        q1 = np.array([0.0, 0.0])
        q2 = np.array([0.3, 0.05])
        result = self.checker.CheckEdge(q1=q1, q2=q2, options=options)
        self.assertEqual(result.verdict, mut.Verdict.kCertifiedFree)
        certificate = result.certificate
        self.assertIsInstance(certificate, mut.Certificate)
        self.assertGreater(len(certificate.records), 0)
        self.assertGreater(len(certificate.pairs), 0)
        record = certificate.records[0]
        self.assertIsInstance(record, mut.CertificateRecord)
        self.assertGreaterEqual(record.s_end, record.s_start)
        self.assertEqual(len(record.qc), 2)

        # CheckEdge normalizes exactly this waypoint matrix, so the replay
        # runs against the same path the certificate was recorded on.
        path = mut.PiecewiseBezierPath.FromWaypoints(
            waypoints=np.column_stack([q1, q2]), options=options)
        self.assertTrue(mut.VerifyCertificate(
            checker=self.checker, path=path, certificate=certificate))

        # A tampered certificate must not verify.
        tampered = mut.Certificate(records=list(certificate.records),
                                   pairs=list(certificate.pairs))
        bad = tampered.records[0]
        bad.phi_hat = bad.phi_hat + 100.0
        tampered.records = [bad] + list(tampered.records[1:])
        self.assertFalse(mut.VerifyCertificate(
            checker=self.checker, path=path, certificate=tampered))

    def test_piecewise_bezier_path(self):
        options = mut.Options()
        waypoints = np.array([[0.0, 0.3, 0.6], [0.0, 0.05, 0.10]])
        dut = mut.PiecewiseBezierPath.FromWaypoints(waypoints=waypoints,
                                                    options=options)
        self.assertEqual(dut.num_positions(), 2)
        self.assertEqual(len(dut.segments()), 2)
        self.assertIsInstance(dut.segments()[0], mut.BezierSegment)
        self.assertEqual(dut.start_time(), 0.0)
        self.assertEqual(dut.end_time(), 2.0)
        np.testing.assert_allclose(dut.Value(t=0.0), waypoints[:, 0])
        np.testing.assert_allclose(dut.Value(t=2.0), waypoints[:, 2])
        np.testing.assert_allclose(dut.EvaluateSegment(segment_index=0, s=0.0),
                                   waypoints[:, 0])
        np.testing.assert_allclose(dut.global_lower_bound(), waypoints[:, 0])
        np.testing.assert_allclose(dut.global_upper_bound(), waypoints[:, 2])
        self.assertEqual(len(dut.constant_coordinates()), 2)

        trajectory = BezierCurve(0.0, 1.0, waypoints)
        from_traj = mut.PiecewiseBezierPath.FromTrajectory(
            trajectory=trajectory, options=options)
        self.assertEqual(from_traj.num_positions(), 2)

        # The out-params of DeCasteljauSplitAtHalf come back as a tuple.
        left, right, mid = mut.DeCasteljauSplitAtHalf(cps=waypoints)
        self.assertEqual(left.shape, waypoints.shape)
        self.assertEqual(right.shape, waypoints.shape)
        np.testing.assert_allclose(mid, from_traj.Value(t=0.5))

    def test_bounding_sphere(self):
        dut = mut.ComputeBoundingSphere(
            shape=Sphere(0.25), X_LG=RigidTransform([1.0, 2.0, 3.0]))
        self.assertIsInstance(dut, mut.BoundingSphere)
        self.assertEqual(dut.radius, 0.25)
        np.testing.assert_allclose(dut.center_L, [1.0, 2.0, 3.0])

        box = mut.ComputeBoundingSphere(shape=Box(2.0, 2.0, 2.0),
                                        X_LG=RigidTransform())
        self.assertAlmostEqual(box.radius, np.sqrt(3.0))

    def test_add_vpolytope_obstacle(self):
        """AddVPolytopeObstacle runs on a pre-finalize plant."""
        builder = _make_arm_builder()
        plant = builder.plant()
        vertices = np.array([
            [0.0, 0.1, 0.0, 0.0],
            [0.0, 0.0, 0.1, 0.0],
            [0.0, 0.0, 0.0, 0.1],
        ])
        geometry_id = mut.AddVPolytopeObstacle(
            plant=plant, vpoly=VPolytope(vertices),
            X_WG=RigidTransform([0.0, -0.60, 0.0]), name="vpoly_obstacle")
        self.assertIsNotNone(geometry_id)
        # The new obstacle rides the ordinary narrowphase path, so a checker
        # built on the finalized diagram picks it up as an extra pair.
        model = builder.Build()
        checker = mut.ContinuousCollisionChecker(
            model=model, default_options=_serial_options())
        ids = set()
        for pair in checker.pairs():
            ids.add(pair.id.a)
            ids.add(pair.id.b)
        self.assertIn(geometry_id, ids)

    def test_numerics(self):
        self.assertTrue(mut.IsCertified(phi_hat=1.0, tau=1e-6,
                                        motion_bound=0.1, threshold=0.0,
                                        slack=1e-9))
        self.assertFalse(mut.IsCertified(phi_hat=0.05, tau=1e-6,
                                         motion_bound=0.1, threshold=0.0,
                                         slack=1e-9))
        self.assertTrue(mut.IsDefiniteViolation(phi_hat=-0.1, tau=1e-6,
                                                threshold=0.0))
        self.assertFalse(mut.IsDefiniteViolation(phi_hat=0.1, tau=1e-6,
                                                 threshold=0.0))
