import pydrake.geometry as mut

import unittest

import numpy as np

from pydrake.math import RigidTransform
from pydrake.solvers.mathematicalprogram import MathematicalProgram


class TestGeometryOptimization(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.A = np.eye(3)
        self.b = [1.0, 1.0, 1.0]
        self.prog = MathematicalProgram()
        self.x = self.prog.NewContinuousVariables(3, "x")
        self.t = self.prog.NewContinuousVariables(1, "t")

    def test_point_convex_set(self):
        p = np.array([11.1, 12.2, 13.3])
        point = mut.optimization.Point(p)
        self.assertEqual(point.ambient_dimension(), 3)
        np.testing.assert_array_equal(point.x(), p)
        point.set_x(x=2*p)
        np.testing.assert_array_equal(point.x(), 2*p)
        point.set_x(x=p)

        # TODO(SeanCurtis-TRI): This doesn't test the constructor that
        # builds from shape.

    def test_h_polyhedron(self):
        hpoly = mut.optimization.HPolyhedron(A=self.A, b=self.b)
        self.assertEqual(hpoly.ambient_dimension(), 3)
        np.testing.assert_array_equal(hpoly.A(), self.A)
        np.testing.assert_array_equal(hpoly.b(), self.b)
        self.assertTrue(hpoly.PointInSet(x=[0, 0, 0], tol=0.0))
        hpoly.AddPointInSetConstraints(self.prog, self.x)
        with self.assertRaisesRegex(
                RuntimeError, ".*not implemented yet for HPolyhedron.*"):
            hpoly.ToShapeWithPose()

        h_box = mut.optimization.HPolyhedron.MakeBox(
            lb=[-1, -1, -1], ub=[1, 1, 1])
        h_unit_box = mut.optimization.HPolyhedron.MakeUnitBox(dim=3)
        np.testing.assert_array_equal(h_box.A(), h_unit_box.A())
        np.testing.assert_array_equal(h_box.b(), h_unit_box.b())
        self.assertIsInstance(
            h_box.MaximumVolumeInscribedEllipsoid(),
            mut.optimization.Hyperellipsoid)
        np.testing.assert_array_almost_equal(
            h_box.ChebyshevCenter(), [0, 0, 0])

    def test_hyper_ellipsoid(self):
        ellipsoid = mut.optimization.Hyperellipsoid(A=self.A, center=self.b)
        self.assertEqual(ellipsoid.ambient_dimension(), 3)
        np.testing.assert_array_equal(ellipsoid.A(), self.A)
        np.testing.assert_array_equal(ellipsoid.center(), self.b)
        self.assertTrue(ellipsoid.PointInSet(x=self.b, tol=0.0))
        ellipsoid.AddPointInSetConstraints(self.prog, self.x)
        shape, pose = ellipsoid.ToShapeWithPose()
        self.assertIsInstance(shape, mut.Ellipsoid)
        self.assertIsInstance(pose, RigidTransform)
        p = np.array([11.1, 12.2, 13.3])
        point = mut.optimization.Point(p)
        scale, witness = ellipsoid.MinimumUniformScalingToTouch(point)
        self.assertTrue(scale > 0.0)
        np.testing.assert_array_almost_equal(witness, p)
        e_ball = mut.optimization.Hyperellipsoid.MakeAxisAligned(
            radius=[1, 1, 1], center=self.b)
        np.testing.assert_array_equal(e_ball.A(), self.A)
        np.testing.assert_array_equal(e_ball.center(), self.b)
        e_ball2 = mut.optimization.Hyperellipsoid.MakeHypersphere(
            radius=1, center=self.b)
        np.testing.assert_array_equal(e_ball2.A(), self.A)
        np.testing.assert_array_equal(e_ball2.center(), self.b)
        e_ball3 = mut.optimization.Hyperellipsoid.MakeUnitBall(dim=3)
        np.testing.assert_array_equal(e_ball3.A(), self.A)
        np.testing.assert_array_equal(e_ball3.center(), [0, 0, 0])

    def test_minkowski_sum(self):
        point = mut.optimization.Point(np.array([11.1, 12.2, 13.3]))
        hpoly = mut.optimization.HPolyhedron(A=self.A, b=self.b)
        sum = mut.optimization.MinkowskiSum(setA=point, setB=hpoly)
        self.assertEqual(sum.ambient_dimension(), 3)
        self.assertEqual(sum.num_terms(), 2)
        sum2 = mut.optimization.MinkowskiSum(sets=[point, hpoly])
        self.assertEqual(sum2.ambient_dimension(), 3)
        self.assertEqual(sum2.num_terms(), 2)
        self.assertIsInstance(sum2.term(0), mut.optimization.Point)

    def test_v_polytope(self):
        vertices = np.array([[0.0, 1.0, 2.0], [3.0, 7.0, 5.0]])
        vpoly = mut.optimization.VPolytope(vertices=vertices)
        self.assertEqual(vpoly.ambient_dimension(), 2)
        np.testing.assert_array_equal(vpoly.vertices(), vertices)
        self.assertTrue(vpoly.PointInSet(x=[1.0, 5.0], tol=1e-8))
        vpoly.AddPointInSetConstraints(self.prog, self.x[0:2])
        v_box = mut.optimization.VPolytope.MakeBox(
            lb=[-1, -1, -1], ub=[1, 1, 1])
        self.assertTrue(v_box.PointInSet([0, 0, 0]))
        v_unit_box = mut.optimization.VPolytope.MakeUnitBox(dim=3)
        self.assertTrue(v_unit_box.PointInSet([0, 0, 0]))

    def test_cartesian_product(self):
        point = mut.optimization.Point(np.array([11.1, 12.2, 13.3]))
        h_box = mut.optimization.HPolyhedron.MakeBox(
            lb=[-1, -1, -1], ub=[1, 1, 1])
        sum = mut.optimization.CartesianProduct(setA=point, setB=h_box)
        self.assertEqual(sum.ambient_dimension(), 6)
        self.assertEqual(sum.num_factors(), 2)
        sum2 = mut.optimization.CartesianProduct(sets=[point, h_box])
        self.assertEqual(sum2.ambient_dimension(), 6)
        self.assertEqual(sum2.num_factors(), 2)
        self.assertIsInstance(sum2.factor(0), mut.optimization.Point)
        sum2 = mut.optimization.CartesianProduct(
            sets=[point, h_box], A=np.eye(6, 3), b=[0, 1, 2, 3, 4, 5])
        self.assertEqual(sum2.ambient_dimension(), 3)
        self.assertEqual(sum2.num_factors(), 2)
        self.assertIsInstance(sum2.factor(1), mut.optimization.HPolyhedron)

    def test_make_from_scene_graph_and_iris(self):
        """
        Tests the make from scene graph and iris functionality together as
        the Iris code makes obstacles from geometries registered in SceneGraph.
        """
        scene_graph = mut.SceneGraph()
        source_id = scene_graph.RegisterSource("source")
        frame_id = scene_graph.RegisterFrame(
            source_id=source_id, frame=mut.GeometryFrame("frame"))
        box_geometry_id = scene_graph.RegisterGeometry(
            source_id=source_id, frame_id=frame_id,
            geometry=mut.GeometryInstance(X_PG=RigidTransform(),
                                          shape=mut.Box(1., 1., 1.),
                                          name="box"))
        cylinder_geometry_id = scene_graph.RegisterGeometry(
            source_id=source_id, frame_id=frame_id,
            geometry=mut.GeometryInstance(X_PG=RigidTransform(),
                                          shape=mut.Cylinder(1., 1.),
                                          name="cylinder"))
        sphere_geometry_id = scene_graph.RegisterGeometry(
            source_id=source_id, frame_id=frame_id,
            geometry=mut.GeometryInstance(X_PG=RigidTransform(),
                                          shape=mut.Sphere(1.), name="sphere"))
        capsule_geometry_id = scene_graph.RegisterGeometry(
            source_id=source_id,
            frame_id=frame_id,
            geometry=mut.GeometryInstance(X_PG=RigidTransform(),
                                          shape=mut.Capsule(1., 1.0),
                                          name="capsule"))
        context = scene_graph.CreateDefaultContext()
        pose_vector = mut.FramePoseVector()
        pose_vector.set_value(frame_id, RigidTransform())
        scene_graph.get_source_pose_port(source_id).FixValue(
            context, pose_vector)
        query_object = scene_graph.get_query_output_port().Eval(context)
        H = mut.optimization.HPolyhedron(
            query_object=query_object, geometry_id=box_geometry_id,
            reference_frame=scene_graph.world_frame_id())
        self.assertEqual(H.ambient_dimension(), 3)
        C = mut.optimization.CartesianProduct(
            query_object=query_object, geometry_id=cylinder_geometry_id,
            reference_frame=scene_graph.world_frame_id())
        self.assertEqual(C.ambient_dimension(), 3)
        E = mut.optimization.Hyperellipsoid(
            query_object=query_object, geometry_id=sphere_geometry_id,
            reference_frame=scene_graph.world_frame_id())
        self.assertEqual(E.ambient_dimension(), 3)
        S = mut.optimization.MinkowskiSum(
            query_object=query_object, geometry_id=capsule_geometry_id,
            reference_frame=scene_graph.world_frame_id())
        self.assertEqual(S.ambient_dimension(), 3)
        P = mut.optimization.Point(
            query_object=query_object, geometry_id=sphere_geometry_id,
            reference_frame=scene_graph.world_frame_id(),
            maximum_allowable_radius=1.5)
        self.assertEqual(P.ambient_dimension(), 3)
        V = mut.optimization.VPolytope(
            query_object=query_object, geometry_id=box_geometry_id,
            reference_frame=scene_graph.world_frame_id())
        self.assertEqual(V.ambient_dimension(), 3)

        obstacles = mut.optimization.MakeIrisObstacles(
            query_object=query_object,
            reference_frame=scene_graph.world_frame_id())
        options = mut.optimization.IrisOptions()
        options.require_sample_point_is_contained = True
        options.iteration_limit = 1
        options.termination_threshold = 0.1
        region = mut.optimization.Iris(
            obstacles=obstacles, sample=[2, 3.4, 5],
            domain=mut.optimization.HPolyhedron.MakeBox(
                lb=[-5, -5, -5], ub=[5, 5, 5]), options=options)
        self.assertIsInstance(region, mut.optimization.HPolyhedron)

        obstacles = [
            mut.optimization.HPolyhedron.MakeUnitBox(3),
            mut.optimization.Hyperellipsoid.MakeUnitBall(3),
            mut.optimization.Point([0, 0, 0]),
            mut.optimization.VPolytope.MakeUnitBox(3)]
        region = mut.optimization.Iris(
            obstacles=obstacles, sample=[2, 3.4, 5],
            domain=mut.optimization.HPolyhedron.MakeBox(
                lb=[-5, -5, -5], ub=[5, 5, 5]), options=options)
        self.assertIsInstance(region, mut.optimization.HPolyhedron)
