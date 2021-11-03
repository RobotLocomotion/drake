import pydrake.geometry.optimization as mut

import unittest

import numpy as np

from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.geometry import (
    Box, Capsule, Cylinder, Ellipsoid, FramePoseVector, GeometryFrame,
    GeometryInstance, SceneGraph, Sphere,
)
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.solvers.mathematicalprogram import (
    MathematicalProgram, MathematicalProgramResult, Binding, Cost, Constraint
)
from pydrake.symbolic import Variable


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
        point = mut.Point(p)
        self.assertEqual(point.ambient_dimension(), 3)
        np.testing.assert_array_equal(point.x(), p)
        point.set_x(x=2*p)
        np.testing.assert_array_equal(point.x(), 2*p)
        point.set_x(x=p)

        # TODO(SeanCurtis-TRI): This doesn't test the constructor that
        # builds from shape.

    def test_h_polyhedron(self):
        hpoly = mut.HPolyhedron(A=self.A, b=self.b)
        self.assertEqual(hpoly.ambient_dimension(), 3)
        np.testing.assert_array_equal(hpoly.A(), self.A)
        np.testing.assert_array_equal(hpoly.b(), self.b)
        self.assertTrue(hpoly.PointInSet(x=[0, 0, 0], tol=0.0))
        self.assertFalse(hpoly.IsBounded())
        hpoly.AddPointInSetConstraints(self.prog, self.x)
        with self.assertRaisesRegex(
                RuntimeError, ".*not implemented yet for HPolyhedron.*"):
            hpoly.ToShapeWithPose()

        h_box = mut.HPolyhedron.MakeBox(
            lb=[-1, -1, -1], ub=[1, 1, 1])
        self.assertTrue(h_box.IntersectsWith(hpoly))
        h_unit_box = mut.HPolyhedron.MakeUnitBox(dim=3)
        np.testing.assert_array_equal(h_box.A(), h_unit_box.A())
        np.testing.assert_array_equal(h_box.b(), h_unit_box.b())
        self.assertIsInstance(
            h_box.MaximumVolumeInscribedEllipsoid(),
            mut.Hyperellipsoid)
        np.testing.assert_array_almost_equal(
            h_box.ChebyshevCenter(), [0, 0, 0])
        h2 = h_box.CartesianProduct(other=h_unit_box)
        self.assertIsInstance(h2, mut.HPolyhedron)
        self.assertEqual(h2.ambient_dimension(), 6)
        h3 = h_box.CartesianPower(n=3)
        self.assertIsInstance(h3, mut.HPolyhedron)
        self.assertEqual(h3.ambient_dimension(), 9)

    def test_hyper_ellipsoid(self):
        ellipsoid = mut.Hyperellipsoid(A=self.A, center=self.b)
        self.assertEqual(ellipsoid.ambient_dimension(), 3)
        np.testing.assert_array_equal(ellipsoid.A(), self.A)
        np.testing.assert_array_equal(ellipsoid.center(), self.b)
        self.assertTrue(ellipsoid.PointInSet(x=self.b, tol=0.0))
        ellipsoid.AddPointInSetConstraints(self.prog, self.x)
        shape, pose = ellipsoid.ToShapeWithPose()
        self.assertIsInstance(shape, Ellipsoid)
        self.assertIsInstance(pose, RigidTransform)
        p = np.array([11.1, 12.2, 13.3])
        point = mut.Point(p)
        scale, witness = ellipsoid.MinimumUniformScalingToTouch(point)
        self.assertTrue(scale > 0.0)
        np.testing.assert_array_almost_equal(witness, p)
        e_ball = mut.Hyperellipsoid.MakeAxisAligned(
            radius=[1, 1, 1], center=self.b)
        np.testing.assert_array_equal(e_ball.A(), self.A)
        np.testing.assert_array_equal(e_ball.center(), self.b)
        e_ball2 = mut.Hyperellipsoid.MakeHypersphere(
            radius=1, center=self.b)
        np.testing.assert_array_equal(e_ball2.A(), self.A)
        np.testing.assert_array_equal(e_ball2.center(), self.b)
        e_ball3 = mut.Hyperellipsoid.MakeUnitBall(dim=3)
        np.testing.assert_array_equal(e_ball3.A(), self.A)
        np.testing.assert_array_equal(e_ball3.center(), [0, 0, 0])

    def test_minkowski_sum(self):
        point = mut.Point(np.array([11.1, 12.2, 13.3]))
        hpoly = mut.HPolyhedron(A=self.A, b=self.b)
        sum = mut.MinkowskiSum(setA=point, setB=hpoly)
        self.assertEqual(sum.ambient_dimension(), 3)
        self.assertEqual(sum.num_terms(), 2)
        sum2 = mut.MinkowskiSum(sets=[point, hpoly])
        self.assertEqual(sum2.ambient_dimension(), 3)
        self.assertEqual(sum2.num_terms(), 2)
        self.assertIsInstance(sum2.term(0), mut.Point)

    def test_v_polytope(self):
        vertices = np.array([[0.0, 1.0, 2.0], [3.0, 7.0, 5.0]])
        vpoly = mut.VPolytope(vertices=vertices)
        self.assertEqual(vpoly.ambient_dimension(), 2)
        np.testing.assert_array_equal(vpoly.vertices(), vertices)
        self.assertTrue(vpoly.PointInSet(x=[1.0, 5.0], tol=1e-8))
        vpoly.AddPointInSetConstraints(self.prog, self.x[0:2])
        v_box = mut.VPolytope.MakeBox(
            lb=[-1, -1, -1], ub=[1, 1, 1])
        self.assertTrue(v_box.PointInSet([0, 0, 0]))
        v_unit_box = mut.VPolytope.MakeUnitBox(dim=3)
        self.assertTrue(v_unit_box.PointInSet([0, 0, 0]))

    def test_cartesian_product(self):
        point = mut.Point(np.array([11.1, 12.2, 13.3]))
        h_box = mut.HPolyhedron.MakeBox(
            lb=[-1, -1, -1], ub=[1, 1, 1])
        sum = mut.CartesianProduct(setA=point, setB=h_box)
        self.assertEqual(sum.ambient_dimension(), 6)
        self.assertEqual(sum.num_factors(), 2)
        sum2 = mut.CartesianProduct(sets=[point, h_box])
        self.assertEqual(sum2.ambient_dimension(), 6)
        self.assertEqual(sum2.num_factors(), 2)
        self.assertIsInstance(sum2.factor(0), mut.Point)
        sum2 = mut.CartesianProduct(
            sets=[point, h_box], A=np.eye(6, 3), b=[0, 1, 2, 3, 4, 5])
        self.assertEqual(sum2.ambient_dimension(), 3)
        self.assertEqual(sum2.num_factors(), 2)
        self.assertIsInstance(sum2.factor(1), mut.HPolyhedron)

    def test_make_from_scene_graph_and_iris(self):
        """
        Tests the make from scene graph and iris functionality together as
        the Iris code makes obstacles from geometries registered in SceneGraph.
        """
        scene_graph = SceneGraph()
        source_id = scene_graph.RegisterSource("source")
        frame_id = scene_graph.RegisterFrame(
            source_id=source_id, frame=GeometryFrame("frame"))
        box_geometry_id = scene_graph.RegisterGeometry(
            source_id=source_id, frame_id=frame_id,
            geometry=GeometryInstance(X_PG=RigidTransform(),
                                      shape=Box(1., 1., 1.),
                                      name="box"))
        cylinder_geometry_id = scene_graph.RegisterGeometry(
            source_id=source_id, frame_id=frame_id,
            geometry=GeometryInstance(X_PG=RigidTransform(),
                                      shape=Cylinder(1., 1.),
                                      name="cylinder"))
        sphere_geometry_id = scene_graph.RegisterGeometry(
            source_id=source_id, frame_id=frame_id,
            geometry=GeometryInstance(X_PG=RigidTransform(),
                                      shape=Sphere(1.), name="sphere"))
        capsule_geometry_id = scene_graph.RegisterGeometry(
            source_id=source_id,
            frame_id=frame_id,
            geometry=GeometryInstance(X_PG=RigidTransform(),
                                      shape=Capsule(1., 1.0),
                                      name="capsule"))
        context = scene_graph.CreateDefaultContext()
        pose_vector = FramePoseVector()
        pose_vector.set_value(frame_id, RigidTransform())
        scene_graph.get_source_pose_port(source_id).FixValue(
            context, pose_vector)
        query_object = scene_graph.get_query_output_port().Eval(context)
        H = mut.HPolyhedron(
            query_object=query_object, geometry_id=box_geometry_id,
            reference_frame=scene_graph.world_frame_id())
        self.assertEqual(H.ambient_dimension(), 3)
        C = mut.CartesianProduct(
            query_object=query_object, geometry_id=cylinder_geometry_id,
            reference_frame=scene_graph.world_frame_id())
        self.assertEqual(C.ambient_dimension(), 3)
        E = mut.Hyperellipsoid(
            query_object=query_object, geometry_id=sphere_geometry_id,
            reference_frame=scene_graph.world_frame_id())
        self.assertEqual(E.ambient_dimension(), 3)
        S = mut.MinkowskiSum(
            query_object=query_object, geometry_id=capsule_geometry_id,
            reference_frame=scene_graph.world_frame_id())
        self.assertEqual(S.ambient_dimension(), 3)
        P = mut.Point(
            query_object=query_object, geometry_id=sphere_geometry_id,
            reference_frame=scene_graph.world_frame_id(),
            maximum_allowable_radius=1.5)
        self.assertEqual(P.ambient_dimension(), 3)
        V = mut.VPolytope(
            query_object=query_object, geometry_id=box_geometry_id,
            reference_frame=scene_graph.world_frame_id())
        self.assertEqual(V.ambient_dimension(), 3)

        obstacles = mut.MakeIrisObstacles(
            query_object=query_object,
            reference_frame=scene_graph.world_frame_id())
        options = mut.IrisOptions()
        options.require_sample_point_is_contained = True
        options.iteration_limit = 1
        options.termination_threshold = 0.1
        options.relative_termination_threshold = 0.01
        self.assertNotIn("object at 0x", repr(options))
        region = mut.Iris(
            obstacles=obstacles, sample=[2, 3.4, 5],
            domain=mut.HPolyhedron.MakeBox(
                lb=[-5, -5, -5], ub=[5, 5, 5]), options=options)
        self.assertIsInstance(region, mut.HPolyhedron)

        obstacles = [
            mut.HPolyhedron.MakeUnitBox(3),
            mut.Hyperellipsoid.MakeUnitBall(3),
            mut.Point([0, 0, 0]),
            mut.VPolytope.MakeUnitBox(3)]
        region = mut.Iris(
            obstacles=obstacles, sample=[2, 3.4, 5],
            domain=mut.HPolyhedron.MakeBox(
                lb=[-5, -5, -5], ub=[5, 5, 5]), options=options)
        self.assertIsInstance(region, mut.HPolyhedron)

    def test_iris_cspace(self):
        limits_urdf = """
<robot name="limits">
  <link name="movable">
    <collision>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
  </link>
  <joint name="movable" type="prismatic">
    <axis xyz="1 0 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="world"/>
    <child link="movable"/>
  </joint>
</robot>"""
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        Parser(plant).AddModelFromString(limits_urdf, "urdf")
        plant.Finalize()
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        options = mut.IrisOptions()
        with catch_drake_warnings(expected_count=1):
            region = mut.IrisInConfigurationSpace(
                plant=plant, context=plant.GetMyContextFromRoot(context),
                sample=[0], options=options)
        plant.SetPositions(plant.GetMyMutableContextFromRoot(context), [0])
        region = mut.IrisInConfigurationSpace(
            plant=plant, context=plant.GetMyContextFromRoot(context),
            options=options)
        self.assertIsInstance(region, mut.ConvexSet)
        self.assertEqual(region.ambient_dimension(), 1)
        self.assertTrue(region.PointInSet([1.0]))
        self.assertFalse(region.PointInSet([3.0]))

    def test_graph_of_convex_sets(self):
        spp = mut.GraphOfConvexSets()
        source = spp.AddVertex(set=mut.Point([0.1]), name="source")
        target = spp.AddVertex(set=mut.Point([0.2]), name="target")
        edge0 = spp.AddEdge(u=source, v=target, name="edge0")
        edge1 = spp.AddEdge(u_id=source.id(), v_id=target.id(), name="edge1")
        self.assertEqual(len(spp.Vertices()), 2)
        self.assertEqual(len(spp.Edges()), 2)
        result = spp.SolveShortestPath(
            source_id=source.id(), target_id=target.id(),
            convex_relaxation=True)
        self.assertIsInstance(result, MathematicalProgramResult)
        self.assertIsInstance(spp.SolveShortestPath(
            source=source, target=target, convex_relaxation=True),
            MathematicalProgramResult)
        self.assertIn("source", spp.GetGraphvizString(
            result=result, show_slacks=True, precision=2, scientific=False))

        # Vertex
        self.assertIsInstance(source.id(), mut.GraphOfConvexSets.VertexId)
        self.assertEqual(source.ambient_dimension(), 1)
        self.assertEqual(source.name(), "source")
        self.assertIsInstance(source.x()[0], Variable)
        self.assertIsInstance(source.set(), mut.Point)
        np.testing.assert_array_almost_equal(
            source.GetSolution(result), [0.1], 1e-6)

        # Edge
        self.assertAlmostEqual(edge0.GetSolutionCost(result=result), 0.0, 1e-6)
        np.testing.assert_array_almost_equal(
            edge0.GetSolutionPhiXu(result=result), [0.1], 1e-6)
        np.testing.assert_array_almost_equal(
            edge0.GetSolutionPhiXv(result=result), [0.2], 1e-6)
        self.assertIsInstance(edge0.id(), mut.GraphOfConvexSets.EdgeId)
        self.assertEqual(edge0.name(), "edge0")
        self.assertEqual(edge0.u(), source)
        self.assertEqual(edge0.v(), target)
        self.assertIsInstance(edge0.phi(), Variable)
        self.assertIsInstance(edge0.xu()[0], Variable)
        self.assertIsInstance(edge0.xv()[0], Variable)
        var, binding = edge0.AddCost(e=1.0+edge0.xu()[0])
        self.assertIsInstance(var, Variable)
        self.assertIsInstance(binding, Binding[Cost])
        var, binding = edge0.AddCost(binding=binding)
        self.assertIsInstance(var, Variable)
        self.assertIsInstance(binding, Binding[Cost])
        binding = edge0.AddConstraint(f=(edge0.xu()[0] == edge0.xv()[0]))
        self.assertIsInstance(binding, Binding[Constraint])
        binding = edge0.AddConstraint(binding=binding)
        self.assertIsInstance(binding, Binding[Constraint])
        edge0.AddPhiConstraint(phi_value=False)
        edge0.ClearPhiConstraints()
