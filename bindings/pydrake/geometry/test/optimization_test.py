import pydrake.geometry.optimization as mut

import os.path
import unittest

import numpy as np

from pydrake.common import RandomGenerator, temp_directory
from pydrake.common.test_utilities.pickle_compare import assert_pickle
from pydrake.geometry import (
    Box, Capsule, Cylinder, Ellipsoid, Convex, FramePoseVector, GeometryFrame,
    GeometryInstance, ProximityProperties, SceneGraph, Sphere, GeometryId
)
from pydrake.math import RigidTransform
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import BodyIndex
from pydrake.systems.framework import DiagramBuilder
from pydrake.solvers import (
    Binding, ClpSolver, Constraint, Cost, MathematicalProgram,
    MathematicalProgramResult, SolverOptions, CommonSolverOption,
    ScsSolver, MosekSolver
)
from pydrake.symbolic import Variable, Polynomial


class TestGeometryOptimization(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.A = np.eye(3)
        self.b = [1.0, 1.0, 1.0]
        self.prog = MathematicalProgram()
        self.x = self.prog.NewContinuousVariables(3, "x")
        self.t = self.prog.NewContinuousVariables(1, "t")[0]

        self.Ay = np.array([[1., 0.], [0., 1.], [1., 0.]])
        self.by = np.ones(3)
        self.cz = np.ones(2)
        self.dz = 1.
        self.y = self.prog.NewContinuousVariables(2, "y")
        self.z = self.prog.NewContinuousVariables(2, "z")

    def test_point_convex_set(self):
        mut.Point()
        p = np.array([11.1, 12.2, 13.3])
        point = mut.Point(p)
        self.assertFalse(point.IsEmpty())
        self.assertEqual(point.ambient_dimension(), 3)
        np.testing.assert_array_equal(point.x(), p)
        np.testing.assert_array_equal(point.MaybeGetPoint(), p)
        point.set_x(x=2*p)
        np.testing.assert_array_equal(point.x(), 2*p)
        point.set_x(x=p)
        assert_pickle(self, point, lambda S: S.x())

        # TODO(SeanCurtis-TRI): This doesn't test the constructor that
        # builds from shape.

    def test_h_polyhedron(self):
        mut.HPolyhedron()
        hpoly = mut.HPolyhedron(A=self.A, b=self.b)
        self.assertEqual(hpoly.ambient_dimension(), 3)
        np.testing.assert_array_equal(hpoly.A(), self.A)
        np.testing.assert_array_equal(hpoly.b(), self.b)
        self.assertTrue(hpoly.PointInSet(x=[0, 0, 0], tol=0.0))
        self.assertFalse(hpoly.IsEmpty())
        self.assertFalse(hpoly.IsBounded())
        new_vars, new_constraints = hpoly.AddPointInSetConstraints(
            self.prog, self.x)
        self.assertEqual(new_vars.size, 0)
        constraints = hpoly.AddPointInNonnegativeScalingConstraints(
            prog=self.prog, x=self.x, t=self.t)
        self.assertGreaterEqual(len(constraints), 2)
        self.assertIsInstance(constraints[0], Binding[Constraint])
        constraints = hpoly.AddPointInNonnegativeScalingConstraints(
            prog=self.prog, A=self.Ay, b=self.by, c=self.cz, d=self.dz,
            x=self.y, t=self.z)
        self.assertGreaterEqual(len(constraints), 2)
        self.assertIsInstance(constraints[0], Binding[Constraint])
        with self.assertRaisesRegex(
                RuntimeError, ".*not implemented yet for HPolyhedron.*"):
            hpoly.ToShapeWithPose()
        assert_pickle(self, hpoly, lambda S: np.vstack((S.A(), S.b())))

        h_box = mut.HPolyhedron.MakeBox(
            lb=[-1, -1, -1], ub=[1, 1, 1])
        self.assertTrue(h_box.IntersectsWith(hpoly))
        h_unit_box = mut.HPolyhedron.MakeUnitBox(dim=3)
        np.testing.assert_array_equal(h_box.A(), h_unit_box.A())
        np.testing.assert_array_equal(h_box.b(), h_unit_box.b())
        A_l1 = np.array([[1, 1, 1],
                         [-1, 1, 1],
                         [1, -1, 1],
                         [-1, -1, 1],
                         [1, 1, -1],
                         [-1, 1, -1],
                         [1, -1, -1],
                         [-1, -1, -1]])
        b_l1 = np.ones(8)
        h_l1_ball = mut.HPolyhedron.MakeL1Ball(dim=3)
        np.testing.assert_array_equal(A_l1, h_l1_ball.A())
        np.testing.assert_array_equal(b_l1, h_l1_ball.b())
        self.assertIsInstance(
            h_box.MaximumVolumeInscribedEllipsoid(),
            mut.Hyperellipsoid)
        np.testing.assert_array_almost_equal(
            h_box.ChebyshevCenter(), [0, 0, 0])
        h_scaled = h_box.Scale(scale=2.0, center=[1, 2, 3])
        self.assertIsInstance(h_scaled, mut.HPolyhedron)
        h_scaled = h_box.Scale(scale=2.0)
        self.assertIsInstance(h_scaled, mut.HPolyhedron)
        h2 = h_box.CartesianProduct(other=h_unit_box)
        self.assertIsInstance(h2, mut.HPolyhedron)
        self.assertEqual(h2.ambient_dimension(), 6)
        h3 = h_box.CartesianPower(n=3)
        self.assertIsInstance(h3, mut.HPolyhedron)
        self.assertEqual(h3.ambient_dimension(), 9)
        h4 = h_box.Intersection(other=h_unit_box)
        self.assertIsInstance(h4, mut.HPolyhedron)
        self.assertEqual(h4.ambient_dimension(), 3)
        h5 = h_box.PontryaginDifference(other=h_unit_box)
        self.assertIsInstance(h5, mut.HPolyhedron)
        np.testing.assert_array_equal(h5.A(), h_box.A())
        np.testing.assert_array_equal(h5.b(), np.zeros(6))

        generator = RandomGenerator()
        sample = h_box.UniformSample(generator=generator)
        self.assertEqual(sample.shape, (3,))
        self.assertEqual(
            h_box.UniformSample(generator=generator,
                                previous_sample=sample).shape, (3, ))

        h_half_box = mut.HPolyhedron.MakeBox(
            lb=[-0.5, -0.5, -0.5], ub=[0.5, 0.5, 0.5])
        self.assertTrue(h_half_box.ContainedIn
                        (other=h_unit_box, tol=1E-9))
        h_half_box2 = h_half_box.Intersection(
            other=h_unit_box, check_for_redundancy=True, tol=1E-9)
        self.assertIsInstance(h_half_box2, mut.HPolyhedron)
        self.assertEqual(h_half_box2.ambient_dimension(), 3)
        np.testing.assert_array_almost_equal(
            h_half_box2.A(), h_half_box.A())
        np.testing.assert_array_almost_equal(
            h_half_box2.b(), h_half_box.b())

        # Intersection of 1/2*unit_box and unit_box and reducing the redundant
        # inequalities should result in the 1/2*unit_box.
        h_half_box_intersect_unit_box = h_half_box.Intersection(
            other=h_unit_box,
            check_for_redundancy=False)
        # Check that the ReduceInequalities binding works.
        redundant_indices = h_half_box_intersect_unit_box.FindRedundant(
            tol=1E-9)
        # Check FindRedundant with default tol.
        h_half_box_intersect_unit_box.FindRedundant()
        h_half_box3 = h_half_box_intersect_unit_box.ReduceInequalities(
            tol=1E-9)

        # This polyhedron is intentionally constructed to be an empty set.
        A_empty = np.vstack([np.eye(3), -np.eye(3)])
        b_empty = -np.ones(6)
        h_empty = mut.HPolyhedron(A_empty, b_empty)
        self.assertTrue(h_empty.IsEmpty())
        self.assertFalse(h_l1_ball.IsEmpty())

        vpoly = mut.VPolytope(np.array(
            [[0., 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))
        hpoly = mut.HPolyhedron(vpoly=vpoly)
        self.assertEqual(hpoly.ambient_dimension(), 3)
        self.assertEqual(hpoly.A().shape, (4, 3))

    def test_hyper_ellipsoid(self):
        mut.Hyperellipsoid()
        ellipsoid = mut.Hyperellipsoid(A=self.A, center=self.b)
        self.assertEqual(ellipsoid.ambient_dimension(), 3)
        self.assertFalse(ellipsoid.IsEmpty())
        np.testing.assert_array_equal(ellipsoid.A(), self.A)
        np.testing.assert_array_equal(ellipsoid.center(), self.b)
        self.assertTrue(ellipsoid.PointInSet(x=self.b, tol=0.0))
        new_vars, new_constraints = ellipsoid.AddPointInSetConstraints(
            self.prog, self.x)
        self.assertEqual(new_vars.size, 0)
        self.assertGreater(len(new_constraints), 0)
        constraints = ellipsoid.AddPointInNonnegativeScalingConstraints(
            prog=self.prog, x=self.x, t=self.t)
        self.assertGreaterEqual(len(constraints), 2)
        self.assertIsInstance(constraints[0], Binding[Constraint])
        constraints = ellipsoid.AddPointInNonnegativeScalingConstraints(
            prog=self.prog, A=self.Ay, b=self.by, c=self.cz, d=self.dz,
            x=self.y, t=self.z)
        self.assertGreaterEqual(len(constraints), 2)
        self.assertIsInstance(constraints[0], Binding[Constraint])
        shape, pose = ellipsoid.ToShapeWithPose()
        self.assertIsInstance(shape, Ellipsoid)
        self.assertIsInstance(pose, RigidTransform)
        p = np.array([11.1, 12.2, 13.3])
        point = mut.Point(p)
        scale, witness = ellipsoid.MinimumUniformScalingToTouch(point)
        self.assertTrue(scale > 0.0)
        np.testing.assert_array_almost_equal(witness, p)
        assert_pickle(self, ellipsoid,
                      lambda S: np.vstack((S.A(), S.center())))
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
        mut.MinkowskiSum()
        point = mut.Point(np.array([11.1, 12.2, 13.3]))
        hpoly = mut.HPolyhedron(A=self.A, b=self.b)
        sum = mut.MinkowskiSum(setA=point, setB=hpoly)
        self.assertEqual(sum.ambient_dimension(), 3)
        self.assertEqual(sum.num_terms(), 2)
        sum2 = mut.MinkowskiSum(sets=[point, hpoly])
        self.assertEqual(sum2.ambient_dimension(), 3)
        self.assertEqual(sum2.num_terms(), 2)
        self.assertIsInstance(sum2.term(0), mut.Point)
        self.assertFalse(sum.IsEmpty())

    def test_spectrahedron(self):
        s = mut.Spectrahedron()
        prog = MathematicalProgram()
        X = prog.NewSymmetricContinuousVariables(3)
        prog.AddPositiveSemidefiniteConstraint(X)
        prog.AddLinearEqualityConstraint(X[0, 0] + X[1, 1] + X[2, 2], 1)
        s = mut.Spectrahedron(prog=prog)
        self.assertEqual(s.ambient_dimension(), 6)

    def test_v_polytope(self):
        mut.VPolytope()
        vertices = np.array([[0.0, 1.0, 2.0], [3.0, 7.0, 5.0]])
        vpoly = mut.VPolytope(vertices=vertices)
        self.assertFalse(vpoly.IsEmpty())
        self.assertEqual(vpoly.ambient_dimension(), 2)
        np.testing.assert_array_equal(vpoly.vertices(), vertices)
        self.assertTrue(vpoly.PointInSet(x=[1.0, 5.0], tol=1e-8))
        new_vars, new_constraints = vpoly.AddPointInSetConstraints(
            self.prog, self.x[0:2])
        self.assertEqual(new_vars.size, vertices.shape[1])
        constraints = vpoly.AddPointInNonnegativeScalingConstraints(
            prog=self.prog, x=self.x[:2], t=self.t)
        self.assertGreaterEqual(len(constraints), 2)
        self.assertIsInstance(constraints[0], Binding[Constraint])
        constraints = vpoly.AddPointInNonnegativeScalingConstraints(
            prog=self.prog, A=self.Ay[:2], b=self.by[:2], c=self.cz, d=self.dz,
            x=self.y, t=self.z)
        self.assertGreaterEqual(len(constraints), 2)
        self.assertIsInstance(constraints[0], Binding[Constraint])
        assert_pickle(self, vpoly, lambda S: S.vertices())
        v_box = mut.VPolytope.MakeBox(
            lb=[-1, -1, -1], ub=[1, 1, 1])
        self.assertTrue(v_box.PointInSet([0, 0, 0]))
        self.assertAlmostEqual(v_box.CalcVolume(), 8, 1E-10)
        v_unit_box = mut.VPolytope.MakeUnitBox(dim=3)
        self.assertTrue(v_unit_box.PointInSet([0, 0, 0]))
        v_from_h = mut.VPolytope(H=mut.HPolyhedron.MakeUnitBox(dim=3))
        self.assertTrue(v_from_h.PointInSet([0, 0, 0]))
        # Test creating a vpolytope from a non-minimal set of vertices
        # 2D: Random points inside a circle
        r = 2.0
        n = 400
        vertices = np.zeros((2, n + 4))
        theta = np.linspace(0, 2 * np.pi, n, endpoint=False)
        vertices[0, 0:n] = r * np.cos(theta)
        vertices[1, 0:n] = r * np.sin(theta)
        vertices[:, n:] = np.array([
            [r/2, r/3, r/4, r/5],
            [r/2, r/3, r/4, r/5]
        ])

        vpoly = mut.VPolytope(vertices=vertices).GetMinimalRepresentation()
        self.assertAlmostEqual(vpoly.CalcVolume(), np.pi * r * r, delta=1e-3)
        self.assertEqual(vpoly.vertices().shape[1], n)
        # Calculate the length of the path that visits all the vertices
        # sequentially.
        # If the vertices are in clockwise/counter-clockwise order,
        # the length of the path will coincide with the perimeter of a
        # circle.
        self.assertAlmostEqual(self._calculate_path_length(vpoly.vertices()),
                               2 * np.pi * r, delta=1e-3)
        # 3D: Random points inside a box
        a = 2.0
        vertices = np.array([
            [0, a, 0, a, 0, a, 0, a, a/2, a/3, a/4, a/5],
            [0, 0, a, a, 0, 0, a, a, a/2, a/3, a/4, a/5],
            [0, 0, 0, 0, a, a, a, a, a/2, a/3, a/4, a/5]
        ])
        vpoly = mut.VPolytope(vertices=vertices).GetMinimalRepresentation()
        self.assertAlmostEqual(vpoly.CalcVolume(), a * a * a)
        self.assertEqual(vpoly.vertices().shape[1], 8)
        temp_file_name = f"{temp_directory()}/vpoly.obj"
        vpoly.WriteObj(filename=temp_file_name)
        self.assertTrue(os.path.isfile(temp_file_name))

    def _calculate_path_length(self, vertices):
        n = vertices.shape[1]
        length = 0

        for i in range(n):
            j = (i + 1) % n
            diff = vertices[:, i] - vertices[:, j]
            length += np.sqrt(np.dot(diff, diff))

        return length

    def test_cartesian_product(self):
        mut.CartesianProduct()
        point = mut.Point(np.array([11.1, 12.2, 13.3]))
        h_box = mut.HPolyhedron.MakeBox(
            lb=[-1, -1, -1], ub=[1, 1, 1])
        sum = mut.CartesianProduct(setA=point, setB=h_box)
        self.assertFalse(sum.IsEmpty())
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

    def test_intersection(self):
        mut.Intersection()
        point = mut.Point(np.array([0.1, 0.2, 0.3]))
        h_box = mut.HPolyhedron.MakeBox(
            lb=[-1, -1, -1], ub=[1, 1, 1])
        intersect = mut.Intersection(setA=point, setB=h_box)
        self.assertFalse(intersect.IsEmpty())
        self.assertEqual(intersect.ambient_dimension(), 3)
        self.assertEqual(intersect.num_elements(), 2)
        intersect2 = mut.Intersection(sets=[point, h_box])
        self.assertEqual(intersect2.ambient_dimension(), 3)
        self.assertEqual(intersect2.num_elements(), 2)
        self.assertIsInstance(intersect2.element(0), mut.Point)

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
        for geometry_id in [box_geometry_id, cylinder_geometry_id,
                            sphere_geometry_id, capsule_geometry_id]:
            scene_graph.AssignRole(source_id, geometry_id,
                                   properties=ProximityProperties())
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
        self.assertGreater(len(obstacles), 0)
        for obstacle in obstacles:
            self.assertIsInstance(obstacle, mut.ConvexSet)
        options = mut.IrisOptions()
        options.require_sample_point_is_contained = True
        options.iteration_limit = 1
        options.termination_threshold = 0.1
        options.relative_termination_threshold = 0.01
        options.random_seed = 1314
        options.starting_ellipse = mut.Hyperellipsoid.MakeUnitBall(3)
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
        Parser(plant).AddModelsFromString(limits_urdf, "urdf")
        plant.Finalize()
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        options = mut.IrisOptions()
        options.num_collision_infeasible_samples = 3
        ik = InverseKinematics(plant)
        options.prog_with_additional_constraints = ik.prog()
        options.num_additional_constraint_infeasible_samples = 2
        plant.SetPositions(plant.GetMyMutableContextFromRoot(context), [0])
        region = mut.IrisInConfigurationSpace(
            plant=plant, context=plant.GetMyContextFromRoot(context),
            options=options)
        self.assertIsInstance(region, mut.ConvexSet)
        self.assertEqual(region.ambient_dimension(), 1)
        self.assertTrue(region.PointInSet([1.0]))
        self.assertFalse(region.PointInSet([3.0]))
        options.configuration_obstacles = [mut.Point([-0.5])]
        point, = options.configuration_obstacles
        self.assertEqual(point.x(), [-0.5])
        point2, = options.configuration_obstacles
        self.assertIs(point2, point)
        region = mut.IrisInConfigurationSpace(
            plant=plant, context=plant.GetMyContextFromRoot(context),
            options=options)
        self.assertIsInstance(region, mut.ConvexSet)
        self.assertEqual(region.ambient_dimension(), 1)
        self.assertTrue(region.PointInSet([1.0]))
        self.assertFalse(region.PointInSet([-1.0]))

    def test_serialize_iris_regions(self):
        iris_regions = {
            "box1":
            mut.HPolyhedron.MakeBox(lb=[-1, -2, -3], ub=[1, 2, 3]),
            "box2":
            mut.HPolyhedron.MakeBox(lb=[-4.1, -5.2, -6.3], ub=[4.1, 4.2, 6.3])
        }
        temp_file_name = f"{temp_directory()}/iris.yaml"
        mut.SaveIrisRegionsYamlFile(filename=temp_file_name,
                                    regions=iris_regions,
                                    child_name="test")
        loaded_regions = mut.LoadIrisRegionsYamlFile(filename=temp_file_name,
                                                     child_name="test")
        self.assertEqual(iris_regions.keys(), loaded_regions.keys())
        for k in iris_regions.keys():
            np.testing.assert_array_equal(iris_regions[k].A(),
                                          loaded_regions[k].A())
            np.testing.assert_array_equal(iris_regions[k].b(),
                                          loaded_regions[k].b())

    def test_graph_of_convex_sets(self):
        options = mut.GraphOfConvexSetsOptions()
        self.assertIsNone(options.convex_relaxation)
        self.assertIsNone(options.preprocessing)
        self.assertIsNone(options.max_rounded_paths)
        options.convex_relaxation = True
        options.preprocessing = False
        options.max_rounded_paths = 2
        options.max_rounding_trials = 5
        options.flow_tolerance = 1e-6
        options.rounding_seed = 1
        options.solver = ClpSolver()
        options.solver_options = SolverOptions()
        options.solver_options.SetOption(ClpSolver.id(), "scaling", 2)
        options.rounding_solver_options = SolverOptions()
        options.rounding_solver_options.SetOption(ClpSolver.id(), "dual", 0)
        self.assertIn("scaling",
                      options.solver_options.GetOptions(ClpSolver.id()))
        self.assertIn(
            "dual", options.rounding_solver_options.GetOptions(ClpSolver.id()))
        self.assertIn("convex_relaxation", repr(options))

        spp = mut.GraphOfConvexSets()
        source = spp.AddVertex(set=mut.Point([0.1]), name="source")
        target = spp.AddVertex(set=mut.Point([0.2]), name="target")
        edge0 = spp.AddEdge(u=source, v=target, name="edge0")
        edge1 = spp.AddEdge(u_id=source.id(), v_id=target.id(), name="edge1")
        self.assertEqual(len(spp.Vertices()), 2)
        self.assertEqual(len(spp.Edges()), 2)
        result = spp.SolveShortestPath(
            source_id=source.id(), target_id=target.id(), options=options)
        self.assertIsInstance(result, MathematicalProgramResult)
        self.assertIsInstance(spp.SolveShortestPath(
            source=source, target=target, options=options),
            MathematicalProgramResult)

        self.assertIn("source", spp.GetGraphvizString(
            result=result, show_slacks=True, precision=2, scientific=False))

        # Vertex
        self.assertAlmostEqual(
            source.GetSolutionCost(result=result), 0.0, 1e-6)
        np.testing.assert_array_almost_equal(
            source.GetSolution(result), [0.1], 1e-6)
        self.assertIsInstance(source.id(), mut.GraphOfConvexSets.VertexId)
        self.assertEqual(source.ambient_dimension(), 1)
        self.assertEqual(source.name(), "source")
        self.assertIsInstance(source.x()[0], Variable)
        self.assertIsInstance(source.set(), mut.Point)
        var, binding = source.AddCost(e=1.0+source.x()[0])
        self.assertIsInstance(var, Variable)
        self.assertIsInstance(binding, Binding[Cost])
        var, binding = source.AddCost(binding=binding)
        self.assertIsInstance(var, Variable)
        self.assertIsInstance(binding, Binding[Cost])
        self.assertEqual(len(source.GetCosts()), 2)
        binding = source.AddConstraint(f=(source.x()[0] <= 1.0))
        self.assertIsInstance(binding, Binding[Constraint])
        binding = source.AddConstraint(binding=binding)
        self.assertIsInstance(binding, Binding[Constraint])
        self.assertEqual(len(source.GetConstraints()), 2)

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
        self.assertEqual(len(edge0.GetCosts()), 2)
        binding = edge0.AddConstraint(f=(edge0.xu()[0] == edge0.xv()[0]))
        self.assertIsInstance(binding, Binding[Constraint])
        binding = edge0.AddConstraint(binding=binding)
        self.assertIsInstance(binding, Binding[Constraint])
        self.assertEqual(len(edge0.GetConstraints()), 2)
        edge0.AddPhiConstraint(phi_value=False)
        edge0.ClearPhiConstraints()
        edge1.AddPhiConstraint(phi_value=True)
        spp.ClearAllPhiConstraints()

        # Remove Edges
        self.assertEqual(len(spp.Edges()), 2)
        spp.RemoveEdge(edge1.id())
        self.assertEqual(len(spp.Edges()), 1)
        spp.RemoveEdge(edge0)
        self.assertEqual(len(spp.Edges()), 0)

        # Remove Vertices
        self.assertEqual(len(spp.Vertices()), 2)
        spp.RemoveVertex(source.id())
        self.assertEqual(len(spp.Vertices()), 1)
        spp.RemoveVertex(target)
        self.assertEqual(len(spp.Vertices()), 0)


class TestCspaceFreePolytope(unittest.TestCase):
    def setUp(self):
        unittest.TestCase.setUp(self)
        limits_urdf = """
                 <robot name="limits">
                   <link name="movable">
                     <collision>
                       <geometry><box size="0.1 0.1 0.1"/></geometry>
                     </collision>
                      <geometry>
                          <cylinder length="0.1" radius="0.2"/>
                     </geometry>
                     <geometry>
                          <capsule length="0.1" radius="0.2"/>
                     </geometry>
                     <geometry>
                          <sphere radius="0.2"/>
                     </geometry>
                   </link>
                   <link name="unmovable">
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
                   <joint name="unmovable" type = "fixed">
                         <parent link="world"/>
                         <child link="unmovable"/>
                         <origin xyz="1 0 0"/>
                   </joint>
                 </robot>"""

        builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            builder, 0.01)
        Parser(self.plant).AddModelsFromString(limits_urdf, "urdf")

        self.plant.Finalize()

        diagram = builder.Build()

        # Tests the constructor
        options = mut.CspaceFreePolytope.Options()
        options.with_cross_y = False
        self.cspace_free_polytope = mut.CspaceFreePolytope(
            plant=self.plant,
            scene_graph=self.scene_graph,
            plane_order=mut.SeparatingPlaneOrder.kAffine,
            q_star=np.zeros(self.plant.num_positions()),
            options = options)

    # def test_CollisionGeometry(self):
    #     # Check that the plane sides are properly enumerated.
    #     plane_side_possible_values = [mut.PlaneSide.kPositive,
    #                                   mut.PlaneSide.kNegative]
    #
    #     # Check that the CIrisCollisionGeometry types are properly enumerated.
    #     collision_geometries = mut.GetCollisionGeometries(
    #         plant=self.plant, scene_graph=self.scene_graph)
    #
    #     geom_type_possible_values = [
    #         mut.GeometryType.kPolytope,
    #         mut.GeometryType.kSphere,
    #         mut.GeometryType.kCylinder,
    #         mut.GeometryType.kCapsule]
    #     geom_shape_possible_values = [
    #         Capsule, Sphere, Cylinder, Box, Convex
    #     ]
    #
    #     for geom_lst in collision_geometries.values():
    #         for geom in geom_lst:
    #             self.assertIn(geom.type(), geom_type_possible_values)
    #             self.assertIn(
    #                 type(
    #                     geom.geometry()),
    #                 geom_shape_possible_values)
    #             self.assertIn(geom.body_index(), collision_geometries.keys())
    #             self.assertGreater(geom.num_rationals(), 0)
    #             self.assertIsInstance(geom.X_BG(), RigidTransform)
    #             self.assertIsInstance(geom.id(), GeometryId)

    def test_CspaceFreePolytope_Options(self):
        dut = mut.CspaceFreePolytope

        solver_options = SolverOptions()
        solver_options.SetOption(CommonSolverOption.kPrintToConsole, 1)

        # FindSeparationCertificateGivenPolytopeOptions
        lagrangian_options = \
            dut.\
                FindSeparationCertificateGivenPolytopeOptions()
        self.assertEqual(
            lagrangian_options.num_threads, -1)
        self.assertFalse(
            lagrangian_options.verbose)
        self.assertEqual(
            lagrangian_options.solver_id,
            MosekSolver.id())
        self.assertTrue(
            lagrangian_options.terminate_at_failure)
        self.assertIsNone(
            lagrangian_options.solver_options)
        self.assertFalse(
            lagrangian_options.ignore_redundant_C)
        num_threads = 1
        lagrangian_options.num_threads = num_threads
        lagrangian_options.verbose = True
        lagrangian_options.solver_id = ScsSolver.id()
        lagrangian_options.terminate_at_failure = False
        lagrangian_options.solver_options = solver_options
        lagrangian_options.ignore_redundant_C = True
        self.assertEqual(
            lagrangian_options.num_threads,
            num_threads)
        self.assertTrue(
            lagrangian_options.verbose)
        self.assertEqual(
            lagrangian_options.solver_id,
            ScsSolver.id())
        self.assertFalse(
            lagrangian_options.terminate_at_failure)
        self.assertEqual(
            lagrangian_options.solver_options.common_solver_options()[
                CommonSolverOption.kPrintToConsole], 1)
        self.assertTrue(
            lagrangian_options.ignore_redundant_C)

        # EllipsoidMarginCost
        margin_cost = [dut.EllipsoidMarginCost.kGeometricMean,
                       dut.EllipsoidMarginCost.kSum]

        # FindPolytopeGivenLagrangianOptions
        polytope_options = dut.FindPolytopeGivenLagrangianOptions()
        self.assertIsNone(polytope_options.backoff_scale)
        self.assertEqual(
            polytope_options.ellipsoid_margin_epsilon, 1e-5)
        self.assertEqual(
            polytope_options.solver_id,
            MosekSolver.id())
        self.assertIsNone(polytope_options.solver_options)
        self.assertIsNone(polytope_options.s_inner_pts)
        self.assertTrue(
            polytope_options.search_s_bounds_lagrangians)
        self.assertEqual(
            polytope_options.ellipsoid_margin_cost,
            dut.EllipsoidMarginCost.kGeometricMean)
        polytope_options.backoff_scale = 1e-3
        polytope_options.ellipsoid_margin_epsilon = 1e-6
        polytope_options.solver_id = ScsSolver.id()
        polytope_options.solver_options = solver_options
        polytope_options.s_inner_pts = np.zeros((2, 1))
        polytope_options.search_s_bounds_lagrangians = False
        polytope_options.ellipsoid_margin_cost = dut.EllipsoidMarginCost.kSum
        self.assertEqual(
            polytope_options.backoff_scale, 1e-3)
        self.assertEqual(
            polytope_options.ellipsoid_margin_epsilon, 1e-6)
        self.assertEqual(
            polytope_options.solver_id,
            ScsSolver.id())
        self.assertEqual(
            polytope_options.solver_options.common_solver_options()[
                CommonSolverOption.kPrintToConsole], 1)
        np.testing.assert_array_almost_equal(
            polytope_options.s_inner_pts, np.zeros(
                (2, 1)), 1e-5)
        self.assertFalse(
            polytope_options.search_s_bounds_lagrangians)
        self.assertEqual(
            polytope_options.ellipsoid_margin_cost,
            dut.EllipsoidMarginCost.kSum)

        # BilinearAlternationOptions
        bilinear_alternation_options = dut.BilinearAlternationOptions()
        self.assertEqual(bilinear_alternation_options.max_iter, 10)
        self.assertAlmostEqual(bilinear_alternation_options.convergence_tol,
                               1e-3)
        self.assertAlmostEqual(bilinear_alternation_options.ellipsoid_scaling,
                               0.99)
        self.assertTrue(bilinear_alternation_options.
                        find_polytope_options.search_s_bounds_lagrangians)
        self.assertFalse(
            bilinear_alternation_options.find_lagrangian_options.verbose)
        bilinear_alternation_options.max_iter = 4
        bilinear_alternation_options.convergence_tol = 1e-2
        bilinear_alternation_options.find_polytope_options = polytope_options
        bilinear_alternation_options.find_lagrangian_options = \
            lagrangian_options
        bilinear_alternation_options.ellipsoid_scaling = 0.5
        self.assertEqual(bilinear_alternation_options.max_iter, 4)
        self.assertAlmostEqual(bilinear_alternation_options.convergence_tol,
                               1e-2)
        self.assertAlmostEqual(bilinear_alternation_options.ellipsoid_scaling,
                               0.5)
        self.assertFalse(bilinear_alternation_options.
                         find_polytope_options.search_s_bounds_lagrangians)
        self.assertTrue(bilinear_alternation_options.
                        find_lagrangian_options.verbose)

        # BinarySearchOptions
        binary_search_options = dut.BinarySearchOptions()
        self.assertAlmostEqual(binary_search_options.scale_max, 1)
        self.assertAlmostEqual(binary_search_options.scale_min, 0.01)
        self.assertEqual(binary_search_options.max_iter, 10)
        self.assertAlmostEqual(
            binary_search_options.convergence_tol, 1e-3)
        self.assertFalse(
            binary_search_options.find_lagrangian_options.verbose)

        binary_search_options.scale_max = 2
        binary_search_options.scale_min = 1
        binary_search_options.max_iter = 2
        binary_search_options.convergence_tol = 1e-5
        binary_search_options.find_lagrangian_options = lagrangian_options
        self.assertAlmostEqual(binary_search_options.scale_max, 2)
        self.assertAlmostEqual(binary_search_options.scale_min, 1)
        self.assertEqual(binary_search_options.max_iter, 2)
        self.assertAlmostEqual(
            binary_search_options.convergence_tol, 1e-5)
        self.assertTrue(
            binary_search_options.find_lagrangian_options.verbose)

        options = dut.Options()
        self.assertFalse(options.with_cross_y)
        options.with_cross_y = True
        self.assertTrue(options.with_cross_y)

    def test_CspaceFreePolytope_constructor_and_getters(self):
        dut = self.cspace_free_polytope
        # rat_forward = dut.rational_forward_kin()
        # self.assertEqual(
        #     rat_forward.ComputeSValue(
        #         np.zeros(self.plant.num_positions()),
        #         np.zeros(self.plant.num_positions())),
        #     np.zeros(self.plant.num_positions()))
        self.assertGreaterEqual(
            len(dut.map_geometries_to_separating_planes().keys()), 1)
        self.assertGreaterEqual(
            len(dut.separating_planes()), 1)
        self.assertEqual(len(dut.y_slack()), 3)

        # Test SeparatingPlane and CIrisCollisionGeometry
        plane_side_possible_values = [mut.PlaneSide.kPositive,
                                      mut.PlaneSide.kNegative]
        geom_type_possible_values = [
            mut.CIrisGeometryType.kPolytope,
            mut.CIrisGeometryType.kSphere,
            mut.CIrisGeometryType.kCylinder,
            mut.CIrisGeometryType.kCapsule]
        geom_shape_possible_values = [
            Capsule, Sphere, Cylinder, Box, Convex
        ]
        possible_orders = [mut.SeparatingPlaneOrder.kAffine]
        for plane_idx in dut.map_geometries_to_separating_planes().values():
            plane = dut.separating_planes()[plane_idx]
            self.assertIsInstance(plane.a[0], Polynomial)
            self.assertIsInstance(plane.b, Polynomial)
            self.assertIsInstance(plane.expressed_body, BodyIndex)
            self.assertIn(plane.plane_order, possible_orders)
            self.assertIsInstance(plane.decision_variables[0], Variable)
            for geom in [plane.positive_side_geometry, plane.negative_side_geometry]:
                self.assertIn(geom.type(), geom_type_possible_values)
                self.assertIn(
                    type(
                        geom.geometry()),
                    geom_shape_possible_values)

                self.assertIsInstance(geom.body_index(), BodyIndex)
                self.assertGreater(geom.num_rationals(), 0)
                self.assertIsInstance(geom.X_BG(), RigidTransform)
                self.assertIsInstance(geom.id(), GeometryId)

    def test_CspaceFreePolytopeMethods(self):
        C_init = np.vstack([np.atleast_2d(np.eye(self.plant.num_positions(
        ))), -np.atleast_2d(np.eye(self.plant.num_positions()))])
        d_init = 3 * np.ones((C_init.shape[0], 1))

        bilinear_alternation_options = mut.CspaceFreePolytope.BilinearAlternationOptions()
        binary_search_options = mut.CspaceFreePolytope.BinarySearchOptions()
        binary_search_options.scale_min = 1e-4
        bilinear_alternation_options.find_lagrangian_options.verbose = False
        binary_search_options.find_lagrangian_options.verbose = False

        result = self.cspace_free_polytope.BinarySearch(
            ignored_collision_pairs=set(),
            C=C_init,
            d=d_init,
            s_center=np.zeros(self.plant.num_positions()),
            options=binary_search_options
        )
        # Accesses all members of SearchResult
        self.assertGreaterEqual(result.num_iter(), 1)
        self.assertGreaterEqual(len(result.C()), 1)
        self.assertGreaterEqual(len(result.d()), 1)
        self.assertIsInstance(result.certified_polytope(), mut.HPolyhedron)
        self.assertEqual(len(result.a()), 1)
        self.assertEqual(len(result.b()), 1)
        self.assertIsInstance(result.a()[0][0], Polynomial)

        result = self.cspace_free_polytope.SearchWithBilinearAlternation(
            ignored_collision_pairs=set(),
            C_init=C_init,
            d_init=d_init,
            options=bilinear_alternation_options)
        self.assertIsNotNone(result)

    def testSeparationCertificateMethods(self):
        C_init = np.vstack([np.atleast_2d(np.eye(self.plant.num_positions(
        ))), -np.atleast_2d(np.eye(self.plant.num_positions()))])
        d_init = 1e-10 * np.ones((C_init.shape[0], 1))
        pair = list(self.cspace_free_polytope.map_geometries_to_separating_planes().keys())[0]
        lagrangian_options = \
            mut.CspaceFreePolytope. \
                FindSeparationCertificateGivenPolytopeOptions()
        num_threads = 1
        lagrangian_options.num_threads = num_threads
        lagrangian_options.solver_id = ScsSolver.id()


        cert_prog = self.cspace_free_polytope.MakeIsGeometrySeparableProgram(
            geometry_pair=pair, C=C_init, d=d_init
        )
        # Call all CspaceFreePolytope.SeparationCertificateProgram methods
        certificates = cert_prog.certificate
        self.assertIsInstance(certificates, mut.CspaceFreePolytope.SeparationCertificate)
        self.assertIsInstance(cert_prog.prog(), MathematicalProgram)
        self.assertGreaterEqual(cert_prog.plane_index, 0)

        self.assertIsInstance(certificates.positive_side_rational_lagrangians[0],
                              mut.CspaceFreePolytope.SeparatingPlaneLagrangians)
        self.assertIsInstance(certificates.negative_side_rational_lagrangians[0],
                              mut.CspaceFreePolytope.SeparatingPlaneLagrangians)

        cert_prog_sol = self.cspace_free_polytope.SolveSeparationCertificateProgram(
            certificate_program=cert_prog, options=lagrangian_options
        )

        # Call all CspaceFreePolytope.SeparationCertificateProgramResult methods
        self.assertIsInstance(cert_prog_sol.a[0], Polynomial)
        self.assertIsInstance(cert_prog_sol.b, Polynomial)
        self.assertIsInstance(cert_prog_sol.plane_decision_var_vals[0], float)
        self.assertIsInstance(cert_prog_sol.result, MathematicalProgramResult)