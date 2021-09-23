import unittest
import typing
from collections import namedtuple

import numpy as np

from pydrake.multibody.optimization import (
    CalcGridPointsOptions,
    CentroidalMomentumConstraint,
    QuaternionEulerIntegrationConstraint,
    StaticEquilibriumProblem,
    Toppra,
    ToppraDiscretization,
)
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    CoulombFriction,
    MultibodyPlant,
)
from pydrake.multibody.tree import (
    PrismaticJoint,
    UnitInertia,
    SpatialInertia,
)
import pydrake.multibody.inverse_kinematics as ik
import pydrake.solvers.mathematicalprogram as mp
from pydrake.solvers.snopt import SnoptSolver
from pydrake.systems.framework import DiagramBuilder_
from pydrake.geometry import (
    Box,
    Sphere,
)
from pydrake.math import RigidTransform
from pydrake.autodiffutils import AutoDiffXd
from pydrake.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial

Environment = namedtuple("Environment", [
    "plant", "scene_graph", "diagram", "boxes", "ground_geometry_id",
    "boxes_geometry_id"])


def construct_environment(masses: typing.List, box_sizes: typing.List):
    """
    Construct an environment with many free boxes.
    @param masses masses[i] is the mass of box i.
    @param box_sizes box_sizes[i] is the size of box i.
    """
    builder = DiagramBuilder_[AutoDiffXd]()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    # Add the ground as a big box.
    ground_box = plant.AddRigidBody(
        "ground", SpatialInertia(1, np.array([0, 0, 0]), UnitInertia(1, 1, 1)))
    X_WG = RigidTransform([0, 0, -0.05])
    ground_geometry_id = plant.RegisterCollisionGeometry(
        ground_box, RigidTransform(), Box(10, 10, 0.1), "ground",
        CoulombFriction(0.9, 0.8))
    plant.RegisterVisualGeometry(
        ground_box, RigidTransform(), Box(10, 10, 0.1), "ground",
        [0.5, 0.5, 0.5, 1.])
    plant.WeldFrames(plant.world_frame(), ground_box.body_frame(), X_WG)

    # Add boxes
    assert isinstance(masses, list)
    assert isinstance(box_sizes, list)
    assert len(masses) == len(box_sizes)
    num_boxes = len(masses)
    boxes = []
    boxes_geometry_id = []
    for i in range(num_boxes):
        box_name = f"box{i}"
        boxes.append(plant.AddRigidBody(
            box_name, SpatialInertia(
                masses[i], np.array([0, 0, 0]), UnitInertia(1, 1, 1))))
        box_shape = Box(box_sizes[i][0], box_sizes[i][1], box_sizes[i][2])
        boxes_geometry_id.append(plant.RegisterCollisionGeometry(
            boxes[i], RigidTransform(), box_shape, f"{box_name}_box",
            CoulombFriction(0.9, 0.8)))
        plant.RegisterVisualGeometry(
            ground_box, RigidTransform(), box_shape, f"{box_name}_box",
            [0.5, 0.5, 0.5, 1.])
        # Add small spheres at the corners of the box.
        vertices = np.array([
            [1, 1, 1], [1, 1, -1], [1, -1, 1], [1, -1, -1], [-1, 1, 1],
            [-1, -1, 1], [-1, 1, -1], [-1, -1, -1]]) *\
            np.tile(box_sizes[i]/2, (8, 1))
        sphere_shape = Sphere(0.001)
        for j in range(8):
            plant.RegisterCollisionGeometry(
                boxes[i], RigidTransform(vertices[j]), sphere_shape,
                f"{box_name}_sphere{j}", CoulombFriction(0.9, 0.8))
            plant.RegisterVisualGeometry(
                boxes[i], RigidTransform(vertices[j]), sphere_shape,
                f"{box_name}_sphere{j}", [0.5, 0.5, 0.5, 1])

    plant.Finalize()
    diagram = builder.Build()

    return Environment(
        plant=plant, scene_graph=scene_graph, diagram=diagram, boxes=boxes,
        ground_geometry_id=ground_geometry_id,
        boxes_geometry_id=boxes_geometry_id)


def split_se3(q_se3):
    """
    q_se3 is the quaternion/position of a floating body.
    """
    assert q_se3.shape == (7,)
    quat = q_se3[:4]
    xyz = q_se3[4:]
    return quat, xyz


class TestStaticEquilibriumProblem(unittest.TestCase):

    @unittest.skipUnless(SnoptSolver().available(), "Requires Snopt")
    def test_one_box(self):
        # Test with a single box.
        masses = [1.]
        box_sizes = [np.array([0.1, 0.1, 0.1])]
        env = construct_environment(masses, box_sizes)
        diagram_context = env.diagram.CreateDefaultContext()
        plant_context = env.diagram.GetMutableSubsystemContext(
            env.plant, diagram_context)
        # Ignore the collision between box-box, since currently Drake doesn't
        # support box-box collision with AutoDiffXd.
        ignored_collision_pairs = {(
            env.ground_geometry_id, env.boxes_geometry_id[0])}
        dut = StaticEquilibriumProblem(
            env.plant, plant_context, ignored_collision_pairs)

        # Add the constraint that the quaternion should have unit length.
        box_quat, box_xyz = split_se3(dut.q_vars())
        ik.AddUnitQuaternionConstraintOnPlant(
            env.plant, dut.q_vars(), dut.get_mutable_prog())

        dut.get_mutable_prog().SetInitialGuess(
            box_quat, np.array([0.5, 0.5, 0.5, 0.5]))
        dut.get_mutable_prog().SetInitialGuess(
            box_xyz, np.array([0, 0, 0.3]))

        # Now set the complementarity tolerance.
        dut.UpdateComplementarityTolerance(0.002)
        solver = SnoptSolver()
        result = solver.Solve(dut.prog())
        self.assertTrue(result.is_success())
        q_sol = result.GetSolution(dut.q_vars())
        box_quat_sol, box_xyz_sol = split_se3(q_sol)
        self.assertAlmostEqual(np.linalg.norm(box_quat_sol), 1.)
        np.testing.assert_allclose(box_xyz_sol[2], 0.05, atol=0.002)

        contact_wrenches = dut.GetContactWrenchSolution(result)
        self.assertEqual(len(contact_wrenches), 8)


class TestCentroidalMomentumConstraint(unittest.TestCase):
    def test_autodiff_constructor(self):
        # test with 2 boxes.
        masses = [1., 2]
        box_sizes = [np.array([0.1, 0.1, 0.1]), np.array([0.1, 0.1, 0.2])]
        env = construct_environment(masses, box_sizes)
        plant_context = env.plant.CreateDefaultContext()
        dut = CentroidalMomentumConstraint(
            plant=env.plant, model_instances=None, plant_context=plant_context,
            angular_only=False)
        self.assertIsInstance(dut, mp.Constraint)
        self.assertEqual(
            dut.num_vars(),
            env.plant.num_positions() + env.plant.num_velocities() + 6)
        self.assertEqual(dut.num_constraints(), 6)


class TestQuaternionEulerIntegrationConstraint(unittest.TestCase):
    def test(self):
        dut = QuaternionEulerIntegrationConstraint(
            allow_quaternion_negation=True)
        self.assertIsInstance(dut, mp.Constraint)


class TestToppra(unittest.TestCase):
    def test_gridpoints(self):
        path = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            [0., 1.], np.array([[0., -1.], [1, 1.1], [2, 3]]), np.zeros(3),
            np.zeros(3))

        options = CalcGridPointsOptions()
        self.assertEqual(options.max_err, 1e-3)
        options.max_err = 1e-2
        self.assertEqual(options.max_iter, 100)
        options.max_iter = 5
        self.assertEqual(options.max_seg_length, 0.05)
        options.max_seg_length = 0.1
        self.assertEqual(options.min_points, 100)
        options.min_points = 10

        grid_points = Toppra.CalcGridPoints(path=path, options=options)
        self.assertIsInstance(grid_points, np.ndarray)

    def test_all_constraints(self):
        plant = MultibodyPlant(0)
        file_path = FindResourceOrThrow(
            "drake/manipulation/models/iiwa_description/iiwa7/"
            "iiwa7_no_collision.sdf")
        iiwa_id = Parser(plant).AddModelFromFile(file_path, "iiwa")
        plant.WeldFrames(plant.world_frame(),
                         plant.GetFrameByName("iiwa_link_0", iiwa_id))
        plant.Finalize()

        knots = np.array([[-1.57, 0.1, 0, -1.2, 0, 1.6, 0],
                          [-0.8, -0.6, 0.5, 0.1, -0.5, -0.1, -1]]).T
        path = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            [0., 1.], knots, np.zeros(7), np.zeros(7))
        gridpoints = Toppra.CalcGridPoints(path, CalcGridPointsOptions())
        toppra = Toppra(path=path, plant=plant, gridpoints=gridpoints)

        # Joint constraints
        upper_limit = np.arange(7)
        lower_limit = -upper_limit
        frame = plant.GetFrameByName("iiwa_link_7")

        constraint = toppra.AddJointVelocityLimit(lower_limit=lower_limit,
                                                  upper_limit=upper_limit)
        self.assertIsInstance(constraint, mp.Binding[mp.BoundingBoxConstraint])
        backward_con, forward_con = toppra.AddJointAccelerationLimit(
            lower_limit=lower_limit, upper_limit=upper_limit,
            discretization=ToppraDiscretization.kCollocation)
        self.assertIsInstance(backward_con, mp.Binding[mp.LinearConstraint])
        self.assertIsInstance(forward_con, mp.Binding[mp.LinearConstraint])
        backward_con, forward_con = toppra.AddJointAccelerationLimit(
            lower_limit=lower_limit, upper_limit=upper_limit,
            discretization=ToppraDiscretization.kInterpolation)
        self.assertIsInstance(backward_con, mp.Binding[mp.LinearConstraint])
        self.assertIsInstance(forward_con, mp.Binding[mp.LinearConstraint])
        backward_con, forward_con = toppra.AddJointTorqueLimit(
            lower_limit=lower_limit, upper_limit=upper_limit,
            discretization=ToppraDiscretization.kCollocation)
        self.assertIsInstance(backward_con, mp.Binding[mp.LinearConstraint])
        self.assertIsInstance(forward_con, mp.Binding[mp.LinearConstraint])
        backward_con, forward_con = toppra.AddJointTorqueLimit(
            lower_limit=lower_limit, upper_limit=upper_limit,
            discretization=ToppraDiscretization.kInterpolation)
        self.assertIsInstance(backward_con, mp.Binding[mp.LinearConstraint])
        self.assertIsInstance(forward_con, mp.Binding[mp.LinearConstraint])

        # Frame constraints
        upper_limit = np.arange(6)
        lower_limit = -upper_limit
        frame = plant.GetFrameByName("iiwa_link_7")

        constraint = toppra.AddFrameVelocityLimit(constraint_frame=frame,
                                                  lower_limit=lower_limit,
                                                  upper_limit=upper_limit)
        self.assertIsInstance(constraint, mp.Binding[mp.BoundingBoxConstraint])
        constraint = toppra.AddFrameTranslationalSpeedLimit(
            constraint_frame=frame, upper_limit=1.)
        self.assertIsInstance(constraint, mp.Binding[mp.BoundingBoxConstraint])
        constraint = toppra.AddFrameAccelerationLimit(
            constraint_frame=frame, lower_limit=lower_limit,
            upper_limit=upper_limit,
            discretization=ToppraDiscretization.kCollocation)
        self.assertIsInstance(backward_con, mp.Binding[mp.LinearConstraint])
        self.assertIsInstance(forward_con, mp.Binding[mp.LinearConstraint])
        constraint = toppra.AddFrameAccelerationLimit(
            constraint_frame=frame, lower_limit=lower_limit,
            upper_limit=upper_limit,
            discretization=ToppraDiscretization.kInterpolation)
        self.assertIsInstance(backward_con, mp.Binding[mp.LinearConstraint])
        self.assertIsInstance(forward_con, mp.Binding[mp.LinearConstraint])

    def test_solve(self):
        plant = MultibodyPlant(0)
        M_AAo_A = SpatialInertia(1, np.zeros(3), UnitInertia(1, 1, 1))
        body = plant.AddRigidBody("body", M_AAo_A)
        plant.AddJoint(PrismaticJoint(
            "joint", plant.world_frame(), body.body_frame(), [1, 0, 0]))
        plant.Finalize()

        path = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            [0., 1.], np.array([[0., 1.]]), [0.], [0.])
        gridpoints = Toppra.CalcGridPoints(path, CalcGridPointsOptions())
        toppra = Toppra(path=path, plant=plant, gridpoints=gridpoints)

        toppra.AddJointAccelerationLimit([-1.], [1.])
        trajectory = toppra.SolvePathParameterization()
        self.assertIsInstance(trajectory, PiecewisePolynomial)
