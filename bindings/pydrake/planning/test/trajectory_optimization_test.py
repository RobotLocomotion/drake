import unittest
import weakref

import numpy as np

from pydrake.examples import PendulumPlant
from pydrake.geometry.optimization import (
    GcsGraphvizOptions,
    GraphOfConvexSets,
    GraphOfConvexSetsOptions,
    HPolyhedron,
    Point,
    VPolytope,
)
from pydrake.math import BsplineBasis, eq
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.planning import (
    AddDirectCollocationConstraint,
    DirectCollocation,
    DirectCollocationConstraint,
    DirectTranscription,
    GcsTrajectoryOptimization,
    GetContinuousRevoluteJointIndices,
    KinematicTrajectoryOptimization,
)
import pydrake.solvers as mp
from pydrake.solvers import (
    Binding,
    Constraint,
    Cost,
    ExpressionConstraint,
    ExpressionCost,
)
from pydrake.symbolic import Variable
from pydrake.systems.framework import InputPortSelection
from pydrake.systems.primitives import LinearSystem
from pydrake.trajectories import (
    BsplineTrajectory,
    CompositeTrajectory,
    PiecewisePolynomial,
)


class TestTrajectoryOptimization(unittest.TestCase):
    def test_direct_collocation(self):
        plant = PendulumPlant()
        context = plant.CreateDefaultContext()

        num_time_samples = 21
        dircol = DirectCollocation(
            plant,
            context,
            num_time_samples=num_time_samples,
            minimum_time_step=0.2,
            maximum_time_step=0.5,
            input_port_index=InputPortSelection.kUseFirstInputIfItExists,
            assume_non_continuous_states_are_fixed=False,
        )
        prog = dircol.prog()
        num_initial_vars = prog.num_vars()

        # Spell out most of the methods, regardless of whether they make sense
        # as a consistent optimization.  The goal is to check the bindings,
        # not the implementation.
        dircol.time()
        dircol.time_step(index=0)
        x = dircol.state()
        dircol.state(index=2)
        dircol.initial_state()
        dircol.final_state()
        u = dircol.input()
        dircol.input(index=2)
        dircol.NewSequentialVariable(rows=1, name="test")
        dircol.GetSequentialVariableAtIndex(name="test", index=2)

        dircol.AddRunningCost(x.dot(x))
        input_con = dircol.AddConstraintToAllKnotPoints(u[0] == 0)
        self.assertEqual(len(input_con), 21)
        interval_bound = dircol.AddTimeIntervalBounds(
            lower_bound=0.3, upper_bound=0.4
        )
        self.assertIsInstance(
            interval_bound.evaluator(), mp.BoundingBoxConstraint
        )
        equal_time_con = dircol.AddEqualTimeIntervalsConstraints()
        self.assertEqual(len(equal_time_con), 19)
        duration_bound = dircol.AddDurationBounds(
            lower_bound=0.3 * 21, upper_bound=0.4 * 21
        )
        self.assertIsInstance(duration_bound.evaluator(), mp.LinearConstraint)
        final_cost = dircol.AddFinalCost(2 * x.dot(x))
        self.assertIsInstance(final_cost.evaluator(), mp.Cost)

        initial_u = PiecewisePolynomial.ZeroOrderHold(
            [0, 0.3 * 21], np.zeros((1, 2))
        )
        initial_x = PiecewisePolynomial()
        dircol.SetInitialTrajectory(
            traj_init_u=initial_u, traj_init_x=initial_x
        )

        was_called = dict(input=False, state=False, complete=False)

        def input_callback(t, u):
            was_called["input"] = True

        def state_callback(t, x):
            was_called["state"] = True

        def complete_callback(t, x, u, v):
            was_called["complete"] = True

        dircol.AddInputTrajectoryCallback(callback=input_callback)
        dircol.AddStateTrajectoryCallback(callback=state_callback)
        dircol.AddCompleteTrajectoryCallback(
            callback=complete_callback, names=["test"]
        )

        result = mp.Solve(dircol.prog())
        self.assertTrue(was_called["input"])
        self.assertTrue(was_called["state"])
        self.assertTrue(was_called["complete"])

        dircol.GetSampleTimes(result=result)
        dircol.GetInputSamples(result=result)
        dircol.GetStateSamples(result=result)
        dircol.GetSequentialVariableSamples(result=result, name="test")
        dircol.ReconstructInputTrajectory(result=result)
        dircol.ReconstructStateTrajectory(result=result)

        constraint = DirectCollocationConstraint(plant, context)
        AddDirectCollocationConstraint(
            constraint,
            dircol.time_step(0),
            dircol.state(0),
            dircol.state(1),
            dircol.input(0),
            dircol.input(1),
            prog,
        )

        # Test AddConstraintToAllKnotPoints variants.
        nc = len(prog.bounding_box_constraints())
        c = dircol.AddConstraintToAllKnotPoints(
            constraint=mp.BoundingBoxConstraint([0], [1]), vars=u
        )
        self.assertIsInstance(c[0], mp.Binding[mp.BoundingBoxConstraint])
        self.assertEqual(
            len(prog.bounding_box_constraints()), nc + num_time_samples
        )
        nc = len(prog.linear_equality_constraints())
        c = dircol.AddConstraintToAllKnotPoints(
            constraint=mp.LinearEqualityConstraint([1], [0]), vars=u
        )
        self.assertIsInstance(c[0], mp.Binding[mp.LinearEqualityConstraint])
        self.assertEqual(
            len(prog.linear_equality_constraints()), nc + num_time_samples
        )
        nc = len(prog.linear_constraints())
        c = dircol.AddConstraintToAllKnotPoints(
            constraint=mp.LinearConstraint([1], [0], [1]), vars=u
        )
        self.assertIsInstance(c[0], mp.Binding[mp.LinearConstraint])
        self.assertEqual(len(prog.linear_constraints()), nc + num_time_samples)
        nc = len(prog.linear_equality_constraints())
        # eq(x, 2) produces a 2-dimensional vector of Formula.
        c = dircol.AddConstraintToAllKnotPoints(eq(x, 2))
        self.assertIsInstance(c[0].evaluator(), mp.LinearEqualityConstraint)
        self.assertEqual(
            len(prog.linear_equality_constraints()), nc + 2 * num_time_samples
        )

        # Add a second direct collocation problem to the same prog.
        num_vars = prog.num_vars()
        dircol2 = DirectCollocation(
            plant,
            context,
            num_time_samples=num_time_samples,
            minimum_time_step=0.2,
            maximum_time_step=0.5,
            input_port_index=InputPortSelection.kUseFirstInputIfItExists,
            assume_non_continuous_states_are_fixed=False,
            prog=prog,
        )
        self.assertEqual(dircol.prog(), dircol2.prog())
        self.assertEqual(prog.num_vars(), num_vars + num_initial_vars)

    def test_all_knot_points_constraint_python_wrapper_lost(self):
        # Ensure constraints' python wrappers are kept alive when added to a
        # multiple shooting instance via AddConstraintToAllKnotPoints(). See
        # issue #20131 for original problem description.

        # Make some objects in a function to let most of them be deleted at
        # scope exit. We will test if the contents of the first return value
        # ("keepers") succeed in keeping alive the objects tracked by "spies".
        def make_object_graph():
            spies = []
            plant = PendulumPlant()
            context = plant.CreateDefaultContext()

            num_time_samples = 21
            dircol = DirectCollocation(
                plant,
                context,
                num_time_samples=num_time_samples,
                minimum_time_step=0.2,
                maximum_time_step=0.5,
                input_port_index=InputPortSelection.kUseFirstInputIfItExists,
                assume_non_continuous_states_are_fixed=False,
            )

            con = mp.LinearConstraint([1], [0], [1])
            dircol.AddConstraintToAllKnotPoints(con, vars=dircol.input())
            spies.append(weakref.finalize(con, lambda: None))
            return dircol, spies

        keeper, spies = make_object_graph()
        self.assertTrue(all(spy.alive for spy in spies))

    def test_direct_transcription(self):
        # Integrator.
        plant = LinearSystem(
            A=[0.0], B=[1.0], C=[1.0], D=[0.0], time_period=0.1
        )
        context = plant.CreateDefaultContext()

        # Constructor for discrete systems.
        dirtran = DirectTranscription(plant, context, num_time_samples=21)

        # Spell out most of the methods, regardless of whether they make sense
        # as a consistent optimization.  The goal is to check the bindings,
        # not the implementation.
        dirtran.time()
        dirtran.fixed_time_step()
        x = dirtran.state()
        dirtran.state(2)
        dirtran.initial_state()
        dirtran.final_state()
        u = dirtran.input()
        dirtran.input(2)

        dirtran.AddRunningCost(x.dot(x))
        dirtran.AddConstraintToAllKnotPoints(u[0] == 0)
        dirtran.AddFinalCost(2 * x.dot(x))

        initial_u = PiecewisePolynomial.ZeroOrderHold(
            [0, 0.3 * 21], np.zeros((1, 2))
        )
        initial_x = PiecewisePolynomial()
        dirtran.SetInitialTrajectory(initial_u, initial_x)

        result = mp.Solve(dirtran.prog())
        dirtran.GetSampleTimes(result)
        dirtran.GetInputSamples(result)
        dirtran.GetStateSamples(result)
        dirtran.ReconstructInputTrajectory(result)
        dirtran.ReconstructStateTrajectory(result)

        # Confirm that the constructor for continuous systems works (and
        # confirm binding of nested TimeStep).
        plant = LinearSystem(
            A=[0.0], B=[1.0], C=[1.0], D=[0.0], time_period=0.0
        )
        context = plant.CreateDefaultContext()
        dirtran = DirectTranscription(
            plant,
            context,
            num_time_samples=21,
            fixed_time_step=DirectTranscription.TimeStep(0.1),
        )

    def test_kinematic_traj_opt_constraint_python_wrapper_lost(self):
        # Ensure constraints' python wrappers are kept alive when added
        # to a k.t.o. instance. See issue #20131 for original problem
        # description.

        # Make some objects in a function to let most of them be deleted at
        # scope exit. We will test if the contents of the first return value
        # ("keepers") succeed in keeping alive the objects tracked by "spies".
        def make_object_graph():
            spies = []
            trajopt = KinematicTrajectoryOptimization(
                num_positions=2,
                num_control_points=10,
                spline_order=3,
                duration=2.0,
            )

            b2 = np.zeros((2, 1))
            con = mp.LinearConstraint(np.eye(2), lb=b2, ub=b2)
            trajopt.AddPathPositionConstraint(con, 0)
            spies.append(weakref.finalize(con, lambda: None))

            b4 = np.zeros((4, 1))
            con = mp.LinearConstraint(np.eye(4), lb=b4, ub=b4)
            trajopt.AddVelocityConstraintAtNormalizedTime(con, 0)
            spies.append(weakref.finalize(con, lambda: None))

            return trajopt, spies

        keeper, spies = make_object_graph()
        self.assertTrue(all(spy.alive for spy in spies))

    def test_kinematic_trajectory_optimization(self):
        trajopt = KinematicTrajectoryOptimization(
            num_positions=2, num_control_points=10, spline_order=3, duration=2.0
        )
        self.assertIsInstance(trajopt.prog(), mp.MathematicalProgram)
        self.assertIsInstance(
            trajopt.get_mutable_prog(), mp.MathematicalProgram
        )
        self.assertEqual(trajopt.num_positions(), 2)
        self.assertEqual(trajopt.num_control_points(), 10)
        self.assertIsInstance(trajopt.basis(), BsplineBasis)
        self.assertEqual(trajopt.basis().order(), 3)
        self.assertEqual(trajopt.control_points().shape, (2, 10))
        self.assertIsInstance(trajopt.duration(), Variable)
        self.assertEqual(
            trajopt.prog().GetInitialGuess(trajopt.duration()), 2.0
        )
        self.assertEqual(len(trajopt.q()), 2)
        self.assertEqual(len(trajopt.qdot()), 2)
        self.assertEqual(len(trajopt.qddot()), 2)

        b = np.zeros((2, 1))
        trajopt.AddPathPositionConstraint(lb=b, ub=b, s=0)
        con = mp.LinearConstraint(np.eye(2), lb=b, ub=b)
        trajopt.AddPathPositionConstraint(con, 0)
        trajopt.AddPathVelocityConstraint(lb=b, ub=b, s=0)
        velocity_constraint = mp.LinearConstraint(
            np.eye(4), lb=np.zeros((4, 1)), ub=np.zeros((4, 1))
        )
        velocity_binding = mp.Binding[mp.LinearConstraint](
            mp.LinearConstraint(np.eye(2), lb=np.zeros(2), ub=np.zeros(2)),
            trajopt.qdot(),
        )
        trajopt.AddVelocityConstraintAtNormalizedTime(velocity_constraint, s=0)
        trajopt.AddVelocityConstraintAtNormalizedTime(
            binding=velocity_binding, s=0
        )
        trajopt.AddPathAccelerationConstraint(lb=b, ub=b, s=0)
        trajopt.AddDurationConstraint(1, 1)
        trajopt.AddPositionBounds(lb=b, ub=b)
        trajopt.AddVelocityBounds(lb=b, ub=b)
        trajopt.AddAccelerationBounds(lb=b, ub=b)
        trajopt.AddJerkBounds(lb=b, ub=b)

        plant = MultibodyPlant(time_step=0.0)
        Parser(plant).AddModelsFromUrl(
            url="package://drake/examples/multibody/cart_pole/cart_pole.sdf"
        )
        plant.Finalize()
        plant_context = plant.CreateDefaultContext()
        trajopt.AddEffortBoundsAtNormalizedTimes(
            plant=plant,
            s=[0.1, 0.9],
            lb=[-1] * plant.num_actuators(),
            ub=[3] * plant.num_actuators(),
            plant_context=plant_context,
        )

        trajopt.AddDurationCost(weight=1)
        trajopt.AddPathLengthCost(weight=1)
        trajopt.AddPathEnergyCost(weight=1)

        result = mp.Solve(trajopt.prog())
        q = trajopt.ReconstructTrajectory(result=result)
        self.assertIsInstance(q, BsplineTrajectory)
        trajopt.SetInitialGuess(trajectory=q)

    def test_gcs_trajectory_optimization_basic(self):
        """This based on the C++ GcsTrajectoryOptimizationTest.Basic test. It's
        a simple test of the bindings. It uses a single region (the unit box),
        and plans a line segment inside that box.
        """
        gcs = GcsTrajectoryOptimization(num_positions=2)
        start = [-0.5, -0.5]
        end = [0.5, 0.5]
        source = gcs.AddRegions(regions=[Point(start)], order=0)
        self.assertIn(source, gcs.GetSubgraphs())
        target = gcs.AddRegions(regions=[Point(end)], order=0)
        self.assertIn(target, gcs.GetSubgraphs())
        regions = gcs.AddRegions(
            regions=[HPolyhedron.MakeUnitBox(2)], order=1, h_min=1.0
        )
        self.assertEqual(len(regions.Edges()), 0)

        self.assertIn(regions, gcs.GetSubgraphs())
        edges = gcs.AddEdges(
            from_subgraph=source,
            to_subgraph=regions,
            subspace=None,
            edges_between_regions=None,
            edge_offsets=None,
        )
        self.assertIn(edges, gcs.GetEdgesBetweenSubgraphs())
        self.assertEqual(len(edges.Edges()), 1)
        self.assertEqual(edges.Edges()[0].u(), source.Vertices()[0])
        self.assertEqual(edges.Edges()[0].v(), regions.Vertices()[0])
        self.assertIn(
            gcs.AddEdges(regions, target), gcs.GetEdgesBetweenSubgraphs()
        )
        traj, result = gcs.SolvePath(source, target)
        self.assertTrue(result.is_success())
        self.assertEqual(traj.rows(), 2)
        self.assertEqual(traj.cols(), 1)
        traj_start = traj.value(traj.start_time()).squeeze()
        traj_end = traj.value(traj.end_time()).squeeze()
        np.testing.assert_allclose(traj_start, start, atol=1e-6)
        np.testing.assert_allclose(traj_end, end, atol=1e-6)

        # Since each segment of the normalized trajectory is one second long,
        # we expect the duration of the normalized trajectory to match the
        # number of segments.
        normalized_traj = GcsTrajectoryOptimization.NormalizeSegmentTimes(
            trajectory=traj
        )
        self.assertEqual(traj.start_time(), normalized_traj.start_time())
        self.assertEqual(
            traj.get_number_of_segments(),
            normalized_traj.get_number_of_segments(),
        )
        self.assertEqual(
            normalized_traj.get_number_of_segments(),
            normalized_traj.end_time() - normalized_traj.start_time(),
        )

        # The start and goal should be not altered.
        normalized_traj_start = normalized_traj.value(
            normalized_traj.start_time()
        ).squeeze()
        normalized_traj_end = normalized_traj.value(
            normalized_traj.end_time()
        ).squeeze()
        np.testing.assert_allclose(normalized_traj_start, start, atol=1e-6)
        np.testing.assert_allclose(normalized_traj_end, end, atol=1e-6)

        # We can also solve the convex restriction, by manually defining
        # through which regions the path should go.
        active_vertices = (
            source.Vertices() + regions.Vertices() + target.Vertices()
        )
        restricted_traj, restricted_result = gcs.SolveConvexRestriction(
            active_vertices
        )
        self.assertTrue(restricted_result.is_success())
        self.assertEqual(restricted_traj.rows(), 2)
        self.assertEqual(restricted_traj.cols(), 1)
        restricted_traj_start = restricted_traj.value(
            restricted_traj.start_time()
        ).squeeze()
        restricted_traj_end = restricted_traj.value(
            restricted_traj.end_time()
        ).squeeze()
        np.testing.assert_allclose(restricted_traj_start, start, atol=1e-6)
        np.testing.assert_allclose(restricted_traj_end, end, atol=1e-6)

        # We can add additional costs and constraints to Subgraphs with the
        # placeholder variables.
        vertex_duration = regions.vertex_duration()
        vertex_control_points = regions.vertex_control_points()
        edge_constituent_vertex_durations = (
            regions.edge_constituent_vertex_durations()
        )
        edge_constituent_vertex_control_points = (
            regions.edge_constituent_vertex_control_points()
        )

        vertex_cost = sum(
            [
                vertex_duration,
                vertex_control_points[0, 0],
                vertex_control_points[0, 1],
            ]
        )
        edge_cost = sum(
            [
                edge_constituent_vertex_durations[0],
                edge_constituent_vertex_durations[1],
                edge_constituent_vertex_control_points[0][0, 0],
                edge_constituent_vertex_control_points[0][0, 1],
                edge_constituent_vertex_control_points[1][0, 0],
                edge_constituent_vertex_control_points[1][0, 1],
            ]
        )
        vertex_constraint = vertex_cost >= 0
        edge_constraint = edge_cost >= 0

        regions.AddVertexCost(e=vertex_cost)
        regions.AddVertexConstraint(e=vertex_constraint)
        regions.AddEdgeCost(e=edge_cost)
        regions.AddEdgeConstraint(e=edge_constraint)

        all_transcriptions = {
            GraphOfConvexSets.Transcription.kMIP,
            GraphOfConvexSets.Transcription.kRelaxation,
            GraphOfConvexSets.Transcription.kRestriction,
        }
        regions.AddVertexCost(
            e=vertex_cost, use_in_transcription=all_transcriptions
        )
        regions.AddVertexConstraint(
            e=vertex_constraint, use_in_transcription=all_transcriptions
        )
        regions.AddEdgeCost(
            e=edge_cost, use_in_transcription=all_transcriptions
        )
        regions.AddEdgeConstraint(
            e=edge_constraint, use_in_transcription=all_transcriptions
        )

        to_bind_vertex_cost = ExpressionCost(vertex_cost)
        to_bind_vertex_constraint = ExpressionConstraint(
            [vertex_cost], [-1], [1]
        )
        to_bind_edge_cost = ExpressionCost(edge_cost)
        to_bind_edge_constraint = ExpressionConstraint([edge_cost], [-1], [1])

        restriction_only = {GraphOfConvexSets.Transcription.kRestriction}
        regions.AddVertexCost(
            binding=Binding[Cost](
                to_bind_vertex_cost, to_bind_vertex_cost.vars()
            ),
            use_in_transcription=restriction_only,
        )
        regions.AddVertexConstraint(
            binding=Binding[Constraint](
                to_bind_vertex_constraint, to_bind_vertex_constraint.vars()
            ),
            use_in_transcription=restriction_only,
        )
        regions.AddEdgeCost(
            binding=Binding[Cost](to_bind_edge_cost, to_bind_edge_cost.vars()),
            use_in_transcription=restriction_only,
        )
        regions.AddEdgeConstraint(
            binding=Binding[Constraint](
                to_bind_edge_constraint, to_bind_edge_constraint.vars()
            ),
            use_in_transcription=restriction_only,
        )

        # We can add additional costs and constraints to EdgesBetweenSubgraphs
        # with the placeholder variables.
        edge_constituent_vertex_durations = (
            edges.edge_constituent_vertex_durations()
        )
        edge_constituent_vertex_control_points = (
            edges.edge_constituent_vertex_control_points()
        )
        edge_cost = sum(
            [
                edge_constituent_vertex_durations[0],
                edge_constituent_vertex_durations[1],
                edge_constituent_vertex_control_points[0][0, 0],
                edge_constituent_vertex_control_points[1][0, 0],
                edge_constituent_vertex_control_points[1][0, 1],
            ]
        )
        edge_constraint = edge_cost >= 0

        edges.AddEdgeCost(e=edge_cost)
        edges.AddEdgeConstraint(e=edge_constraint)

        edges.AddEdgeCost(e=edge_cost, use_in_transcription=all_transcriptions)
        edges.AddEdgeConstraint(
            e=edge_constraint, use_in_transcription=all_transcriptions
        )

        to_bind_edge_cost = ExpressionCost(edge_cost)
        to_bind_edge_constraint = ExpressionConstraint([edge_cost], [-1], [1])

        edges.AddEdgeCost(
            binding=Binding[Cost](to_bind_edge_cost, to_bind_edge_cost.vars()),
            use_in_transcription=restriction_only,
        )
        edges.AddEdgeConstraint(
            binding=Binding[Constraint](
                to_bind_edge_constraint, to_bind_edge_constraint.vars()
            ),
            use_in_transcription=restriction_only,
        )

        traj, result = gcs.SolvePath(source, target)
        self.assertTrue(result.is_success())

        # Test that removing all subgraphs, removes all vertices and edges.
        self.assertEqual(
            len(gcs.graph_of_convex_sets().Vertices()),
            len(source.Vertices())
            + len(regions.Vertices())
            + len(target.Vertices()),
        )
        self.assertEqual(len(gcs.GetSubgraphs()), 3)
        self.assertEqual(len(gcs.GetEdgesBetweenSubgraphs()), 2)
        gcs.RemoveSubgraph(source)
        gcs.RemoveSubgraph(regions)
        gcs.RemoveSubgraph(target)
        # We expect the underlying gcs problem to be empty.
        self.assertFalse(len(gcs.graph_of_convex_sets().Vertices()))
        self.assertFalse(len(gcs.graph_of_convex_sets().Edges()))
        self.assertFalse(len(gcs.GetSubgraphs()))
        self.assertFalse(len(gcs.GetEdgesBetweenSubgraphs()), 0)

    def test_gcs_trajectory_optimization_2d(self):
        """The following 2D environment has been presented in the GCS paper.

            We have two possible starts, S1 and S2, and two possible goals,
            G1 and G2. The goal is to find a the shortest path from either of
            the starts to either of the goals while avoiding the obstacles.
            Further we constraint the path to go through either the intermediate
            point I or the subspace.

        ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
        ░░      ░░░░░░░░                                     ░░░░░░░░         ░░
        ░░      ░░░░░░░░                                     ░░░░░░░░         ░░
        ░░      ░░░░░░░░                                     ░░░░░░░░    G1   ░░
        ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░      ░░░░░░░░         ░░
        ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░      ░░░░░░░░         ░░
        ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░      ░░░░░░░░         ░░
        ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░      ░░░░░░░░         ░░
        ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░      ░░░░░░░░         ░░
        ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░      ░░░░░░░░         ░░
        ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░      ░░░░░░░░         ░░
        ░░    S2░░░░░░░░      ░░░░░░░░░░ I ░░░░░░░░░░░░      ░░░░░░░░         ░░
        ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░      ░░░░░░░░         ░░
        ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░                       ░░
        ░░      ░░░░░░░░      ░░░░░░░░░░   ░░░░░░░░░░░░                       ░░
        ░░                                 ░░░░░░░░░░░░      ░░░░░░░░░░░░░░░░░░░
        ░░                          ░░░░░░░░░░░░░░░░░░░                       ░░
        ░░                        ░░░░░░░░░░░░░░░░░░░░░░░░                 G2 ░░
        ░░      ░░░░░░░░        ░░░░░░░░░░░░░░░░░░░░░░░░░░░░                  ░░
        ░░      ░░░░░░░░          ░░░░░░░░░░░░░░░░░░░░░░░░░░░░                ░░
        ░░      ░░░░░░░░            ░░░░░░░░░░░░░░░░░░░░░░░░░░░░              ░░
        ░░      ░░░░░░░░              ░░░░░░░░░░░░░░░░░░░░░░░░░░░░            ░░
        ░░      ░░░░░░░░                ░░░░░░░░░░░░░░░░░░░░░░░░░░░░          ░░
        ░░      ░░░░░░░░                  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░        ░░
        ░░      ░░░░░░░░                    ░░░░░░░░░░░░░░░░░░░░░░░░░░░░xxxxxx░░
        ░░      ░░░░░░░░                      ░░░░░░░░░░░░░░░░░░░░░░░░xxxxxxxx░░
        ░░      ░░░░░░░░                        ░░░░░░░░░░░░░░░░░░░░xxxxxxxxxx░░
        ░░      ░░░░░░░░                          ░░░░░░░░░░░░░░░░xxxxxxxxxxxx░░
        ░░      ░░░░░░░░                            ░░░░░░░░░░░░xxx Subspace x░░
        ░░      ░░░░░░░░                                      xxxxxxxxxxxxxxxx░░
        ░░  S1  ░░░░░░░░                                      xxxxxxxxxxxxxxxx░░
        ░░      ░░░░░░░░                                      xxxxxxxxxxxxxxxx░░
        ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
        """

        dimension = 2
        gcs = GcsTrajectoryOptimization(num_positions=dimension)
        self.assertEqual(gcs.num_positions(), dimension)

        # Define the collision free space.
        vertices = [
            np.array([[0.4, 0.4, 0.0, 0.0], [0.0, 5.0, 5.0, 0.0]]),
            np.array([[0.4, 1.0, 1.0, 0.4], [2.4, 2.4, 2.6, 2.6]]),
            np.array([[1.4, 1.4, 1.0, 1.0], [2.2, 4.6, 4.6, 2.2]]),
            np.array([[1.4, 2.4, 2.4, 1.4], [2.2, 2.6, 2.8, 2.8]]),
            np.array([[2.2, 2.4, 2.4, 2.2], [2.8, 2.8, 4.6, 4.6]]),
            np.array([[1.4, 1.0, 1.0, 3.8, 3.8], [2.2, 2.2, 0.0, 0.0, 0.2]]),
            np.array([[3.8, 3.8, 1.0, 1.0], [4.6, 5.0, 5.0, 4.6]]),
            np.array([[5.0, 5.0, 4.8, 3.8, 3.8], [0.0, 1.2, 1.2, 0.2, 0.0]]),
            np.array([[3.4, 4.8, 5.0, 5.0], [2.6, 1.2, 1.2, 2.6]]),
            np.array([[3.4, 3.8, 3.8, 3.4], [2.6, 2.6, 4.6, 4.6]]),
            np.array([[3.8, 4.4, 4.4, 3.8], [2.8, 2.8, 3.0, 3.0]]),
            np.array([[5.0, 5.0, 4.4, 4.4], [2.8, 5.0, 5.0, 2.8]]),
        ]

        max_vel = np.ones((2, 1))

        # We add a path length cost to the entire graph.
        # This can be called ahead of time or after adding the regions.
        gcs.AddPathLengthCost(weight=1.0)
        # This cost is equivalent to the above.
        # It will be added twice, which is unnecessary,
        # but we do it to test the binding.
        gcs.AddPathLengthCost(weight_matrix=np.eye(dimension))

        # Add a mimimum time cost to the entire graph.
        gcs.AddTimeCost(weight=1.0)
        # Add the cost again, which is unnecessary for the optimization
        # but useful to check the binding with the default values.
        gcs.AddTimeCost()

        # Add velocity bounds to the entire graph.
        gcs.AddVelocityBounds(lb=-max_vel, ub=max_vel)

        # Add a velocity and acceleration continuity to the entire graph.
        gcs.AddPathContinuityConstraints(1)
        gcs.AddPathContinuityConstraints(2)

        # Add two subgraphs with different orders.
        main1 = gcs.AddRegions(
            regions=[HPolyhedron(VPolytope(v)) for v in vertices],
            order=4,
            name="main1",
        )
        self.assertIsInstance(main1, GcsTrajectoryOptimization.Subgraph)
        self.assertEqual(main1.order(), 4)
        self.assertEqual(main1.name(), "main1")
        self.assertEqual(main1.size(), len(vertices))
        self.assertIsInstance(main1.regions(), list)
        self.assertIsInstance(main1.regions()[0], HPolyhedron)
        self.assertIn(main1, gcs.GetSubgraphs())

        # This adds the edges manually.
        # Doing this is much faster since it avoids computing
        # pairwise set intersection checks.
        main2 = gcs.AddRegions(
            regions=[HPolyhedron(VPolytope(v)) for v in vertices],
            edges_between_regions=[
                (0, 1),
                (1, 0),
                (1, 2),
                (2, 1),
                (2, 3),
                (3, 2),
                (2, 5),
                (5, 2),
                (2, 6),
                (6, 2),
                (3, 4),
                (4, 3),
                (3, 5),
                (5, 3),
                (4, 6),
                (6, 4),
                (5, 7),
                (7, 5),
                (6, 9),
                (9, 6),
                (7, 8),
                (8, 7),
                (8, 9),
                (9, 8),
                (9, 10),
                (10, 9),
                (10, 11),
                (11, 10),
            ],
            order=6,
            name="main2",
        )

        self.assertIsInstance(main2, GcsTrajectoryOptimization.Subgraph)
        self.assertEqual(main2.order(), 6)
        self.assertEqual(main2.name(), "main2")
        self.assertEqual(main2.size(), len(vertices))
        self.assertIsInstance(main2.regions(), list)
        self.assertIsInstance(main2.regions()[0], HPolyhedron)
        self.assertIsInstance(main2.Vertices(), list)
        self.assertIsInstance(main2.Vertices()[0], GraphOfConvexSets.Vertex)
        self.assertIn(main2, gcs.GetSubgraphs())

        # Add two start and goal regions.
        start1 = np.array([0.2, 0.2])
        start2 = np.array([0.3, 3.2])

        goal1 = np.array([4.8, 4.8])
        goal2 = np.array([4.9, 2.4])

        source = gcs.AddRegions(
            [Point(start1), Point(start2)], order=0, name="starts"
        )
        self.assertIsInstance(source, GcsTrajectoryOptimization.Subgraph)
        self.assertEqual(source.order(), 0)
        self.assertEqual(source.name(), "starts")
        self.assertEqual(source.size(), 2)
        self.assertIsInstance(source.regions(), list)
        self.assertIsInstance(source.regions()[0], Point)
        self.assertIsInstance(source.Vertices(), list)
        self.assertIsInstance(source.Vertices()[0], GraphOfConvexSets.Vertex)
        self.assertIn(source, gcs.GetSubgraphs())

        # Here we force a delay of 10 seconds at the goal.
        target = gcs.AddRegions(
            regions=[Point(goal1), Point(goal2)],
            order=0,
            h_min=10,
            h_max=10,
            name="goals",
        )
        self.assertIsInstance(target, GcsTrajectoryOptimization.Subgraph)
        self.assertEqual(target.order(), 0)
        self.assertEqual(target.name(), "goals")
        self.assertEqual(target.size(), 2)
        self.assertIsInstance(target.regions(), list)
        self.assertIsInstance(target.regions()[0], Point)
        self.assertIsInstance(target.Vertices(), list)
        self.assertIsInstance(target.Vertices()[0], GraphOfConvexSets.Vertex)
        self.assertIn(target, gcs.GetSubgraphs())

        # We connect the subgraphs main1 and main2 by constraining it to
        # go through either of the subspaces.
        subspace_region = HPolyhedron(VPolytope(vertices[7]))
        subspace_point = Point([2.3, 3.5])

        main1_to_main2_pt = gcs.AddEdges(main1, main2, subspace=subspace_point)
        self.assertIsInstance(
            main1_to_main2_pt, GcsTrajectoryOptimization.EdgesBetweenSubgraphs
        )
        self.assertIn(main1_to_main2_pt, gcs.GetEdgesBetweenSubgraphs())

        main1_to_main2_region = gcs.AddEdges(
            main1, main2, subspace=subspace_region
        )
        self.assertIsInstance(
            main1_to_main2_region,
            GcsTrajectoryOptimization.EdgesBetweenSubgraphs,
        )
        self.assertIn(main1_to_main2_region, gcs.GetEdgesBetweenSubgraphs())

        # Add half of the maximum velocity constraint at the subspace point
        # and region.
        main1_to_main2_pt.AddVelocityBounds(lb=-max_vel / 2, ub=max_vel / 2)
        main1_to_main2_region.AddVelocityBounds(lb=-max_vel / 2, ub=max_vel / 2)

        # We connect the start and goal regions to the rest of the graph.
        self.assertIsInstance(
            gcs.AddEdges(source, main1),
            GcsTrajectoryOptimization.EdgesBetweenSubgraphs,
        )
        main2_to_target = gcs.AddEdges(main2, target)
        self.assertIsInstance(
            main2_to_target, GcsTrajectoryOptimization.EdgesBetweenSubgraphs
        )
        self.assertIn(main2_to_target, gcs.GetEdgesBetweenSubgraphs())

        # Add final zero velocity and acceleration.
        main2_to_target.AddZeroDerivativeConstraints(derivative_order=1)
        main2_to_target.AddZeroDerivativeConstraints(derivative_order=2)

        # This weight matrix penalizes movement in the y direction three
        # times more than in the x direction only for the main2 subgraph.
        main2.AddPathLengthCost(weight_matrix=np.diag([1.0, 3.0]))

        # Adding this cost checks the python binding. It won't contribute to
        # the solution since we already added the minimum time cost to the
        # whole graph.
        main2.AddTimeCost(weight=1.0)
        # Add the cost again, which is unnecessary for the optimization
        # but useful to check the binding with the default values.
        main2.AddTimeCost()

        # Adding this constraint checks the python binding. It won't
        # contribute to the solution since we already added the continuity
        # constraints to the whole graph.
        main2.AddPathContinuityConstraints(1)
        main1_to_main2_region.AddPathContinuityConstraints(1)

        # Add tighter velocity bounds to the main2 subgraph.
        main2.AddVelocityBounds(lb=-0.5 * max_vel, ub=0.5 * max_vel)

        self.assertIsInstance(gcs.graph_of_convex_sets(), GraphOfConvexSets)

        options = GraphOfConvexSetsOptions()
        options.convex_relaxation = True
        options.max_rounded_paths = 5

        traj, result = gcs.SolvePath(
            source=source, target=target, options=options
        )

        self.assertIsInstance(result, mp.MathematicalProgramResult)
        self.assertIsInstance(traj, CompositeTrajectory)
        self.assertTrue(result.is_success())
        self.assertEqual(traj.rows(), dimension)

        np.testing.assert_array_almost_equal(
            traj.value(traj.start_time()), start2[:, None], 6
        )
        np.testing.assert_array_almost_equal(
            traj.value(traj.end_time()), goal2[:, None], 6
        )

        # Check that the delay at the goal is respected.
        np.testing.assert_array_almost_equal(
            traj.value(traj.end_time() - 10), goal2[:, None], 6
        )
        self.assertTrue(traj.end_time() - traj.start_time() >= 10)

        graphviz_options = GcsGraphvizOptions()
        self.assertIsInstance(
            gcs.GetGraphvizString(result=result, options=graphviz_options), str
        )

        self.assertIsInstance(
            gcs.GetGraphvizString(
                result=result,
                show_slacks=True,
                show_vars=True,
                show_flows=True,
                show_costs=False,
                scientific=True,
                precision=3,
            ),
            str,
        )

        # In the follwoing, we test adding the bindings for nonlinear
        # constraints.
        max_acceleration = np.ones((2, 1))
        max_jerk = 7.5 * np.ones((2, 1))

        # Add global acceleration bounds.
        gcs.AddNonlinearDerivativeBounds(
            lb=-max_acceleration, ub=max_acceleration, derivative_order=2
        )

        # Add the jerk bounds to each subgraph.
        main1.AddNonlinearDerivativeBounds(
            lb=-max_jerk, ub=max_jerk, derivative_order=3
        )
        main2.AddNonlinearDerivativeBounds(
            lb=-max_jerk, ub=max_jerk, derivative_order=3
        )

        # Add half of the maximum acceleration and jerk bounds at the subspace
        # point and region.
        main1_to_main2_pt.AddVelocityBounds(
            lb=-max_acceleration / 2, ub=max_acceleration / 2
        )
        main1_to_main2_region.AddVelocityBounds(
            lb=-max_acceleration / 2, ub=max_acceleration / 2
        )

        main1_to_main2_pt.AddVelocityBounds(lb=-max_jerk / 2, ub=max_jerk / 2)
        main1_to_main2_region.AddVelocityBounds(
            lb=-max_jerk / 2, ub=max_jerk / 2
        )

        # Add global velocity continuity.
        gcs.AddContinuityConstraints(continuity_order=1)

        # Add acceleration continuity to each subgraph.
        main1.AddContinuityConstraints(continuity_order=2)
        main2.AddContinuityConstraints(continuity_order=2)

        # Add acceleration continuity at the subspace point and region.
        main1_to_main2_pt.AddContinuityConstraints(continuity_order=2)
        main1_to_main2_region.AddContinuityConstraints(continuity_order=2)

    def test_gcs_trajectory_optimization_wraparound(self):
        gcs_wraparound = GcsTrajectoryOptimization(
            num_positions=1, continuous_revolute_joints=[0]
        )
        self.assertEqual(len(gcs_wraparound.continuous_revolute_joints()), 1)
        g1 = gcs_wraparound.AddRegions(
            regions=[Point([0]), Point([2 * np.pi])],
            order=1,
            edges_between_regions=[[0, 1]],
            edge_offsets=[[2 * np.pi]],
        )
        g2 = gcs_wraparound.AddRegions(
            regions=[VPolytope([[8 * np.pi, 8 * np.pi + 1]])], order=2
        )
        gcs_wraparound.AddEdges(g1, g2)
        traj, result = gcs_wraparound.SolvePath(g1, g2)
        self.assertTrue(result.is_success())

        new_traj = GcsTrajectoryOptimization.UnwrapToContinuousTrajectory(
            gcs_trajectory=traj,
            continuous_revolute_joints=[0],
            starting_rounds=[43],
            tol=1e-8,
        )
        diff = (
            new_traj.value(new_traj.start_time())
            - traj.value(traj.start_time())
        ) % (2 * np.pi)
        # Modulus may lead to the value being almost 2*pi, instead of zero.
        if diff > np.pi:
            diff -= 2 * np.pi
        np.testing.assert_array_almost_equal(diff, np.zeros_like(diff))

    def test_get_continuous_revolute_joint_indices(self):
        plant = MultibodyPlant(0.0)
        plant.Finalize()
        indices = GetContinuousRevoluteJointIndices(plant=plant)
        self.assertEqual(len(indices), 0)
