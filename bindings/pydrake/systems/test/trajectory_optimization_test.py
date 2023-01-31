import math
import unittest
import warnings

import numpy as np

from pydrake.examples import PendulumPlant
from pydrake.math import eq, BsplineBasis
from pydrake.trajectories import PiecewisePolynomial, BsplineTrajectory
from pydrake.solvers import mathematicalprogram as mp
from pydrake.symbolic import Variable
from pydrake.systems.framework import InputPortSelection
from pydrake.systems.primitives import LinearSystem
from pydrake.systems.trajectory_optimization import (
    AddDirectCollocationConstraint,
    DirectCollocation,
    DirectCollocationConstraint,
    DirectTranscription,
    KinematicTrajectoryOptimization,
    TimeStep,
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
            minimum_timestep=0.2,
            maximum_timestep=0.5,
            input_port_index=InputPortSelection.kUseFirstInputIfItExists,
            assume_non_continuous_states_are_fixed=False)
        prog = dircol.prog()

        # Spell out most of the methods, regardless of whether they make sense
        # as a consistent optimization.  The goal is to check the bindings,
        # not the implementation.
        t = dircol.time()
        dt = dircol.timestep(index=0)
        x = dircol.state()
        x2 = dircol.state(index=2)
        x0 = dircol.initial_state()
        xf = dircol.final_state()
        u = dircol.input()
        u2 = dircol.input(index=2)
        v = dircol.NewSequentialVariable(rows=1, name="test")
        v2 = dircol.GetSequentialVariableAtIndex(name="test", index=2)

        dircol.AddRunningCost(x.dot(x))
        input_con = dircol.AddConstraintToAllKnotPoints(u[0] == 0)
        self.assertEqual(len(input_con), 21)
        interval_bound = dircol.AddTimeIntervalBounds(
            lower_bound=0.3, upper_bound=0.4)
        self.assertIsInstance(interval_bound.evaluator(),
                              mp.BoundingBoxConstraint)
        equal_time_con = dircol.AddEqualTimeIntervalsConstraints()
        self.assertEqual(len(equal_time_con), 19)
        duration_bound = dircol.AddDurationBounds(
            lower_bound=.3*21, upper_bound=0.4*21)
        self.assertIsInstance(duration_bound.evaluator(), mp.LinearConstraint)
        final_cost = dircol.AddFinalCost(2*x.dot(x))
        self.assertIsInstance(final_cost.evaluator(), mp.Cost)

        initial_u = PiecewisePolynomial.ZeroOrderHold([0, .3*21],
                                                      np.zeros((1, 2)))
        initial_x = PiecewisePolynomial()
        dircol.SetInitialTrajectory(traj_init_u=initial_u,
                                    traj_init_x=initial_x)

        was_called = dict(
            input=False,
            state=False,
            complete=False
        )

        def input_callback(t, u):
            was_called["input"] = True

        def state_callback(t, x):
            was_called["state"] = True

        def complete_callback(t, x, u, v):
            was_called["complete"] = True

        dircol.AddInputTrajectoryCallback(callback=input_callback)
        dircol.AddStateTrajectoryCallback(callback=state_callback)
        dircol.AddCompleteTrajectoryCallback(callback=complete_callback,
                                             names=["test"])

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
        AddDirectCollocationConstraint(constraint, dircol.timestep(0),
                                       dircol.state(0), dircol.state(1),
                                       dircol.input(0), dircol.input(1),
                                       prog)

        # Test AddConstraintToAllKnotPoints variants.
        nc = len(prog.bounding_box_constraints())
        c = dircol.AddConstraintToAllKnotPoints(
            constraint=mp.BoundingBoxConstraint([0], [1]), vars=u)
        self.assertIsInstance(c[0], mp.Binding[mp.BoundingBoxConstraint])
        self.assertEqual(len(prog.bounding_box_constraints()),
                         nc + num_time_samples)
        nc = len(prog.linear_equality_constraints())
        c = dircol.AddConstraintToAllKnotPoints(
            constraint=mp.LinearEqualityConstraint([1], [0]), vars=u)
        self.assertIsInstance(c[0], mp.Binding[mp.LinearEqualityConstraint])
        self.assertEqual(len(prog.linear_equality_constraints()),
                         nc + num_time_samples)
        nc = len(prog.linear_constraints())
        c = dircol.AddConstraintToAllKnotPoints(
            constraint=mp.LinearConstraint([1], [0], [1]), vars=u)
        self.assertIsInstance(c[0], mp.Binding[mp.LinearConstraint])
        self.assertEqual(len(prog.linear_constraints()), nc + num_time_samples)
        nc = len(prog.linear_equality_constraints())
        # eq(x, 2) produces a 2-dimensional vector of Formula.
        c = dircol.AddConstraintToAllKnotPoints(eq(x, 2))
        self.assertIsInstance(c[0].evaluator(), mp.LinearEqualityConstraint)
        self.assertEqual(len(prog.linear_equality_constraints()),
                         nc + 2*num_time_samples)

    def test_direct_transcription(self):
        # Integrator.
        plant = LinearSystem(A=[0.], B=[1.], C=[1.], D=[0.], time_period=0.1)
        context = plant.CreateDefaultContext()

        dirtran = DirectTranscription(plant, context, num_time_samples=21)

        # Spell out most of the methods, regardless of whether they make sense
        # as a consistent optimization.  The goal is to check the bindings,
        # not the implementation.
        t = dirtran.time()
        dt = dirtran.fixed_timestep()
        x = dirtran.state()
        x2 = dirtran.state(2)
        x0 = dirtran.initial_state()
        xf = dirtran.final_state()
        u = dirtran.input()
        u2 = dirtran.input(2)

        dirtran.AddRunningCost(x.dot(x))
        dirtran.AddConstraintToAllKnotPoints(u[0] == 0)
        dirtran.AddFinalCost(2*x.dot(x))

        initial_u = PiecewisePolynomial.ZeroOrderHold([0, .3*21],
                                                      np.zeros((1, 2)))
        initial_x = PiecewisePolynomial()
        dirtran.SetInitialTrajectory(initial_u, initial_x)

        result = mp.Solve(dirtran.prog())
        times = dirtran.GetSampleTimes(result)
        inputs = dirtran.GetInputSamples(result)
        states = dirtran.GetStateSamples(result)
        input_traj = dirtran.ReconstructInputTrajectory(result)
        state_traj = dirtran.ReconstructStateTrajectory(result)

    def test_kinematic_trajectory_optimization(self):
        trajopt = KinematicTrajectoryOptimization(num_positions=2,
                                                  num_control_points=10,
                                                  spline_order=3,
                                                  duration=2.0)
        self.assertIsInstance(trajopt.prog(), mp.MathematicalProgram)
        self.assertIsInstance(trajopt.get_mutable_prog(),
                              mp.MathematicalProgram)
        self.assertEqual(trajopt.num_positions(), 2)
        self.assertEqual(trajopt.num_control_points(), 10)
        self.assertIsInstance(trajopt.basis(), BsplineBasis)
        self.assertEqual(trajopt.basis().order(), 3)
        self.assertEqual(trajopt.control_points().shape, (2, 10))
        self.assertIsInstance(trajopt.duration(), Variable)
        self.assertEqual(trajopt.prog().GetInitialGuess(trajopt.duration()),
                         2.0)

        b = np.zeros((2, 1))
        trajopt.AddPathPositionConstraint(lb=b, ub=b, s=0)
        con = mp.LinearConstraint(np.eye(2), lb=b, ub=b)
        trajopt.AddPathPositionConstraint(con, 0)
        trajopt.AddPathVelocityConstraint(lb=b, ub=b, s=0)
        velocity_constraint = mp.LinearConstraint(np.eye(4),
                                                  lb=np.zeros((4, 1)),
                                                  ub=np.zeros((4, 1)))
        trajopt.AddVelocityConstraintAtNormalizedTime(velocity_constraint, s=0)
        trajopt.AddPathAccelerationConstraint(lb=b, ub=b, s=0)
        trajopt.AddDurationConstraint(1, 1)
        trajopt.AddPositionBounds(lb=b, ub=b)
        trajopt.AddVelocityBounds(lb=b, ub=b)
        trajopt.AddAccelerationBounds(lb=b, ub=b)
        trajopt.AddJerkBounds(lb=b, ub=b)

        trajopt.AddDurationCost(weight=1)
        trajopt.AddPathLengthCost(weight=1)

        result = mp.Solve(trajopt.prog())
        q = trajopt.ReconstructTrajectory(result=result)
        self.assertIsInstance(q, BsplineTrajectory)
        trajopt.SetInitialGuess(trajectory=q)
