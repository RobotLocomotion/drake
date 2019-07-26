from __future__ import print_function

import math
import unittest
import warnings

import numpy as np

from pydrake.examples.pendulum import PendulumPlant
from pydrake.trajectories import PiecewisePolynomial
from pydrake.solvers import mathematicalprogram as mp
from pydrake.systems.framework import InputPortSelection
from pydrake.systems.primitives import LinearSystem
from pydrake.systems.trajectory_optimization import (
    AddDirectCollocationConstraint, DirectCollocation,
    DirectCollocationConstraint, DirectTranscription,
    TimeStep,
)


class TestTrajectoryOptimization(unittest.TestCase):
    def test_direct_collocation(self):
        plant = PendulumPlant()
        context = plant.CreateDefaultContext()

        dircol = DirectCollocation(
            plant, context, num_time_samples=21, minimum_timestep=0.2,
            maximum_timestep=0.5,
            input_port_index=InputPortSelection.kUseFirstInputIfItExists,
            assume_non_continuous_states_are_fixed=False)

        # Spell out most of the methods, regardless of whether they make sense
        # as a consistent optimization.  The goal is to check the bindings,
        # not the implementation.
        t = dircol.time()
        dt = dircol.timestep(0)
        x = dircol.state()
        x2 = dircol.state(2)
        x0 = dircol.initial_state()
        xf = dircol.final_state()
        u = dircol.input()
        u2 = dircol.input(2)
        v = dircol.NewSequentialVariable(rows=1, name="test")
        v2 = dircol.GetSequentialVariableAtIndex(name="test", index=2)

        dircol.AddRunningCost(x.dot(x))
        dircol.AddConstraintToAllKnotPoints(u[0] == 0)
        dircol.AddTimeIntervalBounds(0.3, 0.4)
        dircol.AddEqualTimeIntervalsConstraints()
        dircol.AddDurationBounds(.3*21, 0.4*21)
        dircol.AddFinalCost(2*x.dot(x))

        initial_u = PiecewisePolynomial.ZeroOrderHold([0, .3*21],
                                                      np.zeros((1, 2)))
        initial_x = PiecewisePolynomial()
        dircol.SetInitialTrajectory(initial_u, initial_x)

        global input_was_called
        input_was_called = False
        global state_was_called
        state_was_called = False
        global complete_was_called
        complete_was_called = False

        def input_callback(t, u):
            global input_was_called
            input_was_called = True

        def state_callback(t, x):
            global state_was_called
            state_was_called = True

        def complete_callback(t, x, u, v):
            global complete_was_called
            complete_was_called = True

        dircol.AddInputTrajectoryCallback(input_callback)
        dircol.AddStateTrajectoryCallback(state_callback)
        dircol.AddCompleteTrajectoryCallback(callback=complete_callback,
                                             names=["test"])

        result = mp.Solve(dircol)
        self.assertTrue(input_was_called)
        self.assertTrue(state_was_called)
        self.assertTrue(complete_was_called)

        times = dircol.GetSampleTimes(result)
        inputs = dircol.GetInputSamples(result)
        states = dircol.GetStateSamples(result)
        variables = dircol.GetSequentialVariableSamples(result=result,
                                                        name="test")
        input_traj = dircol.ReconstructInputTrajectory(result)
        state_traj = dircol.ReconstructStateTrajectory(result)

        constraint = DirectCollocationConstraint(plant, context)
        AddDirectCollocationConstraint(constraint, dircol.timestep(0),
                                       dircol.state(0), dircol.state(1),
                                       dircol.input(0), dircol.input(1),
                                       dircol)

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

        result = mp.Solve(dirtran)
        times = dirtran.GetSampleTimes(result)
        inputs = dirtran.GetInputSamples(result)
        states = dirtran.GetStateSamples(result)
        input_traj = dirtran.ReconstructInputTrajectory(result)
        state_traj = dirtran.ReconstructStateTrajectory(result)

    def test_direct_transcription_continuous_time(self):
        # Test that the continuous-time constructor is also spelled correctly.
        plant = LinearSystem(A=[0.], B=[1.], C=[1.], D=[0.])
        context = plant.CreateDefaultContext()
        dirtran = DirectTranscription(plant, context, num_time_samples=3,
                                      fixed_timestep=TimeStep(0.1))
        self.assertEqual(len(dirtran.linear_equality_constraints()), 3)
