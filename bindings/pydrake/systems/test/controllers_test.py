#!/usr/bin/env python

from __future__ import print_function

import math
import matplotlib.pyplot as plt
from matplotlib import cm as color_map
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import unittest

from pydrake.examples.pendulum import PendulumPlant
from pydrake.systems.analysis import Simulator
from pydrake._math import BarycentricMesh
from pydrake.systems.controllers import (
    DynamicProgrammingOptions, FittedValueIteration)


class TestControllers(unittest.TestCase):
    def test_fitted_value_iteration_pendulum(self):
        plant = PendulumPlant()
        simulator = Simulator(plant)

        def quadratic_regulator_cost(context):
            x = context.get_continuous_state_vector().CopyToVector()
            x[0] = x[0] - math.pi
            u = plant.EvalVectorInput(context, 0).CopyToVector()
            return x.dot(x) + u.dot(u)

        qbins = np.linspace(0., 2.*math.pi, 51)
        qdotbins = np.linspace(-10., 10., 41)
        state_grid = [set(qbins), set(qdotbins)]

        input_limit = 2.
        input_mesh = [set(np.linspace(-input_limit, input_limit, 9))]
        timestep = 0.01

        [Q, Qdot] = np.meshgrid(qbins, qdotbins)
        fig = plt.figure()
        ax = fig.gca(projection='3d')

        def draw(iteration, mesh, cost_to_go, policy):
            # Drawing is slow, don't draw every frame.
            if iteration % 20 != 0:
                return
            plt.title("iteration " + str(iteration))
            J = np.reshape(cost_to_go, Q.shape)
            surf = ax.plot_surface(Q, Qdot, J, rstride=1, cstride=1,
                                   cmap=color_map.jet)
            plt.pause(0.00001)
            surf.remove()

        options = DynamicProgrammingOptions()
        options.convergence_tol = 1.
        options.state_indices_with_periodic_boundary_conditions = {0}
        options.visualization_callback = draw

        policy, cost_to_go = FittedValueIteration(simulator,
                                                  quadratic_regulator_cost,
                                                  state_grid, input_mesh,
                                                  timestep, options)


if __name__ == '__main__':
    unittest.main()
