import unittest

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import (
    Context, DiagramBuilder, PortDataType, VectorSystem, kUseDefaultName)
from pydrake.systems.primitives import VectorLogSink
from pydrake.systems.pyplot_visualizer import PyPlotVisualizer
from pydrake.trajectories import PiecewisePolynomial


# TODO(tehbelinda): Augment this test with a Jupyter notebook to make this
# easier to visualize.
class TestVisualizer(PyPlotVisualizer):
    # Set limits of view port.
    XLIM = (-20., 20.)
    YLIM = (-6., 6.)
    TICK_DIMS = (0.2, 0.8)
    PATCH_WIDTH = 5.
    PATCH_HEIGHT = 1.

    def __init__(self, size):
        PyPlotVisualizer.__init__(self)
        self.DeclareInputPort(kUseDefaultName, PortDataType.kVectorValued,
                              size)

        self.ax.set_xlim(*self.XLIM)
        self.ax.set_ylim(*self.YLIM)
        self.ax.set_aspect('auto')

        self._make_background()

        self.patch = plt.Rectangle((0.0, 0.0),
                                   self.PATCH_WIDTH, self.PATCH_HEIGHT,
                                   fc='#A31F34', ec='k')
        self.patch.set_x(-self.PATCH_WIDTH / 2)  # Center at x.

    def _make_background(self):
        # X-axis.
        plt.plot(self.XLIM, np.zeros_like(self.XLIM), 'k')
        # Tick mark centered at the origin.
        tick_pos = -0.5 * np.asarray(self.TICK_DIMS)
        self.ax.add_patch(plt.Rectangle(tick_pos, *self.TICK_DIMS, fc='k'))

    def draw(self, context):
        try:
            x = self.EvalVectorInput(context, 0).get_value()[0]
        except TypeError:
            x = context[0]
        self.patch.set_x(x - self.PATCH_WIDTH / 2)


class SimpleContinuousTimeSystem(VectorSystem):
    def __init__(self):
        VectorSystem.__init__(self,
                              0,        # Zero inputs.
                              1)        # One output.
        self.DeclareContinuousState(1)  # One state variable.

    # xdot(t) = -x(t) + x^3(t)
    def DoCalcVectorTimeDerivatives(self, context, u, x, xdot):
        xdot[:] = -x + x**3

    # y(t) = x(t)
    def DoCalcVectorOutput(self, context, u, x, y):
        y[:] = x


class TestPyplotVisualizer(unittest.TestCase):

    def test_simple_visualizer(self):
        builder = DiagramBuilder()
        system = builder.AddSystem(SimpleContinuousTimeSystem())
        logger = builder.AddSystem(VectorLogSink(1))
        builder.Connect(system.get_output_port(0), logger.get_input_port(0))
        visualizer = builder.AddSystem(TestVisualizer(1))
        builder.Connect(system.get_output_port(0),
                        visualizer.get_input_port(0))
        diagram = builder.Build()

        context = diagram.CreateDefaultContext()
        context.SetContinuousState([0.9])

        simulator = Simulator(diagram, context)
        simulator.AdvanceTo(.1)

        ani = visualizer.animate(logger.FindLog(context), repeat=True)
        self.assertIsInstance(ani, animation.FuncAnimation)

    def test_trajectory(self):
        builder = DiagramBuilder()
        visualizer = builder.AddSystem(TestVisualizer(1))
        ppt = PiecewisePolynomial.FirstOrderHold(
            [0., 1.], [[2., 3.], [2., 1.]])
        ani = visualizer.animate(ppt)
        self.assertIsInstance(ani, animation.FuncAnimation)

    def test_recording(self):
        visualizer = PyPlotVisualizer()

        # Assert that we start with no recordings. This uses private API for
        # testing _recorded_contexts and should not be used publicly.
        self.assertEqual(len(visualizer._recorded_contexts), 0)
        visualizer.start_recording()

        # Artificially produce some specific contexts.
        times = [0.003, 0.2, 1.1, 1.12]
        context = visualizer.AllocateContext()
        for time in times:
            context.SetTime(time)
            visualizer.Publish(context)

        # Check that there are now recorded contexts with matching times.
        visualizer.stop_recording()
        self.assertEqual(len(visualizer._recorded_contexts), len(times))
        for i, time in enumerate(times):
            self.assertEqual(time, visualizer._recorded_contexts[i].get_time())

        ani = visualizer.get_recording_as_animation()
        self.assertIsInstance(ani, animation.FuncAnimation)

        visualizer.reset_recording()
        self.assertEqual(len(visualizer._recorded_contexts), 0)
