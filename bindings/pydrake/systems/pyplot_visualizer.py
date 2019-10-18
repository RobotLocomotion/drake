import numpy as np

import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

from pydrake.systems.framework import (
    LeafSystem, PublishEvent, TriggerType, VectorSystem)
from pydrake.systems.primitives import SignalLogger
from pydrake.trajectories import Trajectory


class PyPlotVisualizer(LeafSystem):
    '''
        Base class from planar visualization
        that relies on pyplot.

        In the configuration set up here,
        this visualizer provides one visualization
        window (self.fig) with axes (self.ax).
        The axes can optionally be supplied externally to
        allow other visualizers to overlay additional
        information.

        Subclasses must:
        - During initialization, set up the figure
        bounds and register and input port
        with the appropriate message type.
        - Override the draw method to parse the
        input and draw the robot in the appropriate
        state.
    '''

    def __init__(self, draw_timestep=0.033333, facecolor=[1, 1, 1],
                 figsize=None, ax=None):
        LeafSystem.__init__(self)

        self.set_name('pyplot_visualization')
        self.timestep = draw_timestep
        self.DeclarePeriodicPublish(draw_timestep, 0.0)

        if ax is None:
            (self.fig, self.ax) = plt.subplots(facecolor=facecolor,
                                               figsize=figsize)
        else:
            self.ax = ax
            self.fig = ax.get_figure()

        self.ax.axis('equal')
        self.ax.axis('off')
        self.record = False
        self.recorded_contexts = []
        self.show = (matplotlib.get_backend().lower() != 'template')

        def on_initialize(context, event):
            if self.show:
                self.fig.show()

        self.DeclareInitializationEvent(
            event=PublishEvent(
                trigger_type=TriggerType.kInitialization,
                callback=on_initialize))

    def DoPublish(self, context, event):
        LeafSystem.DoPublish(self, context, event)
        if self.show:
            self.draw(context)
            self.fig.canvas.draw()
            plt.pause(1e-10)
        if self.record:
            snapshot = self.AllocateContext()
            snapshot.SetTimeStateAndParametersFrom(context)
            self.FixInputPortsFrom(self, context, snapshot)
            self.recorded_contexts.append(snapshot)

    def draw(self, context):
        '''' Return a list of updated artists if blitting is used '''
        print("SUBCLASSES MUST IMPLEMENT.")

    def start_recording(self, show=True):
        self.show = show
        self.record = True

    def stop_recording(self):
        self.record = False
        self.show = (matplotlib.get_backend().lower() != 'template')

    def reset_recording(self):
        self.recorded_contexts = []  # reset recorded data

    def draw_recorded_frame(self, i):
        return self.draw(self.recorded_contexts[i])

    def get_recording(self, **kwargs):
        ani = animation.FuncAnimation(fig=self.fig,
                                      func=self.draw_recorded_frame,
                                      frames=len(self.recorded_contexts),
                                      interval=1000*self.timestep,
                                      **kwargs)
        return ani

    def animate(self, log, resample=True, repeat=False):
        # log - a reference to a pydrake.systems.primitives.SignalLogger that
        # contains the plant state after running a simulation.
        # resample -- should we do a resampling operation to make
        # the samples more consistent in time? This can be disabled
        # if you know the draw_timestep passed into the constructor exactly
        # matches the sample timestep of the log.
        # repeat - should the resulting animation repeat?

        if type(log) is SignalLogger:
            t = log.sample_times()
            x = log.data()

            if resample:
                import scipy.interpolate

                t_resample = np.arange(0, t[-1], self.timestep)
                x = scipy.interpolate.interp1d(t, x, kind='linear', axis=1)(t_resample)  # noqa
                t = t_resample

        elif type(log) is Trajectory:
            t = np.arange(log.start_time(), log.end_time(), self.timestep)
            x = np.hstack([log.value(time) for time in t])

        def animate_update(i):
            self.draw(x[:, i])

        ani = animation.FuncAnimation(self.fig,
                                      animate_update,
                                      t.shape[0],
                                      interval=1000*self.timestep,
                                      repeat=repeat)
        return ani


class SliderSystem(VectorSystem):
    def __init__(self, ax, title, min, max):
        # 0 inputs, 1 output.
        VectorSystem.__init__(self, 0, 1)
        self.value = 0
        self.slider = Slider(ax, title, min, max, valinit=self.value)
        self.slider.on_changed(self.update)

    def update(self, val):
        self.value = val

    def DoCalcVectorOutput(self, context, unused, unused2, output):
        output[:] = self.value
