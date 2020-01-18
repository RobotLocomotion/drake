import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np

from pydrake.systems.framework import LeafSystem, PublishEvent, TriggerType
from pydrake.systems.primitives import SignalLogger
from pydrake.trajectories import Trajectory


class PyPlotVisualizer(LeafSystem):
    """
    Base class from planar visualization that relies on pyplot.

    In the configuration set up here, this visualizer provides one
    visualization window (self.fig) with axes (self.ax). The axes can
    optionally be supplied externally to allow other visualizers to overlay
    additional information.

    Subclasses must:
    - During initialization, set up the figure bounds and register input port
    with the appropriate message type.
    - Override the draw method to parse the input and draw the robot in the
    appropriate state.
    """

    def __init__(self, draw_period=1./30, facecolor=[1, 1, 1],
                 figsize=None, ax=None):
        LeafSystem.__init__(self)

        self.set_name('pyplot_visualization')
        self.timestep = draw_period
        self.DeclarePeriodicPublish(draw_period, 0.0)

        if ax is None:
            self.fig = plt.figure(facecolor=facecolor, figsize=figsize)
            self.ax = plt.axes()
            self.fig.add_axes(self.ax)
        else:
            self.ax = ax
            self.fig = ax.get_figure()

        self.ax.axis('equal')
        self.ax.axis('off')

        self._is_recording = False
        self._recorded_contexts = []

        self._show = (matplotlib.get_backend().lower() != 'template')

        def on_initialize(context, event):
            if self._show:
                self.fig.show()

        self.DeclareInitializationEvent(
            event=PublishEvent(
                trigger_type=TriggerType.kInitialization,
                callback=on_initialize))

    def DoPublish(self, context, event):
        LeafSystem.DoPublish(self, context, event)
        if self._show:
            self.draw(context)
            self.fig.canvas.draw()
            plt.pause(1e-10)
        if self._is_recording:
            snapshot = self.AllocateContext()
            snapshot.SetTimeStateAndParametersFrom(context)
            self.FixInputPortsFrom(self, context, snapshot)
            self._recorded_contexts.append(snapshot)

    def draw(self, context):
        """Draws a single frame.
        `context` can either be a Context object, or a raw vector (for ease of
        interpolation).
        """
        raise NotImplementedError

    def start_recording(self, show=True):
        self.show = show
        self._is_recording = True

    def stop_recording(self):
        self._is_recording = False
        self.show = (matplotlib.get_backend().lower() != 'template')

    def reset_recording(self):
        self._recorded_contexts = []  # Reset recorded data.

    def _draw_recorded_frame(self, i):
        return self.draw(self._recorded_contexts[i])

    def get_recording_as_animation(self, **kwargs):
        ani = animation.FuncAnimation(fig=self.fig,
                                      func=self._draw_recorded_frame,
                                      frames=len(self._recorded_contexts),
                                      interval=1000*self.timestep,
                                      **kwargs)
        return ani

    def animate(self, log, resample=True, repeat=False):
        """
        Args:
            log: A reference to a pydrake.systems.primitives.SignalLogger that
                contains the plant state after running a simulation.
            resample: Whether we should do a resampling operation to make the
                samples more consistent in time. This can be disabled if you
                know the draw_period passed into the constructor exactly
                matches the sample timestep of the log.
            repeat: Whether the resulting animation should repeat.
        """
        if isinstance(log, SignalLogger):
            t = log.sample_times()
            x = log.data()

            if resample:
                import scipy.interpolate

                t_resample = np.arange(0, t[-1], self.timestep)
                x = scipy.interpolate.interp1d(t, x, kind='linear', axis=1)(t_resample)  # noqa
                t = t_resample

        elif isinstance(log, Trajectory):
            t = np.arange(log.start_time(), log.end_time(), self.timestep)
            x = np.hstack([log.value(time) for time in t])

        def animate_update(i):
            self.draw(x[:, i])

        ani = animation.FuncAnimation(self.fig,
                                      animate_update,
                                      t.shape[0],
                                      # Convert from s to ms.
                                      interval=1000*self.timestep,
                                      repeat=repeat)
        return ani
