import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from warnings import warn

from pydrake.systems.framework import LeafSystem, PublishEvent, TriggerType
from pydrake.systems.primitives import VectorLog
from pydrake.trajectories import Trajectory
from pydrake.systems._resample_interp1d import _resample_interp1d


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

    def __init__(self, draw_period=None, facecolor=[1, 1, 1],
                 figsize=None, ax=None, show=None):
        LeafSystem.__init__(self)

        # To help avoid small simulation timesteps, we use a default period
        # that has an exact representation in binary floating point; see
        # drake#15021 for details.
        default_draw_period = 1./32

        self.set_name('pyplot_visualization')
        self.timestep = draw_period or default_draw_period
        self.DeclarePeriodicPublish(self.timestep, 0.0)

        if ax is None:
            self.fig = plt.figure(facecolor=facecolor, figsize=figsize)
            self.ax = plt.axes()
            self.fig.add_axes(self.ax)
        else:
            self.ax = ax
            self.fig = ax.get_figure()

        if show is None:
            show = (matplotlib.get_backend().lower() != 'template')
        self._show = show

        self.ax.axis('equal')
        self.ax.axis('off')

        if not show:
            # This is the preferred way to support the jupyter notebook
            # animation workflow and the `inline` backend grabbing an
            # extraneous render of the figure.
            plt.close(self.fig)

        self._is_recording = False
        self._recorded_contexts = []

        def on_initialize(context, event):
            if self._show:
                self.fig.show()

        self.DeclareInitializationEvent(
            event=PublishEvent(
                trigger_type=TriggerType.kInitialization,
                callback=on_initialize))

    def DoPublish(self, context, event):
        # TODO(SeanCurtis-TRI) We want to be able to use this visualizer to
        # draw without having it part of a Simulator. That means we'd like
        # vis.Publish(context) to work. Currently, pydrake offers no mechanism
        # to declare a forced event. However, by overriding DoPublish and
        # putting the forced event callback code in the override, we can
        # simulate it.
        # We need to bind a mechanism for declaring forced events so we don't
        # have to resort to overriding the dispatcher.
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

    def start_recording(self):
        self._is_recording = True

    def stop_recording(self):
        self._is_recording = False

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

    def animate(self, log, resample=True, **kwargs):
        """
        Args:
            log: A reference to a pydrake.systems.primitives.VectorLog, or a
                pydrake.trajectories.Trajectory that contains the plant state
                after running a simulation.
            resample: Whether we should do a resampling operation to make the
                samples more consistent in time. This can be disabled if you
                know the draw_period passed into the constructor exactly
                matches the sample timestep of the log.
            Additional kwargs are passed through to FuncAnimation.
        """
        if isinstance(log, VectorLog):
            t = log.sample_times()
            x = log.data()

            if resample:
                t, x = _resample_interp1d(t, x, self.timestep)
        elif isinstance(log, Trajectory):
            t = np.arange(log.start_time(), log.end_time(), self.timestep)
            x = np.hstack([log.value(time) for time in t])

        def animate_update(i):
            self.draw(x[:, i])

        ani = animation.FuncAnimation(fig=self.fig,
                                      func=animate_update,
                                      frames=t.shape[0],
                                      # Convert from s to ms.
                                      interval=1000*self.timestep,
                                      **kwargs)
        return ani
