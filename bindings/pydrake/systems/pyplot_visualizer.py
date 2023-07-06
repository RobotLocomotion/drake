import matplotlib
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

        # On Ubuntu the Debian package python3-tk is a recommended (but not
        # required) dependency of python3-matplotlib; help users understand
        # that by providing a nicer message upon a failure to import.
        try:
            import matplotlib.pyplot as plt
            self._plt = plt
        except ImportError as e:
            if e.name == 'tkinter':
                self._plt = None
            else:
                raise
        if self._plt is None:
            raise NotImplementedError(
                "On Ubuntu when using the default pyplot configuration (i.e.,"
                " the TkAgg backend) you must 'sudo apt install python3-tk' to"
                " obtain Tk support. Alternatively, you may set MPLBACKEND to"
                " something else (e.g., Qt5Agg).")

        # To help avoid small simulation time steps, we use a default period
        # that has an exact representation in binary floating point; see
        # drake#15021 for details.
        default_draw_period = 1./32

        self.set_name('pyplot_visualization')
        self.time_step = draw_period or default_draw_period
        self.DeclarePeriodicPublishNoHandler(self.time_step, 0.0)

        if ax is None:
            self.fig = self._plt.figure(facecolor=facecolor, figsize=figsize)
            self.ax = self._plt.axes()
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
            self._plt.close(self.fig)

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
            self._plt.pause(1e-10)
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
        # We defer this import to this call site to prevent the import
        # from hanging. See #18323.
        import matplotlib.animation as animation
        ani = animation.FuncAnimation(fig=self.fig,
                                      func=self._draw_recorded_frame,
                                      frames=len(self._recorded_contexts),
                                      interval=1000*self.time_step,
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
                matches the sample time step of the log.
            Additional kwargs are passed through to FuncAnimation.
        """
        if isinstance(log, VectorLog):
            t = log.sample_times()
            x = log.data()

            if resample:
                t, x = _resample_interp1d(t, x, self.time_step)
        elif isinstance(log, Trajectory):
            t = np.arange(log.start_time(), log.end_time(), self.time_step)
            x = np.hstack([log.value(time) for time in t])

        def animate_update(i):
            self.draw(x[:, i])

        # We defer this import to this call site to prevent the import
        # from hanging. See #18323.
        import matplotlib.animation as animation
        ani = animation.FuncAnimation(fig=self.fig,
                                      func=animate_update,
                                      frames=t.shape[0],
                                      # Convert from s to ms.
                                      interval=1000*self.time_step,
                                      **kwargs)
        return ani
