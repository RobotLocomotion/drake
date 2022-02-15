import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from warnings import warn

from pydrake.systems.framework import LeafSystem, PublishEvent, TriggerType
from pydrake.systems.primitives import VectorLog
from pydrake.trajectories import Trajectory

###############################################################################
def scipy_resample_log_interp1d(log, timestep):
    """
    Resample the drake systems.primitives.VectorLog evenly using a reduced
    implementation of scipy.interpolate.interp1_d.  The log.sample_times() will
    be resampled evenly according to timestep, and the log.data() will be
    recalculated via linearly interpolating with the resampled times.

    Args:
        log: The drake systems.primitives.VectorLog to resample.
        timestep: The simulation timestep, used to resample log.sample_times()
            evenly between 0 and the last time with a distance of timestep
            between each unit.

    Returns:
        (t, x): The newly sampled (times, data).
    """
    ###########################################################################
    # Copyright (c) 2001-2002 Enthought, Inc. 2003-2022, SciPy Developers.
    # All rights reserved.
    #
    # Redistribution and use in source and binary forms, with or without
    # modification, are permitted provided that the following conditions
    # are met:
    #
    # 1. Redistributions of source code must retain the above copyright
    # notice, this list of conditions and the following disclaimer.
    #
    # 2. Redistributions in binary form must reproduce the above
    # copyright notice, this list of conditions and the following
    # disclaimer in the documentation and/or other materials provided
    # with the distribution.
    #
    # 3. Neither the name of the copyright holder nor the names of its
    # contributors may be used to endorse or promote products derived
    # from this software without specific prior written permission.
    #
    # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
    # A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    # OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    # SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    # LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    # DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    # THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    ###########################################################################
    # We may not assume that the provided sample times are sorted in general.
    # We may assume, however, that sample_times() and data() have equitable
    # dimensions as this condition is asserted in
    # systems::primitives::VectorLog::AddData(time, sample).  The
    # sample_times() rows and data() cols are the same, and we will
    # process data() on the cols (axis=1).
    x = log.data()
    t = log.sample_times()
    axis = 1
    x_copy = np.array(x, copy=True)
    t_copy = np.array(t, copy=True)
    sort_indices = np.argsort(t_copy, kind="mergesort")
    t_copy = t_copy[sort_indices]
    x_copy = np.take(x_copy, sort_indices, axis=axis)

    # Now that we are sorted, resample the times and data.
    t_resample = np.arange(0, t[-1], timestep)
    t_new_indices = np.searchsorted(t_copy, t_resample)
    t_new_indices = t_new_indices.clip(1, len(t_copy) - 1).astype(int)
    lo_indices = t_new_indices - 1
    hi_indices = t_new_indices

    # Resample times.
    t_lo = t_copy[lo_indices]
    t_hi = t_copy[hi_indices]

    # Resample data.
    x_axis = (axis % x_copy.ndim)
    # TODO(svenevs) question: will x.ndim always be 2?  If so, we can just use
    # `axis` without needing to introduce `x_axis`.
    # raise RuntimeError(f"x_axis={x_axis}, x.ndim={x_copy.ndim}")
    xi = np.moveaxis(np.asarray(x_copy), x_axis, 0)
    xi = xi.reshape((xi.shape[0], -1))
    xi_lo = xi[lo_indices]
    xi_hi = xi[hi_indices]

    # Perform the linear interpolation.
    slope = (xi_hi - xi_lo) / (t_hi - t_lo)[:, None]
    xi_new = slope * (t_resample - t_lo)[:, None] + xi_lo
    x_extra_shape = x_copy.shape[:x_axis] + x_copy.shape[x_axis+1:]
    x_final = xi_new.reshape(t_resample.shape + x_extra_shape)
    # TODO(svenevs): relates: x.ndim, if it is always 2 then this will always
    # be true.
    if x_axis != 0 and t_resample.shape != ():
        nx = len(t_resample.shape)
        ny = len(x_extra_shape)
        s = (list(range(nx, nx + x_axis))
                + list(range(nx)) + list(range(nx+x_axis, nx+ny)))
        x_final = x_final.transpose(s)

    return t_resample, x_final
###############################################################################


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
                import scipy.interpolate

                t_resample = np.arange(0, t[-1], self.timestep)
                x = scipy.interpolate.interp1d(t, x, kind='linear', axis=1)(t_resample)  # noqa
                t = t_resample

                t_alt, x_alt = scipy_resample_log_interp1d(log, self.timestep)
                # Just dumping output before failing the test intentionally...
                vsep = "*" * 66
                print(vsep)
                print("Original inputs:")
                print(f"t:\n{log.sample_times()}")
                print(f"x:\n{log.data()}")
                print(vsep)
                print("Scipy Direct:")
                print(f"t:\n{t}")
                print(f"x:\n{x}")
                print(vsep)
                print("Vendored:")
                print(f"t:\n{t_alt}")
                print(f"x:\n{x_alt}")
                print(vsep)
                raise RuntimeError("failthistest")
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
