try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk

import numpy as np

from pydrake.systems.framework import (PublishEvent, VectorSystem)


class SystemSliders(VectorSystem):
    """
    Provides a set of tcl/tk-based sliders intended to control the values
    of a system's input port. Each value of the input port gets a float slider.
    Each slider has can be customized with lower_limit, upper_limit, and
    resolution. These values can be set generally across all sliders or
    per slider with a vector of length port_size.

    Note: The sliders can only be manipulated while the simulation is
    advancing time.

    @warning: Do not close the slider GUI while running a simulation. It will
    cause the simulation to crash.

    .. pydrake_system::

        name: SystemSliders
        output_ports:
        - slider_positions
    """
    def __init__(self, port_size, slider_names=None,
                 lower_limit=-10., upper_limit=10.,
                 resolution=-1, length=200, update_period_sec=0.0166,
                 window=None, title="System inputs"):
        """
        Args:
            port_size:   Size of the input port that's being controlled. This
                         is the number of sliders that will show up.
            slider_names: A list of strings describing the names of the sliders
                         that should be displayed.
            lower_limit: The value(s) for the lower limits of each slider. See
                         class documentation for more details.
            upper_limit: The value(s) for the upper limits of each slider. See
                         class documentation for more details.
            resolution:  A scalar or vector of length port_size
                         that specifies the discretization that the slider will
                         be rounded to. Use -1 (the default) to disable any
                         rounding. For example, a resolution of 0.1 will round
                         to the nearest 0.1. See class documentation for more
                         details.
            length:      The length of the sliders in pixels.
            update_period_sec: Specifies how often the window update() method
                         gets called. Smaller values will theoretically make
                         GUI values available to the simulation more quickly,
                         but may require the simulation to take more steps than
                         necessary. The default value is suitable for most
                         applications.
            window:      Optionally pass in a tkinter.Tk() object to add these
                         widgets to.  Default behavior is to create a new
                         window.
            title:       The string that appears as the title of the gui
                         window.  Default title is "System sliders"  This
                         parameter is only used if window is None.
        """
        VectorSystem.__init__(self, 0, port_size)

        if window is None:
            self.window = tk.Tk()
            self.window.title(title)
        else:
            self.window = window

        self.port_size = port_size

        if slider_names is None:
            slider_names = ["Index " + str(i) for i in range(self.port_size)]
        if len(slider_names) != self.port_size:
            raise ValueError(
                f"Slider names size ({len(slider_names)}) doesn't "
                f"match port size ({self.port_size})")

        def input_to_vector(x, desc):
            """
            Turn scalar inputs into vector of size self.port_size.
            Throws error if vector input is the wrong size,
            otherwise returning the vector.

            Args:
                x: scalar or vector input.
                desc: string describing the vector, used in error message.
            """
            if np.isscalar(x):
                return np.repeat(x, self.port_size)

            if len(x) == self.port_size:
                return x

            raise ValueError(
                f"Size of {desc} ({len(x)}) doesn't "
                f"match port size ({self.port_size})"
            )

        lower_limit = input_to_vector(lower_limit, "lower_limit")
        upper_limit = input_to_vector(upper_limit, "upper_limit")
        resolution = input_to_vector(resolution, "resolution")

        # Schedule window updates in either case (new or existing window):
        self.DeclarePeriodicEvent(update_period_sec, 0.0,
                                  PublishEvent(self._update_window))

        self._sliders = []

        # TODO: support a scroll bar for larger input sizes
        for i in range(self.port_size):
            slider = tk.Scale(self.window,
                              from_=lower_limit[i],
                              to=upper_limit[i],
                              resolution=resolution[i],
                              label=slider_names[i],
                              length=length,
                              orient=tk.HORIZONTAL)
            slider.pack()
            self._sliders.append(slider)

    def set_position(self, q):
        """
        Sets all slider positions to the values in q.

        Args:
            q: a vector whose length is self.port_size.
        """
        if len(q) != self.port_size:
            raise ValueError(
                f"Size of q ({len(q)}) doesn't match input "
                f"port size ({self.port_size})")
        for i in range(self.port_size):
            self._sliders[i].set(q[i])

    # TODO(SeanCurtis-TRI): This is technically making a change to system
    # "state" (the state of the tk window). It's not contained in the
    # context, which is why we can get away with it. However, it would
    # be better to add a discrete state vector that gets updated values
    # from the sliders (and handles the slider update at that time) and
    # update that in a DiscreteUpdate.
    def _update_window(self, context, event):
        # GUI functionality is not automatically tested
        # Must manually test this function to ensure updates run properly
        self.window.update_idletasks()
        self.window.update()

    def DoCalcVectorOutput(self, context, unused, unused2, output):
        for i in range(self.port_size):
            output[i] = self._sliders[i].get()
