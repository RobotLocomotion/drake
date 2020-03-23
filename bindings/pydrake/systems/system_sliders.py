try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk

import numpy as np

from pydrake.systems.framework import VectorSystem


class SystemSliders(VectorSystem):
    """
    Provides a set of tcl/tk-based sliders to control the values of a
    system's input port. Each value of the input port gets a float slider.
    Each slider has can be customized with lower_limit, upper_limit, and
    resolution. These values can be set generally across all sliders or
    per slider with a vector of length input_port.size()

    @system{ SystemSliders,
            , # no input ports
            @output_port{slider_positions} }
    """
    def __init__(self, input_port, slider_names=None,
                 lower_limit=-10., upper_limit=10.,
                 resolution=-1, length=200, update_period_sec=0.005,
                 window=None, title="System inputs"):
        """
        Args:
            input_port:  An input port for a system.
            slider_names: A list of strings describing the names of the sliders
                         that should be displayed.
            lower_limit: A scalar or vector of length input_port.size()
                         If a scalar is passed in, the scalar would be the
                         lower limit of all sliders. If a vector is passed in,
                         each slider would have a lower limit of the
                         corresponding element in the vector.
            upper_limit: A scalar or vector of length input_port.size()
                         If a scalar is passed in, the scalar would be the
                         upper limit of all sliders. If a vector is passed in,
                         each slider would have a upper limit of the
                         corresponding element in the vector.
            resolution:  A scalar or vector of length input_port.size()
                         that specifies the discretization that the slider will
                         be rounded to. Use -1 (the default) to disable any
                         rounding. For example, a resolution of 0.1 will round
                         to the neareset 0.1If a scalar is passed in, the
                         scalar would be the resolution of all sliders. If a
                         vector is passed in, each slider would have a
                         resolution of the corresponding element in the vector.
            length:      The length of the sliders in pixels(passed as an
                         argument to tk.Scale).
            update_period_sec: Specifies how often the window update() method
                         gets called. A smaller value here means that the
                         system will update with the user input more
                         frequently.
            window:      Optionally pass in a tkinter.Tk() object to add these
                         widgets to.  Default behavior is to create a new
                         window.
            title:       The string that appears as the title of the gui
                         window.  Default title is "System sliders"  This
                         parameter is only used if window is None.
        """
        VectorSystem.__init__(self, 0, input_port.size())

        if window is None:
            self.window = tk.Tk()
            self.window.title(title)
        else:
            self.window = window

        self.port_size = input_port.size()

        if slider_names is None:
            slider_names = ["Index " + str(i) for i in range(self.port_size)]
        if len(slider_names) != self.port_size:
            raise ValueError(
                f"Slider names size ({len(slider_names)}) doesn't"
                f"match input port size ({self.port_size})")

        def input_to_vector(x, size, desc):
            """
            Turn scalar inputs into vector of size size.
            Throws error if vector input is the wrong size,
            otherwise returning the vector

            Args:
                x: scalar or vector input
                size: desired vector size
                desc: string describing the vector, used in error message
            """
            if np.isscalar(x):
                return np.repeat(x, size)

            if len(x) == size:
                return x

            raise ValueError(
                f"Size of {desc} ({len(x)}) doesn't match port size ({size})"
            )

        lower_limit = input_to_vector(
            lower_limit, self.port_size, "lower_limit")
        upper_limit = input_to_vector(
            upper_limit, self.port_size, "upper_limit")
        resolution = input_to_vector(resolution, self.port_size, "resolution")

        # Schedule window updates in either case (new or existing window):
        self.DeclarePeriodicPublish(update_period_sec, 0.0)

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
                f"Size of q ({len(q)}) doesn't match input"
                f"port size ({self.port_size})")
        for i in range(self.port_size):
            self._sliders[i].set(q[i])

    # GUI functionality is not automatically tested
    # Must manually test this function to ensure updates run properly
    def DoPublish(self, context, event):
        self.window.update_idletasks()
        self.window.update()

    def DoCalcVectorOutput(self, context, unused, unused2, output):
        for i in range(self.port_size):
            output[i] = self._sliders[i].get()
