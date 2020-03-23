try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk

import numpy as np

from pydrake.systems.framework import VectorSystem


class SystemSliders(VectorSystem):
    """
    Provides a set of tcl/tk-based sliders to control the values of a
    system's input port. Each value of the input port gets a float slider
    @system{ SystemSliders,
            , # no input ports
            @output_port{slider_positions} }
    """
    def __init__(self, input_port, slider_names=None,
                 lower_limit=-10., upper_limit=10.,
                 resolution=-1, length=200, update_period_sec=0.005,
                 window=None, title=None):
        """
        Args:
            input_port:  An input port for a system
            slider_names: A list of strings describing the names of the sliders
                         that should be displayed
            lower_limit: A scalar or vector of length input_port.size()
                         The lower limit of the slider
            upper_limit: A scalar or vector of length input_port.size()
                         The upper limit of the slider
            resolution:  A scalar or vector of length input_port.size()
                         that specifies the discretization of the slider.  Use
                         -1 (the default) to disable any rounding.
            length:      The length of the sliders (passed as an argument to
                         tk.Scale).
            update_period_sec: Specifies how often the window update() method
                         gets called.
            window:      Optionally pass in a tkinter.Tk() object to add these
                         widgets to.  Default behavior is to create a new
                         window.
            title:       The string that appears as the title of the gui
                         window.  Use None to generate a default title.  This
                         parameter is only used if a window==None.
        """
        if title is None:
            title = "System inputs"

        if window is None:
            self.window = tk.Tk()
            self.window.title(title)
        else:
            self.window = window

        self.port_size = input_port.size()

        if slider_names is None:
            slider_names = ["Index " + str(i) for i in range(self.port_size)]
        if len(slider_names) != self.port_size:
            raise ValueError("Slider names size doesn't match input port size")

        VectorSystem.__init__(self, 0, self.port_size)

        # Turn scalar inputs into vector of size input_port.size()
        def input_to_vector(x, size):
            if np.isscalar(x):
                return np.repeat(x, size)

            if len(x) == size:
                return x

            raise ValueError("Argument size doesn't match input port size")

        lower_limit = input_to_vector(lower_limit, self.port_size)
        upper_limit = input_to_vector(upper_limit, self.port_size)
        resolution = input_to_vector(resolution, self.port_size)

        # Schedule window updates in either case (new or existing window):
        self.DeclarePeriodicPublish(update_period_sec, 0.0)

        self._sliders = []

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
        Set all input port positions to the values in q.

        Args:
            q: a vector whose length is input_port.size().
        """
        if len(q) != self.port_size:
            raise ValueError("Size of q doesn't match input port size")
        for i in range(self.port_size):
            self._sliders[i].set(q[i])

    def DoPublish(self, context, event):
        self.window.update_idletasks()
        self.window.update()

    def DoCalcVectorOutput(self, context, unused, unused2, output):
        for i in range(self.port_size):
            output[i] = self._sliders[i].get()
