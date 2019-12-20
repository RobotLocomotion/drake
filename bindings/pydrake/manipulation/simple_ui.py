"""
Provides a number of tcl/tk-based user interfaces helpful for manipulation (
and potentially other robotics) applications.
"""

try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk
import numpy as np

from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import JointIndex
from pydrake.systems.framework import BasicVector, LeafSystem, VectorSystem


class JointSliders(VectorSystem):
    """
    Provides a simple tcl/tk gui with one slider per joint of the
    MultibodyPlant.  Any positions that are not associated with joints (e.g.
    floating-base "mobilizers") are held constant at the default value
    obtained from robot.CreateDefaultContext().

    @system{ JointSliders,
             , # no input ports
             @output_port{positions} }
    """

    def __init__(self, robot, lower_limit=-10., upper_limit=10.,
                 resolution=-1, length=200, update_period_sec=0.005,
                 window=None, title=None):
        """"
        Args:
            robot:       A MultibodyPlant.
            lower_limit: A scalar or vector of length robot.num_positions().
                         The lower limit of the slider will be the maximum
                         value of this number and any limit specified in the
                         Joint.
            upper_limit: A scalar or vector of length robot.num_positions().
                         The upper limit of the slider will be the minimum
                         value of this number and any limit specified in the
                         Joint.
            resolution:  A scalar or vector of length robot.num_positions()
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
        VectorSystem.__init__(self, 0, robot.num_positions())

        def _reshape(x, num):
            x = np.array(x)
            assert len(x.shape) <= 1
            return np.array(x) * np.ones(num)

        lower_limit = _reshape(lower_limit, robot.num_positions())
        upper_limit = _reshape(upper_limit, robot.num_positions())
        resolution = _reshape(resolution, robot.num_positions())

        if title is None:
            if robot.num_model_instances() == 1:
                title = robot.GetModelInstanceName(0) + " Joints"
            else:
                title = "Multibody Joints"

        if window is None:
            self.window = tk.Tk()
            self.window.title(title)
        else:
            self.window = window

        # Schedule window updates in either case (new or existing window):
        self.DeclarePeriodicPublish(update_period_sec, 0.0)

        self._slider = []
        self._slider_position_start = []
        context = robot.CreateDefaultContext()
        state = robot.GetPositionsAndVelocities(context)
        self._default_position = state[:robot.num_positions()]

        k = 0
        for i in range(0, robot.num_joints()):
            joint = robot.get_joint(JointIndex(i))
            low = joint.position_lower_limits()
            upp = joint.position_upper_limits()
            for j in range(0, joint.num_positions()):
                self._slider_position_start.append(joint.position_start() + j)
                self._slider.append(tk.Scale(self.window,
                                             from_=max(low[j],
                                                       lower_limit[k]),
                                             to=min(upp[j], upper_limit[k]),
                                             resolution=resolution[k],
                                             label=joint.name(),
                                             length=length,
                                             orient=tk.HORIZONTAL))
                self._slider[k].pack()
                k += 1

        # TODO(russt): Consider resolving constraints in a slider event
        # callback.

    def set_position(self, q):
        """
        Set all robot positions (corresponding to joint positions and
        potentially positions not associated with any joint) to the
        values in q.

        Args:
            q: a vector whose length is robot.num_positions().
        """
        self._default_position = q
        for i in range(len(self._slider)):
            self._slider[i].set(q[self._slider_position_start[i]])

    def set_joint_position(self, q):
        """
        Set the slider positions to the values in q.

        Args:
            q: a vector whose length is the same as the number of joint
            positions (also the number of sliders) for the robot.
        """
        assert(len(q) == len(self._default_position))
        for i in range(len(self._slider)):
            self._slider[i].set(q[i])

    def DoPublish(self, context, event):
        self.window.update_idletasks()
        self.window.update()

    def DoCalcVectorOutput(self, context, unused, unused2, output):
        output[:] = self._default_position
        for i in range(0, len(self._slider)):
            output[self._slider_position_start[i]] = self._slider[i].get()


class SchunkWsgButtons(LeafSystem):
    """
    Adds buttons to open/close the Schunk WSG gripper to an existing Tkinter
    window.

    @system{ SchunkWsgButtons,
             , # no input ports
             @output_port{position}
             @output_port{max_force} }
    """

    def __init__(self, window=None, open_position=0.107,
                 closed_position=0.002, force_limit=40,
                 update_period_sec=0.05):
        """"
        Args:
            window:          Optionally pass in a tkinter.Tk() object to add
                             these widgets to.  Default behavior is to create
                             a new window.
            update_period_sec: Specifies how often the window update() method
                             gets called.
            open_position:   Target position for the finger when open.
            closed_position: Target position for the gripper when closed.
            force_limit:     Force limit to send to Schunk WSG controller.
        """
        LeafSystem.__init__(self)
        self.DeclareVectorOutputPort("position", BasicVector(1),
                                     self.CalcPositionOutput)
        self.DeclareVectorOutputPort("force_limit", BasicVector(1),
                                     self.CalcForceLimitOutput)

        if window is None:
            self.window = tk.Tk()
            self.window.title(title)
        else:
            self.window = window

        # Schedule window updates in either case (new or existing window):
        self.DeclarePeriodicPublish(update_period_sec, 0.0)

        self._open_button = tk.Button(self.window,
                                      text="Open Gripper (spacebar)",
                                      state=tk.DISABLED,
                                      command=self.open)
        self._open_button.pack()
        self._close_button = tk.Button(self.window,
                                       text="Close Gripper (spacebar)",
                                       command=self.close)
        self._close_button.pack()

        self._open_state = True

        self._open_position = open_position
        self._closed_position = closed_position
        self._force_limit = force_limit

        self.window.bind("<space>", self._space_callback)

    def open(self):
        """
        Output a command that will open the gripper.
        """
        self._open_state = True
        self._open_button.configure(state=tk.DISABLED)
        self._close_button.configure(state=tk.NORMAL)

    def close(self):
        """
        Output a command that will close the gripper.
        """
        self._open_state = False
        self._open_button.configure(state=tk.NORMAL)
        self._close_button.configure(state=tk.DISABLED)

    def _space_callback(self, event):
        if (self._open_state):
            self.close()
        else:
            self.open()

    def DoPublish(self, context, event):
        self.window.update_idletasks()
        self.window.update()

    def CalcPositionOutput(self, context, output):
        if self._open_state:
            # Push to joint limit specified in schunk_wsg_50.sdf.
            output.SetAtIndex(0, self._open_position)
        else:
            # Closing to 0mm can smash the fingers together and keep applying
            # force even when no object is grasped.
            output.SetAtIndex(0, self._closed_position)

    def CalcForceLimitOutput(self, context, output):
        output.SetAtIndex(0, self._force_limit)
