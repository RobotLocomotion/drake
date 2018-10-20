"""
Provides a number of tcl/tk-based user interfaces helpful for manipulation (
and potentially other robotics) applications.
"""

import Tkinter as tk
import numpy as np

from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.multibody_tree import JointIndex
from pydrake.systems.framework import BasicVector, LeafSystem, VectorSystem


class JointSliders(VectorSystem):
    """
    Provides a simple tcl/tk gui with one slider per joint of the
    MultibodyPlant.

    @system{ JointSliders,
             , # no input ports
             @output_port{joint positions} }
    """

    def __init__(self, robot, lower_limit=-10., upper_limit=10.,
                 resolution=-1, update_period_sec=0.1,
                 title=None):
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
            update_period_sec: Specifies how often the gui update() method gets
                         called.
            title:       The string that appears as the title of the gui
                         window.  Use None to generate a default title.
        """
        VectorSystem.__init__(self, 0, robot.num_positions())
        self._DeclarePeriodicPublish(update_period_sec, 0.0)

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

        self.window = tk.Tk()
        self.window.title(title)
        self.slider = []
        k = 0
        for i in range(0, robot.num_joints()):
            joint = robot.tree().get_joint(JointIndex(i))
            low = joint.lower_limits()
            upp = joint.upper_limits()
            for j in range(0, joint.num_positions()):
                self.slider.append(tk.Scale(self.window,
                                            from_=max(low[j], lower_limit[k]),
                                            to=min(upp[j], upper_limit[k]),
                                            resolution=resolution[k],
                                            label=joint.name(),
                                            length=200,
                                            orient=tk.HORIZONTAL))
                self.slider[k].pack()
                k += 1

        # TODO(russt): Consider resolving constraints in a slider event
        # callback.

    def set(self, q):
        """
        Set the slider positions to the values in q.

        Args:
            q: a vector of length robot.num_positions()
        """
        assert(len(q) == len(self.slider))
        for i in range(len(self.slider)):
            self.slider[i].set(q[i])

    def _DoPublish(self, context, event):
        self.window.update_idletasks()
        self.window.update()

    def _DoCalcVectorOutput(self, context, unused, unused2, output):
        for i in range(0, len(self.slider)):
            output[i] = self.slider[i].get()


class SchunkWsgButtons(LeafSystem):
    """
    Adds buttons to open/close the Schunk WSG gripper to an existing Tkinter
    window.

    @system{ SchunkWsgButtons,
             , # no input ports
             @output_port{position}
             @output_port{max_force} }
    """

    def __init__(self, window):
        """"
        Args:
            window:       An existing Tkinter.Tk() object; buttons will be
                          added to it.  This system does not call the window's
                          update methods (or main loop) -- these are typically
                          called by the creator/owner of the window.
        """
        LeafSystem.__init__(self)
        self._DeclareVectorOutputPort("position", BasicVector(1),
                                      self.CalcPositionOutput)
        self._DeclareVectorOutputPort("force_limit", BasicVector(1),
                                      self.CalcForceLimitOutput)

        self.open_button = tk.Button(window, text="Open Gripper",
                                     state=tk.DISABLED,
                                     command=self.open)
        self.open_button.pack()
        self.close_button = tk.Button(window, text="Close Gripper",
                                      command=self.close)
        self.close_button.pack()

        self.open_state = True

    def open(self):
        """
        Output a command that will open the gripper.
        """
        self.open_state = True
        self.open_button.configure(state=tk.DISABLED)
        self.close_button.configure(state=tk.NORMAL)

    def close(self):
        """
        Output a command that will close the gripper.
        """
        self.open_state = False
        self.open_button.configure(state=tk.NORMAL)
        self.close_button.configure(state=tk.DISABLED)

    def CalcPositionOutput(self, context, output):
        if self.open_state:
            # Push to joint limit specified in schunk_wsg_50.sdf.
            output.SetAtIndex(0, 0.055)
        else:
            # Closing to 0mm can smash the fingers together and keep applying
            # force even when no object is grasped.
            output.SetAtIndex(0, 0.008)

    def CalcForceLimitOutput(self, context, output):
        output.SetAtIndex(0, 40.0)
