"""
Provides a number of tcl/tk-based user interfaces helpful for manipulation (
and potentially other robotics) applications.
"""

import Tkinter as tk
import numpy as np

from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.multibody_tree import JointIndex
from pydrake.systems.framework import VectorSystem


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

        self.root = tk.Tk()
        self.root.title(title)
        self.slider = []
        k = 0
        for i in range(0, robot.num_joints()):
            joint = robot.tree().get_joint(JointIndex(i))
            low = joint.lower_limits()
            upp = joint.upper_limits()
            for j in range(0, joint.num_positions()):
                self.slider.append(tk.Scale(self.root,
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
        self.root.update_idletasks()
        self.root.update()

    def _DoCalcVectorOutput(self, context, unused, unused2, output):
        for i in range(0, len(self.slider)):
            output[i] = self.slider[i].get()
