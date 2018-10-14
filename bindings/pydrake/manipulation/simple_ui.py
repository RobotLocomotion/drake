
import Tkinter as tk
import numpy as np

from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.multibody_tree import JointIndex
from pydrake.systems.framework import VectorSystem


# Provides a simple tcl/tk gui with one slider per joint of the MultibodyPlant.
#
# @system{ JointSliders,
#          , # no input ports
#          @output_port{joint positions} }
#
class JointSliders(VectorSystem):
    # @param robot is a MultibodyPlant.
    # @param lower_limit is a scalar or vector of length robot.num_positions().
    #                    The lower limit of the slider will be the maximum
    #                    value of this number and any limit specified in the
    #                    Joint.
    # @param upper_limit is a scalar or vector of length robot.num_positions().
    #                    The upper limit of the slider will be the minimum
    #                    value of this number and any limit specified in the
    #                    Joint.
    # @param resolution is a scalar or vector of length robot.num_positions()
    #                   that specifies the discretization of the slider.  Use
    #                   -1 (the default) to disable any rounding.
    # @param update_period_sec Specifies how often the gui update() method gets
    #                          called.
    # @param title is the string that appears as the title of the gui window.
    #              Use None to generate a default title.
    def __init__(self, robot, lower_limit=-10., upper_limit=10.,
                 resolution=-1, update_period_sec=0.1,
                 title=None):
        VectorSystem.__init__(self, 0, robot.num_positions())
        self._DeclarePeriodicPublish(update_period_sec, 0.0)

        if hasattr(lower_limit, "__len__"):
            assert(len(lower_limit) == robot.num_positions())
        else:
            lower_limit = lower_limit*np.ones(robot.num_positions())

        if hasattr(upper_limit, "__len__"):
            assert(len(upper_limit) == robot.num_positions())
        else:
            upper_limit = upper_limit*np.ones(robot.num_positions())

        if hasattr(resolution, "__len__"):
            assert(len(resolution) == robot.num_positions())
        else:
            resolution = resolution*np.ones(robot.num_positions())

        if (title is None):
            if (robot.num_model_instances() == 1):
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
        assert(len(q) == len(self.slider))
        for i in range(0, len(self.slider)):
            self.slider[i].set(q[i])

    def _DoPublish(self, context, event):
        self.root.update_idletasks()
        self.root.update()

    def _DoCalcVectorOutput(self, context, unused, unused2, output):
        for i in range(0, len(self.slider)):
            output[i] = self.slider[i].get()
