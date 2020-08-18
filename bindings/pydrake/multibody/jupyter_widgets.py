"""
Provides some useful systems using ipywidgets and MultibodyPlant.

This is gui code; to test changes, please manually run
//bindings/pydrake/multibody/jupyter_widgets_examples.ipynb.
"""

import numpy as np

from ipywidgets import FloatSlider, Layout

from pydrake.common.jupyter import process_ipywidget_events
from pydrake.multibody.tree import JointIndex
from pydrake.systems.framework import BasicVector, VectorSystem


class JointSliders(VectorSystem):
    """
    Provides a simple ipywidgets gui with one slider per joint of the
    MultibodyPlant.  Any positions that are not associated with joints (e.g.
    floating-base "mobilizers") are held constant at the default value
    obtained from robot.CreateDefaultContext().

    System YAML
        name: JointSliders
        output_ports:
        - positions
    """

    def __init__(self, robot, lower_limit=-10., upper_limit=10.,
                 resolution=0.01, length=200, update_period_sec=0.005):
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
                         that specifies the discretization of the slider.
            length:      The length of the sliders.
            update_period_sec: Specifies how often the window update() method
                         gets called.
        """
        VectorSystem.__init__(self, 0, robot.num_positions())

        # The widgets themselves have undeclared state.  For now, we accept it,
        # and simply disable caching on the output port.
        self.get_output_port(0).disable_caching_by_default()

        def _reshape(x, num):
            x = np.array(x)
            assert len(x.shape) <= 1
            return np.array(x) * np.ones(num)

        lower_limit = _reshape(lower_limit, robot.num_positions())
        upper_limit = _reshape(upper_limit, robot.num_positions())
        resolution = _reshape(resolution, robot.num_positions())

        # Schedule window updates in either case (new or existing window):
        self.DeclarePeriodicPublish(update_period_sec, 0.0)

        self._slider = []
        self._slider_position_start = []
        context = robot.CreateDefaultContext()
        self._default_position = robot.GetPositions(context)

        k = 0
        for i in range(0, robot.num_joints()):
            joint = robot.get_joint(JointIndex(i))
            low = joint.position_lower_limits()
            upp = joint.position_upper_limits()
            for j in range(0, joint.num_positions()):
                index = joint.position_start() + j
                self._slider_position_start.append(index)
                self._slider.append(
                    FloatSlider(value=self._default_position[index],
                                min=max(low[j], lower_limit[k]),
                                max=min(upp[j], upper_limit[k]),
                                step=resolution[k],
                                continuous_update=True,
                                description=joint.name(),
                                style={'description_width': 'initial'},
                                layout=Layout(width=f"'{length}'")))
                display(self._slider[k])
                k += 1

    def set_positions(self, q):
        """
        Set all robot positions; comparable to MultibodyPlant.SetPositions.
        Note that we only have sliders for joint positions, but the
        MultibodyPlant positions many include non-joint positions.  For
        example, models have a floating-base mobilizer by default (unless the
        MultibodyPlant explicitly welds the base to the world), and so have 7
        positions corresponding to the quaternion representation of that
        floating-base position, but not to any joint.

        Args:
            q: a vector whose length is robot.num_positions().
        """
        self._default_position[:] = q
        for i in range(len(self._slider)):
            self._slider[i].value = q[self._slider_position_start[i]]

    def set_joint_positions(self, q):
        """
        Set the slider positions to the values in q.  A list of positions which
        must be the same length as the number of positions ASSOCIATED WITH
        JOINTS in the MultibodyPlant.  This does not include, e.g.,
        floating-base coordinates, which will be assigned a default value.

        Args:
            q: a vector whose length is the same as the number of joint
            positions (also the number of sliders) for the robot.
        """
        assert(len(q) == len(self._slider))
        for i in range(len(self._slider)):
            self._slider[i].value = q[i]

    def DoPublish(self, context, event):
        process_ipywidget_events()

    def DoCalcVectorOutput(self, context, unused, unused2, output):
        output[:] = self._default_position
        for i in range(0, len(self._slider)):
            output[self._slider_position_start[i]] = self._slider[i].value
