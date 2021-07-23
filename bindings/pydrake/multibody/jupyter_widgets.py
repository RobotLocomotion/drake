"""
Provides some useful systems using ipywidgets and MultibodyPlant.

This is gui code; to test changes, please manually run
//bindings/pydrake/multibody/examples/jupyter_widgets_examples.ipynb.
"""

import numpy as np
from functools import partial

from ipywidgets import FloatSlider, Layout
from IPython.display import display

from pydrake.common.jupyter import process_ipywidget_events
from pydrake.multibody.tree import JointIndex
from pydrake.systems.framework import BasicVector, PublishEvent, VectorSystem


class JointSliders(VectorSystem):
    """
    Provides a simple ipywidgets gui with one slider per joint of the
    MultibodyPlant.  Any positions that are not associated with joints (e.g.
    floating-base "mobilizers") are held constant at the default value
    obtained from robot.CreateDefaultContext().

    .. pydrake_system::

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
        self.DeclarePeriodicEvent(update_period_sec, 0.0,
                                  PublishEvent(self._process_event_queue))

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
                description = joint.name()
                if joint.num_positions() > 1:
                    description += f"[{j}]"
                self._slider_position_start.append(index)
                self._slider.append(
                    FloatSlider(value=self._default_position[index],
                                min=max(low[j], lower_limit[k]),
                                max=min(upp[j], upper_limit[k]),
                                step=resolution[k],
                                continuous_update=True,
                                description=description,
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

    def _process_event_queue(self, unused_context, unused_event):
        process_ipywidget_events()

    def DoCalcVectorOutput(self, context, unused, unused2, output):
        output[:] = self._default_position
        for i in range(0, len(self._slider)):
            output[self._slider_position_start[i]] = self._slider[i].value


def MakeJointSlidersThatPublishOnCallback(
    plant, publishing_system, root_context, my_callback=None,
        lower_limit=-10., upper_limit=10., resolution=0.01, length=200,
        continuous_update=True):
    """
    Creates an ipywidget slider for each joint in the plant.  Unlike the
    JointSliders System, we do not expect this to be used in a Simulator.  It
    simply updates the context and calls Publish directly from the slider
    callback.

    Args:
        plant:        A MultibodyPlant.
        publishing_system: The System whose Publish method will be called.  Can
                           be the entire Diagram, but can also be a subsystem.
        root_context: A mutable root Context of the Diagram containing both the
                      ``plant`` and the ``publishing_system``; we will extract
                      the subcontext's using `GetMyContextFromRoot`.
        my_callback:  An optional additional callback function that will be
                      called once immediately and again whenever the sliders
                      are moved, using ``my_callback(plant_context)``.  This
                      can be useful, e.g. for outputting text that prints the
                      current end-effector Jacobian, or for showing a rendered
                      image from a camera.
        lower_limit:  A scalar or vector of length robot.num_positions().
                      The lower limit of the slider will be the maximum
                      value of this number and any limit specified in the
                      Joint.
        upper_limit:  A scalar or vector of length robot.num_positions().
                      The upper limit of the slider will be the minimum
                      value of this number and any limit specified in the
                      Joint.
        resolution:   A scalar or vector of length robot.num_positions()
                      that specifies the step argument of the FloatSlider.
        length:       The length of the sliders, which will be passed as a
                      string to the CSS width field and can be any valid CSS
                      entry (e.g. 100, 100px).
        continuous_update: The continuous_update field for the FloatSliders.
                      The default ``True`` means that this method will publish/
                      callback as the sliders are dragged.  ``False`` means
                      that the publish/callback will only happen once the user
                      finishes dragging the slider.

    Returns:
        A list of the slider widget objects that are created.

    Note: Some publishers (like MeshcatVisualizer) use an initialization event
    to "load" the geometry.  You should call that *before* calling this method
    (e.g. with `meshcat.load()`).
    """

    def _broadcast(x, num):
        x = np.array(x)
        assert len(x.shape) <= 1
        return np.array(x) * np.ones(num)

    lower_limit = _broadcast(lower_limit, plant.num_positions())
    upper_limit = _broadcast(upper_limit, plant.num_positions())
    resolution = _broadcast(resolution, plant.num_positions())

    publishing_context = publishing_system.GetMyContextFromRoot(root_context)
    plant_context = plant.GetMyContextFromRoot(root_context)
    positions = plant.GetPositions(plant_context)

    # Publish once immediately.
    publishing_system.Publish(publishing_context)
    if my_callback:
        my_callback(plant_context)

    def _slider_callback(change, index):
        positions[index] = change.new
        plant.SetPositions(plant_context, positions)
        publishing_system.Publish(publishing_context)
        if my_callback:
            my_callback(plant_context)

    slider_widgets = []
    slider_num = 0
    for i in range(plant.num_joints()):
        joint = plant.get_joint(JointIndex(i))
        low = joint.position_lower_limits()
        upp = joint.position_upper_limits()
        for j in range(joint.num_positions()):
            index = joint.position_start() + j
            description = joint.name()
            if joint.num_positions() > 1:
                description += f"[{j}]"
            slider = FloatSlider(
                value=positions[index],
                min=max(low[j], lower_limit[slider_num]),
                max=min(upp[j], upper_limit[slider_num]),
                step=resolution[slider_num],
                continuous_update=continuous_update,
                description=description,
                style={'description_width': 'initial'},
                layout=Layout(width=f"'{length}'"))
            slider.observe(partial(_slider_callback, index=index),
                           names='value')
            display(slider)
            slider_widgets.append(slider)
            slider_num += 1

    return slider_widgets
