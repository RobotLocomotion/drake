"""
Provides support for using ipywidgets with the systems framework, and a number
of useful widget systems.

This is gui code; to test changes, please manually execute:

    bazel run //bindings/pydrake/systems:jupyter_widgets_examples
"""

import numpy as np
from collections import namedtuple
from functools import partial

from IPython.display import display
from ipywidgets import FloatSlider, Layout, Widget

from pydrake.common.jupyter import process_ipywidget_events
from pydrake.common.value import AbstractValue
from pydrake.systems.framework import LeafSystem, PublishEvent
from pydrake.math import RigidTransform, RollPitchYaw


class PoseSliders(LeafSystem):
    """
    Provides a set of ipywidget sliders (to be used in a Jupyter notebook) with
    one slider for each of roll, pitch, yaw, x, y, and z.  This can be used,
    for instance, as an interface to teleoperate the end-effector of a robot.

    .. pydrake_system::

        name: PoseSliders
        output_ports:
        - pose
    """
    # TODO(russt): Use namedtuple defaults parameter once we are Python >= 3.7.
    Visible = namedtuple("Visible", ("roll", "pitch", "yaw", "x", "y", "z"))
    Visible.__new__.__defaults__ = (True, True, True, True, True, True)
    MinRange = namedtuple("MinRange", ("roll", "pitch", "yaw", "x", "y", "z"))
    MinRange.__new__.__defaults__ = (-np.pi, -np.pi, -np.pi, -1.0, -1.0, -1.0)
    MaxRange = namedtuple("MaxRange", ("roll", "pitch", "yaw", "x", "y", "z"))
    MaxRange.__new__.__defaults__ = (np.pi, np.pi, np.pi, 1.0, 1.0, 1.0)
    Value = namedtuple("Value", ("roll", "pitch", "yaw", "x", "y", "z"))
    Value.__new__.__defaults__ = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def __init__(self, visible=Visible(), min_range=MinRange(),
                 max_range=MaxRange(), value=Value()):
        """
        Args:
            visible: An object with boolean elements for 'roll', 'pitch',
                     'yaw', 'x', 'y', 'z'; the intention is for this to be the
                     PoseSliders.Visible() namedtuple.  Defaults to all true.
            min_range, max_range, value: Objects with float values for 'roll',
                      'pitch', 'yaw', 'x', 'y', 'z'; the intention is for the
                      caller to use the PoseSliders.MinRange, MaxRange, and
                      Value namedtuples.  See those tuples for default values.
        """
        LeafSystem.__init__(self)
        port = self.DeclareAbstractOutputPort(
            "pose", lambda: AbstractValue.Make(RigidTransform()),
            self.DoCalcOutput)

        # The widgets themselves have undeclared state.  For now, we accept it,
        # and simply disable caching on the output port.
        # TODO(russt): consider implementing the more elaborate methods seen
        # in, e.g., LcmMessageSubscriber.
        port.disable_caching_by_default()

        # Note: This timing affects the keyboard teleop performance. A larger
        #       time step causes more lag in the response.
        self.DeclarePeriodicEvent(0.01, 0.0,
                                  PublishEvent(self._process_event_queue))

        self._roll = FloatSlider(min=min_range.roll,
                                 max=max_range.roll,
                                 value=value.roll,
                                 step=0.01,
                                 continuous_update=True,
                                 description="roll",
                                 layout=Layout(width='90%'))
        self._pitch = FloatSlider(min=min_range.pitch,
                                  max=max_range.pitch,
                                  value=value.pitch,
                                  step=0.01,
                                  continuous_update=True,
                                  description="pitch",
                                  layout=Layout(width='90%'))
        self._yaw = FloatSlider(min=min_range.yaw,
                                max=max_range.yaw,
                                value=value.yaw,
                                step=0.01,
                                continuous_update=True,
                                description="yaw",
                                layout=Layout(width='90%'))
        self._x = FloatSlider(min=min_range.x,
                              max=max_range.x,
                              value=value.x,
                              step=0.01,
                              continuous_update=True,
                              description="x",
                              orient='horizontal',
                              layout=Layout(width='90%'))
        self._y = FloatSlider(min=min_range.y,
                              max=max_range.y,
                              value=value.y,
                              step=0.01,
                              continuous_update=True,
                              description="y",
                              layout=Layout(width='90%'))
        self._z = FloatSlider(min=min_range.z,
                              max=max_range.z,
                              value=value.z,
                              step=0.01,
                              continuous_update=True,
                              description="z",
                              layout=Layout(width='90%'))

        if visible.roll:
            display(self._roll)
        if visible.pitch:
            display(self._pitch)
        if visible.yaw:
            display(self._yaw)
        if visible.x:
            display(self._x)
        if visible.y:
            display(self._y)
        if visible.z:
            display(self._z)

    def SetPose(self, pose):
        """
        Sets the current value of the sliders.
        Args:
            pose: Any viable argument for the RigidTransform
                  constructor.
        """
        tf = RigidTransform(pose)
        self.SetRpy(RollPitchYaw(tf.rotation()))
        self.SetXyz(tf.translation())

    def SetRpy(self, rpy):
        """
        Sets the current value of the sliders for roll, pitch, and yaw.
        Args:
            rpy: An instance of drake.math.RollPitchYaw
        """
        self._roll.value = rpy.roll_angle()
        self._pitch.value = rpy.pitch_angle()
        self._yaw.value = rpy.yaw_angle()

    def SetXyz(self, xyz):
        """
        Sets the current value of the sliders for x, y, and z.
        Args:
            xyz: A 3 element iterable object with x, y, z.
        """
        self._x.value = xyz[0]
        self._y.value = xyz[1]
        self._z.value = xyz[2]

    def _process_event_queue(self, context, event):
        """
        Allows the ipython kernel to process the event queue.
        """
        process_ipywidget_events()

    def DoCalcOutput(self, context, output):
        """
        Constructs the output values from the widget elements.
        """
        output.set_value(RigidTransform(
            RollPitchYaw(self._roll.value, self._pitch.value, self._yaw.value),
            [self._x.value, self._y.value, self._z.value]))


class WidgetSystem(LeafSystem):
    """
    A system that outputs the ``value``s (converted to ``float``) from
    ipywidgets.

    .. pydrake_system::

      name: WidgetSystem
      output_ports:
      - widget_group_0
      - ...
      - widget_group_{N-1}
    """

    def __init__(self, *args, update_period_sec=0.1):
        """
        Constructs the system and displays the widgets.

        Args:
            update_period_sec: Specifies how often the kernel is queried for
                widget ui events.

        An output port is created for each element in *args.  Each element of
        args must itself be an iterable collection (list, tuple, set, ...) of
        ipywidget elements, whose values will be concatenated into a single
        vector output.
        """
        LeafSystem.__init__(self)

        self._widgets = args
        for i, widget_iterable in enumerate(self._widgets):
            for w in widget_iterable:
                assert isinstance(w, Widget), (
                    "args must be collections of widgets")
                display(w)
            port = self.DeclareVectorOutputPort(
                f"widget_group_{i}",
                len(widget_iterable),
                partial(self.DoCalcOutput, port_index=i))
            port.disable_caching_by_default()

        self.DeclarePeriodicEvent(update_period_sec, 0.0,
                                  PublishEvent(self._process_event_queue))

    def _process_event_queue(self, unused_context, unused_event):
        """
        Allows the ipython kernel to process the event queue.
        """
        process_ipywidget_events()

    def DoCalcOutput(self, context, output, port_index):
        """
        Constructs the output values from the widget elements.
        """
        for i, w in enumerate(self._widgets[port_index]):
            output.SetAtIndex(i, float(w.value))
