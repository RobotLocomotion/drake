"""
Provides support for using ipywidgets with the systems framework, and a number
of useful widget systems.
"""

import asyncio
import numpy as np
import sys
from ipywidgets import FloatSlider, Layout
from IPython import get_ipython
from IPython.display import display

from pydrake.common.value import AbstractValue
from pydrake.systems.framework import LeafSystem, BasicVector
from pydrake.math import RigidTransform, RollPitchYaw


class PoseSliders(LeafSystem):
    """
    Provides a set of ipywidget sliders (to be used in a Jupyter notebook) with
    on one slider for each of roll, pitch, yaw, x, y, and z.  This can be used,
    for instance, as an interface to teleoperate the end-effector of a robot.

    System YAML
        name: PoseSliders
        output_ports:
        - pose
    """
    def __init__(self, visible=[True] * 6):
        """
        Args:
            visible: A six element boolean list that specifies whether the
                     slider corresponding slider (ordered by roll, pitch, yaw,
                     x, y, z) is displayed or not.

        See SetRpyRange, and SetXyzRange to set the slider limits, and SetPose
        or SetRpy and SetXyz to set their value.
        """
        LeafSystem.__init__(self)
        self.DeclareAbstractOutputPort(
            "pose", lambda: AbstractValue.Make(RigidTransform()),
            self.DoCalcOutput)

        # Note: This timing affects the keyboard teleop performance. A larger
        #       time step causes more lag in the response.
        self.DeclarePeriodicPublish(0.01, 0.0)

        self.roll = FloatSlider(min=-np.pi,
                                max=np.pi,
                                value=0.0,
                                step=0.01,
                                continuous_update=True,
                                description="roll",
                                layout=Layout(width='90%'))
        self.pitch = FloatSlider(min=-np.pi,
                                 max=np.pi,
                                 value=0.0,
                                 step=0.01,
                                 continuous_update=True,
                                 description="pitch",
                                 layout=Layout(width='90%'))
        self.yaw = FloatSlider(min=-np.pi,
                               max=np.pi,
                               value=0.0,
                               step=0.01,
                               continuous_update=True,
                               description="yaw",
                               layout=Layout(width='90%'))
        self.x = FloatSlider(min=-1.0,
                             max=1.0,
                             value=0.,
                             step=0.01,
                             continuous_update=True,
                             description="x",
                             orient='horizontal',
                             layout=Layout(width='90%'))
        self.y = FloatSlider(min=-1.0,
                             max=1.0,
                             value=0.,
                             step=0.01,
                             continuous_update=True,
                             description="y",
                             layout=Layout(width='90%'))
        self.z = FloatSlider(min=-1.0,
                             max=1.0,
                             value=0.,
                             step=0.01,
                             continuous_update=True,
                             description="z",
                             layout=Layout(width='90%'))
        if visible[0]:
            display(self.roll)
        if visible[1]:
            display(self.pitch)
        if visible[2]:
            display(self.yaw)
        if visible[3]:
            display(self.x)
        if visible[4]:
            display(self.y)
        if visible[5]:
            display(self.z)

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
        self.roll.value = rpy.roll_angle()
        self.pitch.value = rpy.pitch_angle()
        self.yaw.value = rpy.yaw_angle()

    def SetRpyRange(self, min, max):
        """
        Sets the minimum and maximum values of the sliders for roll, pitch, and
        yaw.
        Args:
            min: An instance of drake.math.RollPitchYaw for the lower limit.
            max: An instance of drake.math.RollPitchYaw for the upper limit.
        """
        self.roll.min = min.roll_angle()
        self.pitch.min = min.pitch_angle()
        self.yaw.min = min.yaw_angle()
        self.roll.max = max.roll_angle()
        self.pitch.max = max.pitch_angle()
        self.yaw.max = max.yaw_angle()

    def SetXyz(self, xyz):
        """
        Sets the current value of the sliders for x, y, and z.
        Args:
            xyz: A 3 element iterable object with x, y, z.
        """
        self.x.value = xyz[0]
        self.y.value = xyz[1]
        self.z.value = xyz[2]

    def SetXyzRange(self, min, max):
        """
        Sets the minimum and maximum values of the sliders for x, y, and z.
        Args:
            min: 3 element iterable object representing the (elementwise)
                 x,y,z, minimum values.
            max: 3 element iterable object representing the (elementwise)
                 x,y,z, maximum values.
        """
        self.x.min = min[0]
        self.y.min = min[1]
        self.z.min = min[2]
        self.x.max = max[0]
        self.y.max = max[1]
        self.z.max = max[2]

    def DoPublish(self, context, event):
        """
        Allow the ipython kernel to process the event queue.
        """
        update_widgets()

    def DoCalcOutput(self, context, output):
        """
        Constructs the output values from the widget elements.
        """
        output.set_value(RigidTransform(
            RollPitchYaw(self.roll.value, self.pitch.value, self.yaw.value),
            [self.x.value, self.y.value, self.z.value]))


# Note: The implementation below was inspired by
# https://github.com/Kirill888/jupyter-ui-poll , though I suspect it can be
# optimized.
#
# For reference,
# https://ipywidgets.readthedocs.io/en/latest/examples/Widget%20Asynchronous.html  # noqa
# describes the problem but does not offer a solution.
def update_widgets(num_ui_events_to_process=1):
    """
    Allows the kernel to process GUI events.  This is required in order to
    process ipywidget updates inside a simulation loop.
    """

    shell = get_ipython()
    # Ok to do nothing if running from console
    if shell is None:
        return
    kernel = shell.kernel
    events = []
    kernel.shell_handlers['execute_request'] = lambda *e: events.append(e)
    current_parent = (kernel._parent_ident, kernel._parent_header)

    for _ in range(num_ui_events_to_process):
        # ensure stdout still happens in the same cell
        kernel.set_parent(*current_parent)
        kernel.do_one_iteration()
        kernel.set_parent(*current_parent)

    kernel.shell_handlers['execute_request'] = kernel.execute_request

    def _replay_events(shell, events):
        kernel = shell.kernel
        sys.stdout.flush()
        sys.stderr.flush()
        for stream, ident, parent in events:
            kernel.set_parent(ident, parent)
            if kernel._aborting:
                kernel._send_abort_reply(stream, parent, ident)
            else:
                kernel.execute_request(stream, ident, parent)

    loop = asyncio.get_event_loop()
    if loop.is_running():
        loop.call_soon(lambda: _replay_events(shell, events))
    else:
        warn('Automatic execution of scheduled cells only works with '
             'asyncio-based ipython')
