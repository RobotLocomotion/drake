import numpy as np

from pydrake.systems.framework import (
    LeafSystem
)


class SchunkWsgButtons(LeafSystem):
    """
    Adds buttons to open/close the Schunk WSG gripper

    .. pydrake_system::

        name: SchunkWsgButtons
        output_ports:
        - position
        - max_force
    """

    def __init__(self, meshcat, open_position=0.107,
                 closed_position=0.002, force_limit=40):
        """"
        Args:
            open_position:   Target position for the finger when open.
            closed_position: Target position for the gripper when closed.
            force_limit:     Force limit to send to Schunk WSG controller.
        """
        LeafSystem.__init__(self)
        self.DeclareVectorOutputPort("position", 1, meshcat,
                                     self.CalcPositionOutput)
        self.DeclareVectorOutputPort("force_limit", 1,
                                     self.CalcForceLimitOutput)

        self._open_button = meshcat.AddButton("Open/Close Gripper")

        self._open_state = True

        self._open_position = open_position
        self._closed_position = closed_position
        self._force_limit = force_limit

    def _space_callback(self, event):
        if (self._open_state):
            self.close()
        else:
            self.open()

    def CalcPositionOutput(self, context, meshcat, output):
        if meshcat.GetButtonClicks(name="Open/Close Gripper") % 2 == 0:
            # Push to joint limit specified in schunk_wsg_50.sdf.
            output.SetAtIndex(0, self._open_position)
        else:
            # Closing to 0mm can smash the fingers together and keep applying
            # force even when no object is grasped.
            output.SetAtIndex(0, self._closed_position)

    def CalcForceLimitOutput(self, context, output):
        output.SetAtIndex(0, self._force_limit)
