from pydrake.systems.framework import LeafSystem


class SchunkWsgButtons(LeafSystem):
    """
    Adds buttons to open/close the Schunk WSG gripper.

    .. pydrake_system::

        name: SchunkWsgButtons
        output_ports:
        - position
        - max_force
    """

    _BUTTON_NAME = "Open/Close Gripper"
    """The name of the button added to the meshcat UI."""

    def __init__(self, meshcat, open_position=0.107, closed_position=0.002,
                 force_limit=40):
        """"
        Args:
            open_position:   Target position for the gripper when open.
            closed_position: Target position for the gripper when closed.
                             **Warning**: closing to 0mm can smash the fingers
                             together and keep applying force even when no
                             object is grasped.
            force_limit:     Force limit to send to Schunk WSG controller.
        """
        super().__init__()
        self.meshcat = meshcat
        self.DeclareVectorOutputPort("position", 1, self.CalcPositionOutput)
        self.DeclareVectorOutputPort("force_limit", 1,
                                     self.CalcForceLimitOutput)
        self._open_button = meshcat.AddButton(self._BUTTON_NAME)
        self._open_position = open_position
        self._closed_position = closed_position
        self._force_limit = force_limit

    def CalcPositionOutput(self, context, output):
        if self.meshcat.GetButtonClicks(name=self._BUTTON_NAME) % 2 == 0:
            output.SetAtIndex(0, self._open_position)
        else:
            output.SetAtIndex(0, self._closed_position)

    def CalcForceLimitOutput(self, context, output):
        output.SetAtIndex(0, self._force_limit)
