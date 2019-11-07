# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import numpy as np


class WorldPositionConstraint(_WorldPositionConstraint):
    def __init__(self, model, body, pts, lb, ub, tspan=None):
        pts = np.asarray(pts)
        if pts.ndim == 1:
            # TODO: deprecate this method. We should require that pts be a 2-d
            # array even when the second dimension is of length 1. When that
            # requirement is made, we can delete this entire class and the
            # renamed _WorldPositionConstraint import.
            # Ref: https://github.com/RobotLocomotion/drake/issues/4950
            pts = pts.reshape((pts.size, 1))
        if tspan is None:
            super(WorldPositionConstraint, self).__init__(
                model, body, pts, lb, ub)
        else:
            super(WorldPositionConstraint, self).__init__(
                model, body, pts, lb, ub, tspan)


class WorldPositionInFrameConstraint(_WorldPositionInFrameConstraint):
    def __init__(self, model, body, pts, T_world_to_frame, lb, ub, tspan=None):
        pts = np.asarray(pts)
        if pts.ndim == 1:
            # TODO: deprecate this method, as with WorldPositionConstraint
            # Ref: https://github.com/RobotLocomotion/drake/issues/4950
            pts = pts.reshape((pts.size, 1))
        if tspan is None:
            super(WorldPositionInFrameConstraint, self).__init__(
                model, body, pts, T_world_to_frame, lb, ub)
        else:
            super(WorldPositionInFrameConstraint, self).__init__(
                model, body, pts, T_world_to_frame, lb, ub, tspan)
