from __future__ import absolute_import
import numpy as np

from ._pydrake_ik import (IKResults,
                          IKoptions,
                          InverseKin,
                          InverseKinTraj,
                          InverseKinPointwise,
                          PostureConstraint,
                          WorldEulerConstraint,
                          WorldQuatConstraint,
                          WorldGazeDirConstraint)

from ._pydrake_ik import WorldPositionConstraint as _WorldPositionConstraint
from ._pydrake_ik import WorldPositionInFrameConstraint as \
    _WorldPositionInFrameConstraint


class WorldPositionConstraint(_WorldPositionConstraint):
    def __init__(self, model, body, pts, lb, ub, tspan=None):
        pts = np.asarray(pts)
        if pts.ndim == 1:
            # TODO: deprecate this method
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
            # TODO: deprecate this method
            pts = pts.reshape((pts.size, 1))
        if tspan is None:
            super(WorldPositionInFrameConstraint, self).__init__(
                model, body, pts, T_world_to_frame, lb, ub)
        else:
            super(WorldPositionInFrameConstraint, self).__init__(
                model, body, pts, T_world_to_frame, lb, ub, tspan)
