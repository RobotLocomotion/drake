from __future__ import absolute_import
import numpy as np

from ._ik import (IKResults,
                  IKoptions,
                  InverseKin,
                  InverseKinTraj,
                  InverseKinPointwise,
                  PostureConstraint,
                  WorldEulerConstraint)

from ._ik import WorldPositionConstraint as _WorldPositionConstraint


class WorldPositionConstraint(_WorldPositionConstraint):
    def __init__(self, model, body, pts, lb, ub, tspan=None):
        pts = np.asarray(pts)
        if pts.ndim == 1:
            # TODO: deprecate this method
            pts = pts.reshape((pts.size, 1))
        if tspan is None:
            super(WorldPositionConstraint, self).__init__(model, body,
                                                          pts, lb, ub)
        else:
            super(WorldPositionConstraint, self).__init__(model, body,
                                                          pts, lb, ub, tspan)
