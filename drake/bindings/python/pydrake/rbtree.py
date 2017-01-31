from __future__ import absolute_import, print_function

import numpy as np

from ._pydrake_rbtree import *
from . import autodiffutils as ad


kFixed = FloatingBaseType.kFixed
kRollPitchYaw = FloatingBaseType.kRollPitchYaw
kQuaternion = FloatingBaseType.kQuaternion


def _doKinematics(self, q, v=None):
    q = np.asarray(q)

    if isinstance(q.flat[0], ad.AutoDiffXd):
        q = ad.wrap(ad.VectorXAutoDiffXd, q)
    if v is not None:
        if isinstance(v.flat[0], ad.AutoDiffXd):
            v = ad.wrap(ad.VectorXAutoDiffXd, v)
        return self._doKinematics(q, v)
    else:
        return self._doKinematics(q)

RigidBodyTree.doKinematics = _doKinematics


def _transformPoints(self, *args, **kwargs):
    wrapped = self._transformPoints(*args, **kwargs)
    if isinstance(wrapped, ad.Matrix3XAutoDiffXd):
        return ad.unwrap(wrapped)
    else:
        return wrapped

RigidBodyTree.transformPoints = _transformPoints
