"""
Bindings for ``math``, including overloads for scalar types and basic SE(3)
representations.

Note that arrays of symbolic scalar types, such as ``Variable`` and
``Expression``, are exposed using ``ndarray[object]``, and as such logical
operations are constrained to return boolean values given NumPy's
implementation; this is not desirable, as one should really get a ``Formula``
object. As a workaround, this module provides the following vectorized
operators, following suit with the ``operator`` builtin module:
``lt``, ``le``, ``eq``, ``ne``, ``ge``, and ``gt``.

As an example::

    >>> x = np.array([Variable("x0"), Variable("x1")])
    >>> y = np.array([Variable("y0"), Variable("y1")])
    >>> x >= y
    # This should throw a RuntimeError
    >>> ge(x, y)
    array([<Formula "(x0 >= y0)">, <Formula "(x1 >= y1)">], dtype=object)

"""

# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import operator

import numpy as np

from pydrake.autodiffutils import AutoDiffXd as _AutoDiffXd
from pydrake.symbolic import Expression as _Expression

# As mentioned in top-level, add generic logical operators as ufuncs so that we
# may do comparisons on arrays of any scalar type, without restriction on the
# output type. These are added solely to work around #8315, where arrays of
# Expression can't use direct logical operators (e.g. `<=`) since the output
# type is not bool.
# N.B. Defined in order listed in Python documentation:
# https://docs.python.org/3.6/library/operator.html
lt = np.vectorize(operator.lt)
le = np.vectorize(operator.le)
eq = np.vectorize(operator.eq)
ne = np.vectorize(operator.ne)
ge = np.vectorize(operator.ge)
gt = np.vectorize(operator.gt)


def _indented_repr(o):
    """Returns repr(o), with any lines beyond the first one indented +2."""
    return repr(o).replace("\n", "\n  ")


def _remove_float_suffix(typename):
    suffix = "_[float]"
    if typename.endswith(suffix):
        return typename[:-len(suffix)]
    return typename


def _rotation_matrix_repr(R):
    M = R.matrix().tolist()
    return (
        f"{_remove_float_suffix(type(R).__name__)}([\n"
        f"  {_indented_repr(M[0])},\n"
        f"  {_indented_repr(M[1])},\n"
        f"  {_indented_repr(M[2])},\n"
        f"])")


def _rigid_transform_repr(X):
    return (
        f"{_remove_float_suffix(type(X).__name__)}(\n"
        f"  R={_indented_repr(X.rotation())},\n"
        f"  p={_indented_repr(X.translation().tolist())},\n"
        f")")


RotationMatrix_[float].__repr__ = _rotation_matrix_repr
RotationMatrix_[_AutoDiffXd].__repr__ = _rotation_matrix_repr
RotationMatrix_[_Expression].__repr__ = _rotation_matrix_repr
RigidTransform_[float].__repr__ = _rigid_transform_repr
RigidTransform_[_AutoDiffXd].__repr__ = _rigid_transform_repr
RigidTransform_[_Expression].__repr__ = _rigid_transform_repr
