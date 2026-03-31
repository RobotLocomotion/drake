# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

# ruff: noqa: F821 (undefined-name). This file is only a fragment.

import functools
import operator
import sys
import threading
import typing
import uuid


def logical_and(*formulas):
    assert len(formulas) >= 1, "Must supply at least one operand"
    return functools.reduce(__logical_and, formulas)


def logical_or(*formulas):
    assert len(formulas) >= 1, "Must supply at least one operand"
    return functools.reduce(__logical_or, formulas)


def _reduce_add(*args):
    return functools.reduce(operator.add, args)


def _reduce_mul(*args):
    return functools.reduce(operator.mul, args)


class _Pickler:
    """Private implementation singleton for (un)pickling symbolic.Variable."""

    def __init__(self):
        self._lock = threading.Lock()
        self.reset()

    def reset(self):
        """To pickle symbolic.Variable, pydrake needs to maintain an in-memory
        lookup table of all variables ever pickled or unpicked in the current
        process. The table preserves the invariant that variable equality is
        tied to object identity.

        With a heavy pickling, the table might grow to consume too much memory.
        This function clears the table, releasing the memory. Variables that are
        pickled or unpickled on different sides of the call to reset_pickler()
        may not compare as equal, so this should only be called upon reaching an
        "epoch" of symbolic.Variable scope or lifetime, e.g., resetting an
        experiment.
        """
        self._guid_to_var = dict()
        self._var_to_guid = dict()

    def _pickle_variable(self, var: Variable) -> tuple:
        guid = self._var_to_guid.get(var.get_id())
        if guid is None:
            with self._lock:
                guid = self._var_to_guid.get(var.get_id())
                if guid is None:
                    guid = uuid.uuid4()
                    self._var_to_guid[var.get_id()] = guid
                    self._guid_to_var[guid] = var
        return (
            guid,
            var.get_name(),
            var.get_type(),
        )

    def _unpickle_variable(self, var_pickled: tuple) -> Variable:
        guid, var_name, var_type = var_pickled
        var = self._guid_to_var.get(guid)
        if var is None:
            with self._lock:
                var = self._guid_to_var.get(guid)
                if var is None:
                    var = Variable(name=var_name, type=var_type)
                    self._var_to_guid[var] = guid
                    self._guid_to_var[guid] = var
        return var


_PICKLER = _Pickler()
reset_pickler = _PICKLER.reset
_pickle_variable = _PICKLER._pickle_variable
_unpickle_variable = _PICKLER._unpickle_variable

# Drake's SymPy support is loaded lazily (on demand), so that Drake does not
# directly depend on SymPy. The implementation lives in `_symbolic_sympy.py`.
_symbolic_sympy_defer = None


def to_sympy(
    x: float | int | bool | Variable | Expression | Formula,
    *,
    memo: dict = None,
) -> typing.Union[float, int, bool, "sympy.Expr"]:
    """Converts a pydrake object to the corresponding SymPy Expr.

    Certain expressions are not supported and will raise NotImplementedError.
    (Most acutely, note that `int` is not yet supported.)

    This function aims to support the latest contemporaneous version of SymPy
    as of Drake's release date.

    Args:
        x: The pydrake object to be converted.
        memo: (Optional) Mapping between Drake variables and SymPy variables.
            Converting a ``pydrake.symbolic.Variable`` to SymPy will look up
            the Drake variable's ``v.get_id()`` as a key in this dictionary.
            If a value is found, it will be used as the SymPy atom for that
            Drake variable.  Otherwise, a new SymPy variable will be created
            and used, and both mappings ``{drake_var.get_id(): sympy_var}``
            and ``{sympy_var: drake_var}`` will be inserted into ``memo``.

    See also :meth:`pydrake.symbolic.from_sympy`.
    """
    global _symbolic_sympy_defer
    if _symbolic_sympy_defer is None:
        from pydrake.symbolic import _symbolic_sympy as _symbolic_sympy_defer
    if memo is None:
        memo = dict()
    return _symbolic_sympy_defer._to_sympy(x, memo=memo)


def from_sympy(
    x: typing.Union[float, int, bool, "sympy.Expr"], *, memo: dict = None
) -> float | int | bool | Variable | Expression | Formula:
    """Converts a SymPy Expr to the corresponding pydrake object.

    Certain expressions are not supported and will raise NotImplementedError.

    This function aims to support the latest contemporaneous version of SymPy
    as of Drake's release date.

    Args:
        x: The SymPy object to be converted.
        memo: (Optional) Mapping between SymPy variables and Drake variables.
            Converting a SymPy variable to a ``pydrake.symbolic.Variable`` will
            look up the SymPy variable as a key in this dictionary.
            If a value is found, then it will be used as the Drake variable.
            If no value is found, then (for now) raises an exception. In the
            future, we might support automatically creating variables, but
            that is not yet implemented.

    See also :meth:`pydrake.symbolic.to_sympy`.
    """
    global _symbolic_sympy_defer
    if _symbolic_sympy_defer is None:
        from pydrake.symbolic import _symbolic_sympy as _symbolic_sympy_defer
    if memo is None:
        memo = dict()
    return _symbolic_sympy_defer._from_sympy(x, memo=memo)


# We must be able to do `from pydrake.symbolic import _symbolic_sympy` so we
# need `pydrake.symbolic` to be a Python package, not merely a module. (See
# https://docs.python.org/3/tutorial/modules.html for details.) The way to
# designate something as a package is to define its `__path__` attribute.
__path__ = [sys.modules["pydrake"].__path__[0] + "/symbolic"]
