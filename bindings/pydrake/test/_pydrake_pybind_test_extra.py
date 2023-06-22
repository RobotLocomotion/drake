# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import copy
import weakref


def check_copy(copy_function, obj):
    """Checks `copy_function` to ensure `obj` is equal to its copy, and that
    it is not the same instance."""
    obj_copy = copy_function(obj)
    return obj == obj_copy and obj is not obj_copy


def check_py_rvp_reference_internal_list(cls):
    """
    Expicitly check behavior of `py_rvp::refernce_internal` on a return list of
    pointers given class defitions used in `pydrake_pybind_test.cc`.
    """
    container = cls()
    ref = weakref.ref(container)
    item = container.a_list()[0]
    del container
    # Show that `item` is effectively keeping `container` alive.
    assert ref() is not None, "Container has been gc'd!"
    # Show that removing refcount from `item` will cause `container` to get
    # garbage collected.
    del item
    assert ref() is None, "Container has *not* been gc'd!"
    return True
