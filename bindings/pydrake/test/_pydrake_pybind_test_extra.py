# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import copy


def check_copy(copy_function, obj):
    """Checks `copy_function` to ensure `obj` is equal to its copy, and that
    it is not the same instance."""
    obj_copy = copy_function(obj)
    return obj == obj_copy and obj is not obj_copy
