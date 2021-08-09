"""
Python bindings for
`Drake: Model-Based Design and Verification for Robotics
<https://drake.mit.edu/>`_.

This Python API documentation is a work in progress. Most of it is generated
automatically from the C++ API documentation, and thus may have some
C++-specific artifacts. For general overview and API documentation, please see
the `Doxygen C++ Documentation
<https://drake.mit.edu/doxygen_cxx/index.html>`_.

For examples and tutorials that tie in and use this API, please see
`here <https://drake.mit.edu/#tutorials-and-examples>`_.
"""

import os
import sys
import warnings

# When importing `pydrake` as an external under Bazel, Bazel will use a shared
# library whose relative RPATHs are incorrect for `libdrake.so`, and thus will
# fail to load; this issue is captured in bazelbuild/bazel#4594. As a
# workaround, we can use a library that properly links to `libdrake.so`, but in
# `{runfiles}/{workspace}/external/drake` rather than `{runfiles}/drake`.
# Once this is loaded, all of the Python C-extension libraries
# that depend on it (and its dependencies) will load properly.
# Please note that this workaround is only important when running under the
# Bazel runfiles tree. Installed `pydrake` should not have this issue.
# N.B. We do not import `external.drake.bindings.pydrake` as this may cause
# duplicate classes to be loaded (#8810). This will not be a problem once #7912
# is resolved.
try:
    import external.drake.bindings.bazel_workaround_4594_libdrake
except ImportError:
    pass


__all__ = ['getDrakePath']


def getDrakePath():
    # Compatibility alias.
    from . import common as _common
    return os.path.abspath(_common.GetDrakePath())


def _execute_extra_python_code(m):
    # See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
    # rationale.
    if m.__name__ not in sys.modules:
        # N.B. This is necessary for C++ extensions in Python 3.
        sys.modules[m.__name__] = m
    module_path = m.__name__.split(".")
    if len(module_path) == 1:
        raise RuntimeError((
            "ExecuteExtraPythonCode cannot be used with the top-level "
            "module `{}`. If you are writing modules in a downstream "
            "project, please review this thread and ensure your import is "
            "correct: https://stackoverflow.com/a/57858822/7829525"
            ).format(m.__name__))
    top_module_name = module_path[0]
    top_module_dir = os.path.dirname(sys.modules[top_module_name].__file__)
    mid_module_names = module_path[1:-1]
    base_module_name = module_path[-1]
    if base_module_name.startswith("_"):
        # Do not repeat leading `_`.
        extra_module_name = f"{base_module_name}_extra.py"
    else:
        extra_module_name = f"_{base_module_name}_extra.py"
    extra_path = [top_module_dir] + mid_module_names + [extra_module_name]
    extra_filename = os.path.join(*extra_path)
    with open(extra_filename) as f:
        _code = compile(f.read(), extra_filename, 'exec')
        exec(_code, m.__dict__, m.__dict__)


def _setattr_kwargs(obj, kwargs):
    # For `ParamInit` in `pydrake_pybind.h`.
    for name, value in kwargs.items():
        setattr(obj, name, value)


def _import_cc_module_vars(
    cc_module,
    py_module_name,
    *,
    include_private=False,
):
    # Imports the cc_module's public symbols into the named py module
    # (py_module_name), resetting their __module__ to be the py module in the
    # process.
    # Returns a list[str] of the public symbol names imported.
    py_module = sys.modules[py_module_name]
    var_list = []
    for name, value in cc_module.__dict__.items():
        if name.startswith("_") and not include_private:
            continue
        if name.startswith("__"):
            continue
        if getattr(value, "__module__", None) == cc_module.__name__:
            value.__module__ = py_module_name
        setattr(py_module, name, value)
        var_list.append(name)
    return var_list


class _DrakeImportWarning(Warning):
    pass


_RTLD_GLOBAL_WARNING = r"""
You may have already (directly or indirectly) imported `torch` which uses
`RTLD_GLOBAL`. Using `RTLD_GLOBAL` may cause symbol collisions which manifest
themselves in bugs like "free(): invalid pointer". Please consider importing
`pydrake` (and related C++-wrapped libraries like `cv2`, `open3d`, etc.)
*before* importing `torch`. For more details, see:
https://github.com/pytorch/pytorch/issues/3059#issuecomment-534676459
"""


def _check_for_rtld_global_usages():
    # Naively check if `torch` is using RTLD_GLOBAL. For more information, see
    # the above _RTLD_GLOBAL_WARNING message, #12073, and #13707.
    torch = sys.modules.get("torch")
    if torch is None:
        return False
    # This symbol was introduced in v1.5.0 (pytorch@ddff4efa2).
    # N.B. Per investigation in #13707, it seems like torch==1.4.0 also plays
    # better with pydrake. However, we will keep our warning conservative.
    using_rtld_global = getattr(torch, "USE_RTLD_GLOBAL_WITH_LIBTORCH", True)
    if not using_rtld_global:
        return False
    init_file = getattr(torch, "__file__")
    if init_file.endswith(".pyc"):
        init_file = init_file[:-1]
    if not init_file.endswith(".py"):
        return False
    with open(init_file) as f:
        init_source = f.read()
    return "sys.setdlopenflags(_dl_flags.RTLD_GLOBAL" in init_source


if _check_for_rtld_global_usages():
    warnings.warn(
        _RTLD_GLOBAL_WARNING, category=_DrakeImportWarning, stacklevel=3)
