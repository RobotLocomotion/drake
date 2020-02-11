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

# When running from python, turn DRAKE_ASSERT and DRAKE_DEMAND failures into
# SystemExit, instead of process aborts.  See RobotLocomotion/drake#5268.
# We specifically load `common` prior to loading any other pydrake modules,
# in order to get assertion configuration done as early as possible.
from . import common
from .common.deprecation import ModuleShim

__all__ = ['common', 'getDrakePath']
common.set_assertion_failure_to_throw_exception()


def getDrakePath():
    # Compatibility alias.
    return os.path.abspath(common.GetDrakePath())


def _execute_extra_python_code(m):
    # See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
    # rationale.
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
    extra_path = [top_module_dir] + module_path[1:-1] + [
        "_{}_extra.py".format(module_path[-1])]
    extra_filename = os.path.join(*extra_path)
    with open(extra_filename) as f:
        _code = compile(f.read(), extra_filename, 'exec')
        exec(_code, m.__dict__, m.__dict__)


def _setattr_kwargs(obj, kwargs):
    # For `ParamInit` in `pydrake_pybind.h`.
    for name, value in kwargs.items():
        setattr(obj, name, value)


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
    # Naively check if `torch` is using RTLD_GLOBAL.
    torch = sys.modules.get("torch")
    if torch is None:
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
