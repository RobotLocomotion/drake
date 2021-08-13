# As a deprecation afforfance, if @drake//lcmtypes:lcmtypes_drake_py is a
# declared Bazel dependency of a Python library or binary, then we need to
# provide drake.lcmt_foo names as aliases for pydrake.lcmtypes.lcmt_foo.
#
# To do that, we first conditionally import drake.lcmtypes; if that succeeds,
# then we know the dependency exists, so we'll import the alias names.
#
# Note that this file should NOT be installed (`//:install` should not touch
# it); it is for use by Bazel builds only.
#
# TODO(jwnimmer-tri) When deprecated drake.lcmt_foo is removed on or after
# 2021-12-01, we should remove the code from this file, leaving it as only
# the usual "Empty file to make this a module" boilerplate.

_USES_DEPRECATED_LCMTYPES = False
try:
    # Check if @drake//lcmtypes:lcmtypes_drake_py is a declared dependency.
    from drake.lcmtypes import _MARKER
    _USES_DEPRECATED_LCMTYPES = True
except ImportError:
    pass


# If yes, then alias pydrake.lcmtypes.lcmt_foo as drake.lcmt_foo.
if _USES_DEPRECATED_LCMTYPES:
    from pydrake.lcmtypes import *

    # Warn the user about the pending removal.
    import warnings as _warnings
    _warnings.warn(
        "Drake's lcmtypes for Python are now named pydrake.lcmtypes.lcmt_foo"
        " instead of drake.lcmt_foo. The 'drake' module name is deprecated."
        " Please adjust your import statements to match the new name, and"
        " remove the BUILD dependency on @drake//lcmtypes:lcmtypes_drake_py."
        " The old module will be removed from Drake on or after 2021-12-01.",
        category=DeprecationWarning)
