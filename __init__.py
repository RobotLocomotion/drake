# It confusing to have both drake-the-workspace and drake-the-lcmtypes-package
# on sys.path at the same time via Bazel's py_library(imports = ...).
#
# To prevent that confusion, and possibly also import errors, in our
# //lcmtypes:lcmtypes_py rule we use add_current_package_to_imports = False,
# and then here in drake-the-workspace's package initialization we use __path__
# editing to fold the two directories into the same package.
#
# We need to do it on a best-effort basis, because not all of our py_binary
# rules use lcmtypes -- sometimes the lcmtypes will be absent from runfiles.
#
# Note that this file should NOT be installed (`//:install` should not touch
# it).  The `//lcmtypes`-supplied init file is the correct file to install.

# First, probe whether we need the repair.
_needs_lcmtypes_pathing = False
try:
    import drake.lcmtypes
    _needs_lcmtypes_pathing = True
except ImportError:
    pass

# Run the repair outside of try-except, so we'll yell if it fails.
if _needs_lcmtypes_pathing:
    __path__.append(drake.lcmtypes.__path__[0] + "/drake")
    from drake.lcmtypes.drake import *
