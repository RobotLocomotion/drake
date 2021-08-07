# It confusing to have both drake-the-workspace and drake-the-lcmtypes-package
# on sys.path at the same time via Bazel's py_library(imports = ...).
#
# To prevent that confusion, and possibly also import errors, here in drake-
# the-workspace's package initialization we import the message types, but we
# need to do it on a best-effort basis, because not all of our python targets
# use lcmtypes -- sometimes the lcmtypes will be absent from runfiles.
#
# Note that this file should NOT be installed (`//:install` should not touch
# it).  The `//lcmtypes`-supplied init file is the correct file to install.

try:
    from pydrake.lcmtypes import *
except ImportError:
    pass
