# It's brittle to have both drake-the-workspace and drake-the-lcmtypes-package
# available on the sys.path at the same time in Bazel.
#
# To avoid import errors based on which one came first on the path, in our
# //lcmtypes:lcmtypes_drake_py rule we depend on this file, and here (in
# drake-the-workspace's package initialization) we use __path__ editing to
# fold the two directories into the same package.
#
# We need to do it on a best-effort basis, because not all of our py_binary
# rules use lcmtypes -- sometimes the lcmtypes will be absent from runfiles.
try:
    # fmt: off
    import drake.lcmtypes
    __path__.append(list(drake.lcmtypes.__path__)[0] + "/drake")
    from drake.lcmtypes.drake import *  # noqa: F403 (import-star)
    # fmt: on
except ImportError:
    pass
