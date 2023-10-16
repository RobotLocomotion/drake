load(
    "//tools/workspace/crate_universe/lock/details:defs.bzl",
    _all_crate_deps = "all_crate_deps",
)

def all_crate_deps(**kwargs):
    """Returns the fully qualified labels of all crate_universe crates."""
    return _all_crate_deps(package_name = "", **kwargs)
