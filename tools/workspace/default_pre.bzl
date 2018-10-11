# N.B. Define `python` first so that other repositories may use it.
# WARNING: Downstream projects must explicitly call this!
load("@drake//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")
load("@drake//tools/workspace/python:repository.bzl", "python_repository")

def add_default_pre_repositories(excludes = [], mirrors = DEFAULT_MIRRORS):
    """Declares workspace repositories that are required to load before the
    repositories specified in `add_default_repositories`.

    Args:
        excludes: list of string names of repositories to exclude; this can
          be useful if a WORKSPACE file has already supplied its own external
          of a given name.
    """
    if "python" not in excludes:
        python_repository(name = "python")
