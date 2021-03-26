# -*- python -*-

load("@drake//tools/workspace:which.bzl", "which_repository")

def sphinx_repository(name):
    which_repository(
        name = name,
        command = "sphinx-build",
        additional_search_paths = [
            "/usr/share/sphinx/scripts/python3",
        ],
        allow_missing = True,
        build_epilog = 'print("DRAKE DEPRECATED: The @sphinx repository is being removed from Drake on or after 2021-07-01. Downstream projects should add it to their own WORKSPACE if needed.")',  # noqa
    )
