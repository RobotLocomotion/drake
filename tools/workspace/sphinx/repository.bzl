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
    )
