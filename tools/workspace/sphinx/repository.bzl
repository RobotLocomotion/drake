# -*- python -*-

load("@drake//tools/workspace:which.bzl", "which_repository")

def sphinx_repository(name):
    which_repository(
        name = name,
        command = "sphinx-build",
        additional_search_paths = [
            "/usr/local/opt/sphinx-doc@1.8/bin",
            "/usr/share/sphinx/scripts/python3",
        ],
    )
