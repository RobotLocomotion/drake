# -*- python -*-

load("@drake//tools/workspace:which.bzl", "which_repository")

def sphinx_repository(name):
    which_repository(
        name = name,
        command = "sphinx-build",
        additional_search_paths = [
            "/home/eacousineau/proj/tri/repo/branches/drake/py3/drake/build/py3/bin",
        ],
    )
