# -*- python -*-

load("@drake//tools/workspace:which.bzl", "which_repository")

def python3_repository(name):
    which_repository(
        name = name,
        command = "python3",
    )
