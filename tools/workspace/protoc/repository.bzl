# -*- python -*-

load("@drake//tools/workspace:which.bzl", "which_repository")

def protoc_repository(name):
    # Find the protoc binary on $PATH.
    which_repository(
        name = name,
        command = "protoc",
    )
