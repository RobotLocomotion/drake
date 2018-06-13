# -*- python -*-

load("@drake//tools/workspace:which.bzl", "which_repository")

def protoc_repository(name):
    # Find the protoc binary.
    which_repository(
        name = name,
        command = "protoc",
    )
