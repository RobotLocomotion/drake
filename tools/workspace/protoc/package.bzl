# -*- python -*-

load("@drake//tools/workspace:which.bzl", "which")

def protoc_repository(name):
    # Find the protoc binary on $PATH.
    which(
        name = name,
        command = "protoc",
    )
