# -*- python -*-

load("@drake//tools/workspace:which.bzl", "which_repository")

def ruby_repository(name):
    # Find the ruby binary.
    which_repository(
        name = name,
        command = "ruby",
    )
