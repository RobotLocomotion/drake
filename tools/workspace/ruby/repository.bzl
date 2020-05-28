# -*- python -*-

load("@drake//tools/workspace:which.bzl", "which_repository")

def ruby_repository(name):
    print("The @ruby repository is deprecated and will be removed from " +
          "Drake on or after 2020-09-01.")

    # Find the ruby binary.
    which_repository(
        name = name,
        command = "ruby",
    )
