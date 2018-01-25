# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@drake//tools/workspace:pypi.bzl", "pypi_archive")

def snopt_repository(name):
    # We directly declare a git_repository because the snopt source code
    # requires authentication, and our github_archive does not (yet, easily)
    # support that.
    git_repository(
        name = name,
        remote = "git@github.com:RobotLocomotion/snopt.git",
        commit = "0f475624131c9ca4d5624e74c3f8273ccc926f9b",
    )
