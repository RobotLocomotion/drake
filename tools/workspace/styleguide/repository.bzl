# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "5e805ebc3bb58d4ea16db52e608f3ed9876d5a1b",
        sha256 = "913849b0450be8138c1d7df8bc8d945df7ee291e711e32a2aa05e09931df9431",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
