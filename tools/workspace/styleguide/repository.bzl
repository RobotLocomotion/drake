# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "32caa7e31045167d124430ed13c915761042284c",
        sha256 = "4ae0db426fa1d079d663cf757f405b99406e7ea62d7c6f5fd124556bf498bbbb",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
