# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "d05e627a1917d658ff14d4d52006c133125dbaaa",
        sha256 = "6d1c21502607cec1755f0ad402f25b2a8b6e0729752442380c0acfc3e58c7c1b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
