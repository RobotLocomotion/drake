# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "ab96a7f5e375a31e25e78f281b34da83f51110eb",
        sha256 = "2d0abe99e2b6afa9740b070f3d5a91df32dcceb2694fefe287971d811747ea60",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
