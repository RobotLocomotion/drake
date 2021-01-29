# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def ignition_math_repository(
        name,
        mirrors = None):
    # When updating this commit, also remember to adjust the PROJECT_*
    # constants in ./package.BUILD.bazel to match the new version number.
    github_archive(
        name = name,
        repository = "ignitionrobotics/ign-math",
        commit = "ignition-math6_6.8.0-pre2",
        sha256 = "1670453b35ab382b130cdafaf3927f14f60743bdb7811ae7b2ee82ac97d16fc3",  # noqa
        build_file = "@drake//tools/workspace/ignition_math:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
