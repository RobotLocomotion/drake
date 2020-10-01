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
        commit = "ignition-math6_6.6.0",
        sha256 = "8fd06206e7e3d28a9c8ff54f7c9eaa3f01178170bd31ad57c6bc672ecb0c3645",  # noqa
        build_file = "@drake//tools/workspace/ignition_math:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
