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
        commit = "ignition-math6_6.10.0",
        sha256 = "9e00284cd6d51afe190165b2b44258e19bd4a28781cbacf21fd6b0bae43c16aa",  # noqa
        build_file = "@drake//tools/workspace/ignition_math:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
