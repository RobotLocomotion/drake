# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def ignition_utils_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ignitionrobotics/ign-utils",
        # When updating this commit, also remember to adjust the PROJECT_*
        # constants in ./package.BUILD.bazel to match the new version number.
        commit = "ignition-utils1_1.0.0",
        sha256 = "bd024164f12f66e125b544602ec481f6760d5a710aeb62d34a015fb598ee5a0a",  # noqa
        build_file = "@drake//tools/workspace/ignition_utils:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
