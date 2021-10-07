# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def ignition_math_repository(
        name,
        mirrors = None):
    # When updating this commit, also remember to adjust the PROJECT_*
    # constants in ./package.BUILD.bazel to match the new version number.
    # TODO(jwnimmer-tri) Until we remove the CMake config stuff on 2021-12-01,
    # any version upgrades will also need to manually fix up the *.cmake files
    # in this directory. Once remove the *.cmake installation rules, we should
    # also nix this comment.
    github_archive(
        name = name,
        repository = "ignitionrobotics/ign-math",
        commit = "ignition-math6_6.9.1",
        sha256 = "d1dbd50a1e0d8d7b1263e86f20666a2f4f0000ed8ec7b3f34ec7ced2863924f5",  # noqa
        build_file = "@drake//tools/workspace/ignition_math:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
