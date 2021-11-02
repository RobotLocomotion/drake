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
        commit = "ignition-math6_6.9.2",
        sha256 = "0b932e633d550ab2d25cd12c62b999a2ccfb141035fff1de87147e103e271f50",  # noqa
        build_file = "@drake//tools/workspace/ignition_math:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
