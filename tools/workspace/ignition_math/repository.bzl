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
        commit = "ignition-math6_6.8.0",
        sha256 = "07c4fa9eeaa51fe2b4ee2855ea69bb16571d7e6c315b8b0726fc4383b64303a1",  # noqa
        build_file = "@drake//tools/workspace/ignition_math:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
