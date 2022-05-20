# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

# TODO(jwnimmer-tri) Rename this to match upstream gz-utils.
def ignition_utils_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gazebosim/gz-utils",
        # When updating this commit, also remember to adjust the PROJECT_*
        # constants in ./package.BUILD.bazel to match the new version number.
        commit = "ignition-utils1_1.0.0",
        sha256 = "55d3285692392f9493a35b680f275ec116584baedeef90de57d2b03dfd952d9d",  # noqa
        build_file = "@drake//tools/workspace/ignition_utils:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
