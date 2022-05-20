# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

# TODO(jwnimmer-tri) Rename this to match upstream gz-math.
def ignition_math_repository(
        name,
        mirrors = None):
    # When updating this commit, also remember to adjust the PROJECT_*
    # constants in ./package.BUILD.bazel to match the new version number.
    github_archive(
        name = name,
        repository = "gazebosim/gz-math",
        commit = "ignition-math6_6.10.0",
        sha256 = "94e853e1dfba97ebec4b6152691a89af1e94660b02f4ecdf04356b763c2848bd",  # noqa
        build_file = "@drake//tools/workspace/ignition_math:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
