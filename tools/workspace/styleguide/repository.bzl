load("//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "930013ec1863a0c7bdf8766c8c16443515d2a66d",
        sha256 = "555e5f27b7e318e27f5facc74d70bdff9dde6558119fcb175706b898e4ef0fcc",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
