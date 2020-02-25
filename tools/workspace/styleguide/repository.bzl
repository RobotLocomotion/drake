# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "1a3acc7149820f1185ee3bb2dba2f1f60c6ab5f1",
        sha256 = "9b90bfb865596871bb507a5a475b237bcbd5a4df4b1f0a32bd460cd461809889",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
