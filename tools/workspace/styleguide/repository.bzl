# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "9852493ba124d165f4c7c47daf55292632788395",
        sha256 = "dfa72bd165f06d95978f635d94da5d9425f2da28dae47537b7961326eda9ef4b",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
