# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "2346e9f6c8fb15e6ea8d8f8bffd0a23854e650ce",
        sha256 = "c4aabeb47bf1332a7b407766800aac3944e42a4d67259a766de15eb4417df8ee",  # noqa
        build_file = "@drake//tools/workspace/dm_control:package.BUILD.bazel",
        mirrors = mirrors,
    )
