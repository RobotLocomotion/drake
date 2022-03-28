# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "d72c22f3bb89178bff38728957daf62965632c2f",
        sha256 = "5584ddb74256fe344e9a9004ae92767f9ccaf5d0e6032eec4e09bc645090c1bd",  # noqa
        build_file = "@drake//tools/workspace/dm_control:package.BUILD.bazel",
        mirrors = mirrors,
    )
