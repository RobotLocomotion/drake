# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def ros_xacro_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ros/xacro",
        commit = "1.14.1",
        sha256 = "4c0c38fc8699412ad0f6dd3f94655b10998df46f2784b35185043896c093cb89",  # noqa
        build_file = "@drake//tools/workspace/ros_xacro:package.BUILD.bazel",
        mirrors = mirrors,
    )
