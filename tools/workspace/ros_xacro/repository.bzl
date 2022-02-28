# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def ros_xacro_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ros/xacro",
        # N.B. Even though 1.13.x series is not the highst-numbered release, we
        # are using it here because it aligns with the ROS Melodic version
        # released for Ubuntu 18.04.
        commit = "1.13.17",
        sha256 = "041cb63db114c7a40331bd639645e785b1092983b58a5029c01a22f20a3b49f0",  # noqa
        build_file = "@drake//tools/workspace/ros_xacro:package.BUILD.bazel",
        patches = [
            "@drake//tools/workspace/ros_xacro:disable-console-print.patch",
            "@drake//tools/workspace/ros_xacro:disable-import-warning.patch",
        ],
        mirrors = mirrors,
    )
