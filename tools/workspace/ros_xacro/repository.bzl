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
        commit = "1.13.14",
        sha256 = "6e25ad71600f710244048f8a0e6c19e38d11546a77dbf499fa251c2d217b53e4",  # noqa
        build_file = "@drake//tools/workspace/ros_xacro:package.BUILD.bazel",
        patches = [
            "@drake//tools/workspace/ros_xacro:disable-console-print.patch",
            "@drake//tools/workspace/ros_xacro:disable-import-warning.patch",
        ],
        mirrors = mirrors,
    )
