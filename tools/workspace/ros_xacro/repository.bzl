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
        commit = "1.13.11",
        sha256 = "d16c56d3400a281f0e038f18377ce02ff69d3344145a308664d55ae4f54e980a",  # noqa
        build_file = "@drake//tools/workspace/ros_xacro:package.BUILD.bazel",
        patches = [
            "@drake//tools/workspace/ros_xacro:disable-console-print.patch",
            "@drake//tools/workspace/ros_xacro:disable-import-warning.patch",
        ],
        mirrors = mirrors,
    )
