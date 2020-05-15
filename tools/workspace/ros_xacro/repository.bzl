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
        commit = "1.13.5",
        sha256 = "145da225ab360e4d8cb9e2a190ff7427550e65c834919f58989284bc83d8f188",  # noqa
        build_file = "@drake//tools/workspace/ros_xacro:package.BUILD.bazel",
        patches = [
            "@drake//tools/workspace/ros_xacro:disable-console-print.patch",
            "@drake//tools/workspace/ros_xacro:disable-import-warning.patch",
        ],
        mirrors = mirrors,
    )
