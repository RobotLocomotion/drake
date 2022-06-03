# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def ros_xacro_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ros/xacro",
        # N.B. Even though 1.14.x series might not be the highest-numbered
        # release, we are using it here because it aligns with the ROS Noetic
        # version released for Ubuntu 20.04.  See:
        # https://index.ros.org/p/xacro/github-ros-xacro/#noetic
        commit = "1.14.13",
        sha256 = "e210b1e9c478d53350ef565b502ff5e53f29fd2f78eff04bb16fd465b43f4143",  # noqa
        build_file = "@drake//tools/workspace/ros_xacro_internal:package.BUILD.bazel",  # noqa
        patches = [
            "@drake//tools/workspace/ros_xacro_internal:disable-console-print.patch",  # noqa
            "@drake//tools/workspace/ros_xacro_internal:disable-import-warning.patch",  # noqa
        ],
        mirrors = mirrors,
    )
