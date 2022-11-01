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
        commit = "1.14.14",
        sha256 = "9f37db16d0fd690007e655ee6ba91475e35ab3ddb0fff7d63b4a841b717442e6",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":disable-console-print.patch",
            ":disable-import-warning.patch",
        ],
        mirrors = mirrors,
    )
