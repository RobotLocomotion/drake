load("//tools/workspace:github.bzl", "github_archive")

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
        commit = "1.14.17",
        sha256 = "ac277534b26772f0e231df6640570a8ae0472eabd7a94614bf757ac73489ff85",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/disable-console-print.patch",
            ":patches/disable-import-warning.patch",
        ],
        mirrors = mirrors,
    )
