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
        commit = "1.14.16",
        sha256 = "8031a544dded557c9c32345a3fceff416f703b615f9082b2c3aefe1b7612ad90",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/disable-console-print.patch",
            ":patches/disable-import-warning.patch",
        ],
        mirrors = mirrors,
    )
