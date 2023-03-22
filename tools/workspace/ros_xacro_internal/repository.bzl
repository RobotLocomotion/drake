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
        commit = "1.14.15",
        sha256 = "9cf76ead44d9389f0a046c62d1f8bf28e92998786ff92b1775f0d8af751282ba",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":disable-console-print.patch",
            ":disable-import-warning.patch",
        ],
        mirrors = mirrors,
    )
