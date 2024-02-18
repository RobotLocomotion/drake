load("//tools/workspace:github.bzl", "github_archive")

def ros_xacro_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ros/xacro",
        commit = "2.0.10",
        sha256 = "dd9112a1bd955ba987c2a29d79aebf5a2892dfd362d661b7a345e28c0f5807b3",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/disable-import-warning.patch",
        ],
        mirrors = mirrors,
    )
