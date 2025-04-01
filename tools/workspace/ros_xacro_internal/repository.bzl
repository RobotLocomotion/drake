load("//tools/workspace:github.bzl", "github_archive")

def ros_xacro_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ros/xacro",
        commit = "2.0.13",
        sha256 = "6df0d44af8a6bf9f23f0536ce167a0cd7b4c01281100dfea7e17d504363b424d",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/disable-import-warning.patch",
        ],
        mirrors = mirrors,
    )
