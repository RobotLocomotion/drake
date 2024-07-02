load("//tools/workspace:github.bzl", "github_archive")

def ros_xacro_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ros/xacro",
        commit = "2.0.11",
        sha256 = "0c9b1619f1cdcf863e5a29fe8c034ae5c310e39722ff089d5d1e440c4e41967f",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/disable-import-warning.patch",
        ],
        mirrors = mirrors,
    )
