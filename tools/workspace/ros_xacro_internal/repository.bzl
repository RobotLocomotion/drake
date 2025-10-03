load("//tools/workspace:github.bzl", "github_archive")

def ros_xacro_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ros/xacro",
        commit = "2.1.1",
        sha256 = "f9d94956574015427e59011d4ee113b206e9c10a27a0c01d4b08ee4268d76741",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/disable-import-warning.patch",
        ],
        mirrors = mirrors,
    )
