load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "b6d1a78b87df474b760af8c86ace9b34ab40612b",
        sha256 = "fe1fc37921983877f3b264a23005f76e18786a939a8a6108d97f3d970ccb2ccc",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/cassert.patch",
        ],
        mirrors = mirrors,
    )
