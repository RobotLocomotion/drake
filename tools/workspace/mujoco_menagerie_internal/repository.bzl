load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "d3b4507788d70ccc150cf7a5090e99167f6159b1",
        sha256 = "33b5bea1d0f51dc8457b4c94aa0e7f04ee4f4cd7438831fd74f87e9705097fd2",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
