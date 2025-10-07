load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "1f2300db40caa341786bffdad5ee02966c5f8d0a",
        sha256 = "641a6736f6dc15651e16eab8019402580fa4f80868cb1b9f1e7e06c919860b5b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
